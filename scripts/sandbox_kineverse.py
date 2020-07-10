#!/usr/bin/env python
import rospy
import py_trees
import py_trees_ros
import urdf_parser_py.urdf as up

from collections import defaultdict

from py_trees          import Sequence, \
                              Selector, \
                              BehaviourTree, \
                              Blackboard,    \
                              Status

from std_msgs.msg      import Header      as HeaderMsg
from geometry_msgs.msg import PoseStamped as PoseStampedMsg

import actionlib

# Giskardpy
import giskardpy.identifier as identifier

from giskardpy.god_map     import GodMap
from giskardpy.urdf_object import hacky_urdf_parser_fix
from giskardpy.utils       import memoize, render_dot_tree
from giskardpy.plugin                          import PluginBehavior, \
                                                      GiskardBehavior
from giskardpy.plugin_update_constraints       import GoalToConstraints
from giskardpy.plugin_kinematic_sim            import KinSimPlugin
from giskardpy.plugin_instantaneous_controller import ControllerPlugin
from giskardpy.plugin_time                     import TimePlugin
from giskardpy.plugin_log_trajectory           import LogTrajPlugin
from giskardpy.plugin_plot_trajectory          import PlotTrajectory
from giskardpy.plugin_goal_reached             import GoalReachedPlugin

from giskard_msgs.msg      import MoveGoal as MoveGoalMsg, \
                                  MoveCmd  as MoveCmdMsg,  \
                                  MoveAction as MoveActionMsg, \
                                  JointConstraint     as JointConstraintMsg, \
                                  CartesianConstraint as CartesianConstraintMsg

# Kineverse
import kineverse.gradients.gradient_math as gm
from kineverse.urdf_fix import urdf_filler
from kineverse.utils    import res_pkg_path
from kineverse.model.paths                  import find_common_root
from kineverse.model.geometry_model         import GeometryModel, \
                                                   Path as KPath, \
                                                   ArticulatedObject
from kineverse.operations.urdf_operations   import load_urdf
from kineverse.visualization.bpb_visualizer import ROSBPBVisualizer

from tqdm import tqdm


class GiskardRobot(ArticulatedObject):
  def __init__(self, name):
    super(GiskardRobot, self).__init__(name)

  def get_name(self):
    return self.name

  @property
  def controlled_joints(self):
    return self.get_controlled_joints()

  def get_controlled_joints(self):
    return [gm.DiffSymbol(j) for j in self.get_joint_position_symbols()]

  def get_joint_position_symbols(self):
    out = set()
    for j in self.joints.values():
      if hasattr(j, 'position') and gm.is_symbol(j.position):
        out.add(j.position)
    return list(out)

  def get_joint_velocity_symbols(self):
    return [gm.DiffSymbol(s) for s in self.get_joint_position_symbols()]

  def get_joint_velocity_limit(self, joint_name):
    return 1.0


def create_example_goal():
  out = MoveCmdMsg()
  constraint = CartesianConstraintMsg()
  constraint.type = 'SimpleCartesianPosition' # CartesianConstraintMsg.TRANSLATION_3D
  # THIS IS REALLY UGLY
  constraint.root_link = str(KPath(identifier.robot[len(identifier.world):]) + KPath('links/base_link'))
  constraint.tip_link  = str(KPath(identifier.robot[len(identifier.world):]) + KPath('links/r_gripper_tool_frame'))
  constraint.goal.header.stamp = rospy.Time.now()
  constraint.goal.header.frame_id = str(KPath(identifier.robot[len(identifier.world):]) + KPath('links/base_link'))
  constraint.goal.pose.position.x = 0.6
  constraint.goal.pose.position.z = 0.6
  constraint.goal.pose.orientation.w = 1.0

  out.cartesian_constraints.append(constraint)

  return out


class KineverseVisualizationPlugin(GiskardBehavior):
  def __init__(self, name, topic='giskardpy/kineverse_world', base_frame='map'):
    super(KineverseVisualizationPlugin, self).__init__(name)
    self.coll_subworld = None
    self.visualizer    = ROSBPBVisualizer(topic, base_frame)

  def initialise(self):
    super(KineverseVisualizationPlugin, self).initialise()
    km = self.get_world()

    if not isinstance(km, GeometryModel):
      print('World retrieved from blackboard is not a geometry world. Thus visualization is not possible.')
      return

    robot = god_map.get_data(identifier.robot)
    self.coll_subworld = km.get_active_geometry(robot.get_joint_position_symbols())
    print('Obtained collision subworld containing {} objects.'.format(len(self.coll_subworld.names)))
    self.coll_subworld.update_world({s: 0.0 for s in self.coll_subworld.free_symbols})

  def update(self):
    state = self.get_god_map().get_data(identifier.joint_states)
    state = {KPath(identifier.joint_states + [s]).to_symbol(): v for s, v in state.items()}
    print(state)
    self.coll_subworld.update_world(state)

    self.visualizer.begin_draw_cycle('world')
    self.visualizer.draw_world('world', self.coll_subworld)
    self.visualizer.render('world')

    return Status.RUNNING

def as_execute_cb(*args):
  print('AS execute cb was called')
  pass


if __name__ == '__main__':
  rospy.init_node(u'giskard')

  action_server_name = 'sandbox_server'
  action_server = actionlib.SimpleActionServer(action_server_name, MoveActionMsg,
                                          execute_cb=as_execute_cb, auto_start=False)

  god_map = GodMap()
  blackboard = Blackboard
  blackboard.god_map = god_map
  blackboard().set(action_server_name, action_server)

  # god_map.safe_set_data(identifier.rosparam, rospy.get_param(rospy.get_name()))
  with open(res_pkg_path('package://iai_pr2_description/robots/pr2_calibrated_with_ft2.xml'), 'r') as urdf_file:
      god_map.safe_set_data(identifier.robot_description, hacky_urdf_parser_fix(urdf_file.read()))

  km = GeometryModel()
  god_map.safe_set_data(identifier.world, km)
  god_map.safe_set_data(identifier.rosparam, {})
  god_map.safe_set_data(identifier.qp_solver, {})
  god_map.safe_set_data(identifier.nWSR, None)
  god_map.safe_set_data(identifier.general_options, {})
  god_map.safe_set_data(identifier.sample_period, 0.02)


  robot_urdf = urdf_filler(up.URDF.from_xml_string(god_map.get_data(identifier.robot_description)))

  # TODO: Not cool, prefix generation exploits knowledge about the structure of robot identifier
  load_urdf(km, 
            KPath(identifier.robot[len(identifier.world):]), 
            robot_urdf, 
            reference_frame='world', 
            joint_prefix=KPath(identifier.joint_states),
            robot_class=GiskardRobot)

  km.clean_structure()
  km.dispatch_events()

  robot = god_map.get_data(identifier.robot)
  joint_symbols = robot.get_joint_position_symbols() + robot.get_joint_velocity_symbols()
  joint_paths   = [KPath(j) for j in joint_symbols]
  js_prefix = find_common_root(joint_paths)

  # print('\n'.join(km.get_data(KPath(identifier.robot[len(identifier.world):])).links.keys()))

  god_map.safe_set_data(identifier.next_move_goal, create_example_goal())
  god_map.safe_set_data(identifier.joint_states, defaultdict(float))

  p_behavior = PluginBehavior(u'planning')
  p_behavior.add_plugin(ControllerPlugin(u'controller'))
  p_behavior.add_plugin(KinSimPlugin(u'kin sim'))
  # p_behavior.add_plugin(LogTrajPlugin(u'log'))
  p_behavior.add_plugin(KineverseVisualizationPlugin(u'k_visualization'))
  # p_behavior.add_plugin(GoalReachedPlugin(u'goal reached'))
  p_behavior.add_plugin(TimePlugin(u'time'))

  root = Sequence(u'root')
  root.add_child(GoalToConstraints(u'update constraints', action_server_name))
  root.add_child(p_behavior)  


  tree = BehaviourTree(root)
  render_dot_tree(root, name='kineverse_sandbox_tree')

  tree.setup(1)

  # for x in range(100): # tqdm(range(100), desc='Ticking tree a couple times'):
    # print('Tick {}'.format(x))
  tree.tick()
