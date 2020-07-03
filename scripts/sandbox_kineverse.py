#!/usr/bin/env python
import rospy
import py_trees
import py_trees_ros
import urdf_parser_py.urdf as up

from py_trees          import Sequence, \
                              Selector, \
                              BehaviourTree, \
                              Blackboard  

from std_msgs.msg      import Header      as HeaderMsg
from geometry_msgs.msg import PoseStamped as PoseStampedMsg

# Giskardpy
import giskardpy.identifier as identifier

from giskardpy.god_map     import GodMap
from giskardpy.urdf_object import hacky_urdf_parser_fix
from giskardpy.utils       import memoize
from giskardpy.plugin_update_constraints import GoalToConstraints
from giskardpy.plugin_kinematic_sim import KinSimPlugin
from giskardpy.plugin_instantaneous_controller import ControllerPlugin

from giskard_msgs.msg      import MoveGoal as MoveGoalMsg, \
                                  MoveCmd  as MoveCmdMsg,  \
                                  JointConstraint     as JointConstraintMsg, \
                                  CartesianConstraint as CartesianConstraintMsg

# Kineverse
import kineverse.gradients.gradient_math as gm
from kineverse.urdf_fix import urdf_filler
from kineverse.utils    import res_pkg_path
from kineverse.model.paths                import find_common_root
from kineverse.model.geometry_model       import GeometryModel, \
                                                 Path as KPath, \
                                                 ArticulatedObject
from kineverse.operations.urdf_operations import load_urdf


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


if __name__ == '__main__':
  rospy.init_node(u'giskard')

  action_server_name = 'sandbox_server'

  god_map = GodMap()
  blackboard = Blackboard
  blackboard.god_map = god_map

  # god_map.safe_set_data(identifier.rosparam, rospy.get_param(rospy.get_name()))
  with open(res_pkg_path('package://iai_pr2_description/robots/pr2_calibrated_with_ft2.xml'), 'r') as urdf_file:
      god_map.safe_set_data(identifier.robot_description, hacky_urdf_parser_fix(urdf_file.read()))

  km = GeometryModel()
  god_map.safe_set_data(identifier.world, km)


  robot_urdf = urdf_filler(up.URDF.from_xml_string(god_map.get_data(identifier.robot_description)))

  # TODO: Not cool, prefix generation exploits knowledge about the structure of robot identifier
  load_urdf(km, 
            KPath(identifier.robot[len(identifier.world):]), 
            robot_urdf, 
            reference_frame='world', 
            joint_prefix=KPath(identifier.joint_states[len(identifier.world):]),
            robot_class=GiskardRobot)

  km.clean_structure()
  km.dispatch_events()

  robot = god_map.get_data(identifier.robot)
  joint_symbols = robot.get_joint_position_symbols() + robot.get_joint_velocity_symbols()
  joint_paths   = [KPath(j) for j in joint_symbols]
  js_prefix = find_common_root(joint_paths)

  # print('\n'.join(km.get_data(KPath(identifier.robot[len(identifier.world):])).links.keys()))

  god_map.safe_set_data(identifier.next_move_goal, create_example_goal())

  root = Sequence(u'root')
  


  root.add_child(GoalToConstraints(u'update constraints', action_server_name))
  root.add_child(ControllerPlugin(u'controller'))
  root.add_child(KinSimPlugin(u'kin sim'))

  tree = BehaviourTree(root)
  tree.tick()
