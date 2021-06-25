import rospy
from giskard_msgs.msg import MoveCmd, JointConstraint
from giskard_msgs.msg import MoveAction
from giskard_msgs.msg import MoveGoal
from giskardpy.python_interface import GiskardWrapper
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3Stamped, PointStamped, Transform
from giskard_msgs.srv import UpdateWorld, UpdateWorldRequest
from giskard_msgs.msg import WorldBody
from shape_msgs.msg import SolidPrimitive
import casadi.casadi as w
from rospy_message_converter.message_converter import \
    convert_dictionary_to_ros_message, \
    convert_ros_message_to_dictionary
from tf.transformations import rotation_from_matrix, quaternion_matrix
from giskardpy.tfwrapper import transform_pose, lookup_pose
from giskardpy import logging, identifier
from giskardpy.garden import grow_tree
from giskardpy.identifier import robot, world
from giskardpy.pybullet_world import PyBulletWorld
from giskardpy.python_interface import GiskardWrapper
from giskardpy.robot import Robot
from giskardpy.tfwrapper import transform_pose, lookup_pose
from giskardpy.utils import msg_to_list, KeyDefaultDict, position_dict_to_joint_states, get_ros_pkg_path, \
    to_joint_state_position_dict

# Brings in the SimpleActionClient
import actionlib
from giskard_msgs.msg import MoveResult

gaya_pose_name = [u'r_shoulder_pan_joint',
                  u'r_shoulder_lift_joint',
                  u'r_upper_arm_roll_joint',
                  u'r_elbow_flex_joint',
                  u'r_forearm_roll_joint',
                  u'r_wrist_flex_joint',
                  u'r_wrist_roll_joint',
                  u'l_shoulder_pan_joint',
                  u'l_shoulder_lift_joint',
                  u'l_upper_arm_roll_joint',
                  u'l_elbow_flex_joint',
                  u'l_forearm_roll_joint',
                  u'l_wrist_flex_joint',
                  u'l_wrist_roll_joint',
                  u'torso_lift_joint',
                  #
                  u'head_pan_joint',
                  u'head_tilt_joint',
                  ]

gaya_pose_position = [-1.7125,
                      -0.25672,
                      -1.46335,
                      -2.12,
                      1.76632,
                      -0.10001,
                      0.05106,
                      1.9652,
                      - 0.26499,
                      1.3837,
                      -2.12,
                      16.99,
                      - 0.10001,
                      0,
                      0.2,
                      #
                      0,
                      0
                      ]

gaya_pose = {u'r_shoulder_pan_joint': -1.7125,
             u'r_shoulder_lift_joint': -0.25672,
             u'r_upper_arm_roll_joint': -1.46335,
             u'r_elbow_flex_joint': -2.12,
             u'r_forearm_roll_joint': 1.76632,
             u'r_wrist_flex_joint': -0.10001,
             u'r_wrist_roll_joint': 0.05106,
             u'l_shoulder_pan_joint': 1.9652,
             u'l_shoulder_lift_joint': - 0.26499,
             u'l_upper_arm_roll_joint': 1.3837,
             u'l_elbow_flex_joint': -2.12,
             u'l_forearm_roll_joint': 16.99,
             u'l_wrist_flex_joint': - 0.10001,
             u'l_wrist_roll_joint': 0,
             u'torso_lift_joint': 0.2,

             u'head_pan_joint': 0,
             u'head_tilt_joint': 0,
             }


class CutCucumber():
    primary_hand = u'r_gripper_tool_frame'
    support_hand = u'l_gripper_tool_frame'
    cucumber_frame = u'cucumber'
    cucumber_length = 0.4
    cucumber_radius = 0.03

    def __init__(self):
        self.giskard = GiskardWrapper()
        self.default_root = self.get_robot().get_root()

    def get_god_map(self):
        """
        :rtype: giskardpy.god_map.GodMap
        """
        return self.giskard.god_map

    def add_cucumber_on_table(self):
        cucumber = WorldBody()
        cucumber.type = WorldBody.PRIMITIVE_BODY
        cucumber.name = self.cucumber_frame

        cucumber_T_table = PoseStamped()
        cucumber_T_table.header.frame_id = u'iai_kitchen/dining_area_jokkmokk_table_main'
        cucumber_T_table.pose.position.x = 0
        cucumber_T_table.pose.position.y = -0.3
        cucumber_T_table.pose.position.z = 0.43
        cucumber_T_table.pose.orientation.x = 0
        cucumber_T_table.pose.orientation.y = 0.71
        cucumber_T_table.pose.orientation.z = 0
        cucumber_T_table.pose.orientation.w = 0.71

        res = self.giskard.add_cylinder(u'cucumber',
                                        height=self.cucumber_length,  # length 40 cm
                                        radius=self.cucumber_radius,  # radius 3 cm -> diameter 6 cm
                                        frame_id=u'iai_kitchen/dining_area_jokkmokk_table_main',
                                        pose=cucumber_T_table
                                        )
        print('Adding returns:')
        print(res)

    def get_robot(self):
        """
        :rtype: Robot
        """
        return self.get_god_map().get_data(robot)

    def move_base(self, goal_pose):
        goal_pose = transform_pose(self.default_root, goal_pose)
        js = {u'odom_x_joint': goal_pose.pose.position.x,
              u'odom_y_joint': goal_pose.pose.position.y,
              u'odom_z_joint': rotation_from_matrix(quaternion_matrix([goal_pose.pose.orientation.x,
                                                                       goal_pose.pose.orientation.y,
                                                                       goal_pose.pose.orientation.z,
                                                                       goal_pose.pose.orientation.w]))[0]}

    def grasp_cucumber(self):
        # Creates the SimpleActionClient, passing the type of the action
        # (MoveAction) to the constructor.
        self.giskard.set_joint_goal(gaya_pose)
        self.giskard.plan_and_execute()

        p = PoseStamped()
        p.header.frame_id = u'map'
        p.pose.position.x = -2
        p.pose.orientation = Quaternion(0, 0, -1, 0)
        self.move_base(p)

        p = PoseStamped()
        p.header.frame_id = self.cucumber_frame
        # p.pose.position.z = 0.1

        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = self.cucumber_frame
        bar_axis.vector.z = 1

        bar_center = PointStamped()
        bar_center.header.frame_id = self.cucumber_frame

        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = self.support_hand
        tip_grasp_axis.vector.z = 1

        self.giskard.grasp_bar(tip_link=self.support_hand,
                          root_link=self.giskard.get_root(),
                          tip_grasp_axis=tip_grasp_axis,
                          bar_axis=bar_axis,
                          bar_center=bar_center,
                          bar_length=self.cucumber_length)
        self.giskard.plan_and_execute()

        # res = self.giskard.attach_object(self.cucumber_frame, self.support_hand)
        # print('Attaching returns:')
        # print(res)


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('joint_space_client')

        cutting = CutCucumber()
        cutting.add_cucumber_on_table()
        cutting.grasp_cucumber()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
