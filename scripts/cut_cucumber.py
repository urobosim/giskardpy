import rospy
from giskard_msgs.msg import MoveCmd, JointConstraint
from giskard_msgs.msg import MoveAction
from giskard_msgs.msg import MoveGoal
from giskardpy.python_interface import GiskardWrapper
from geometry_msgs.msg import PoseStamped
from giskard_msgs.srv import UpdateWorld, UpdateWorldRequest
from giskard_msgs.msg import WorldBody
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Point, Quaternion

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


def execute_joint_goal():
    # Creates the SimpleActionClient, passing the type of the action
    # (MoveAction) to the constructor.
    giskard = GiskardWrapper()
    giskard.set_joint_goal(gaya_pose)
    giskard.plan_and_execute()

    p = PoseStamped()
    p.header.frame_id = u'cucumber'
    # p.pose.position.z = 0.1

    giskard.set_cart_goal(goal_pose=p,
                          tip_link=u'r_gripper_tool_frame',
                          root_link=giskard.get_root())
    giskard.plan_and_execute()

def add_cucumber():
    giskard = GiskardWrapper()

    cucumber = WorldBody()
    cucumber.type = WorldBody.PRIMITIVE_BODY
    cucumber.name = u'cucumber'

    cucumber_T_table = PoseStamped()
    cucumber_T_table.header.frame_id = u'iai_kitchen/dining_area_jokkmokk_table_main'
    cucumber_T_table.pose.position.x = 0
    cucumber_T_table.pose.position.y = 0
    cucumber_T_table.pose.position.z = 0.3
    cucumber_T_table.pose.orientation.x = 0
    cucumber_T_table.pose.orientation.y = 0
    cucumber_T_table.pose.orientation.z = 0
    cucumber_T_table.pose.orientation.w = 0

    res = giskard.add_cylinder(u'cucumber',
                               height=0.4,  # length 40 cm
                               radius=0.03,  # radius 3 cm -> diameter 6 cm
                               frame_id=u'iai_kitchen/dining_area_jokkmokk_table_main',
                               pose=cucumber_T_table
                               )
    print('Adding returns:')
    print(res)


def attach_cucumber():
    giskard = GiskardWrapper()
    res = giskard.attach_object(u'cucumber', u'l_gripper_tool_frame')
    print('Attaching returns:')
    print(res)


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('joint_space_client')
        giskard = GiskardWrapper()
        giskard.clear_world()
        add_cucumber()
        execute_joint_goal()
        attach_cucumber()
        giskard.clear_world()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
