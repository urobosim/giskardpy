import rospy
from giskard_msgs.msg import MoveCmd, JointConstraint
from giskard_msgs.msg import MoveAction
from giskard_msgs.msg import MoveGoal

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


def execute_joint_goal():
    # Creates the SimpleActionClient, passing the type of the action
    # (MoveAction) to the constructor.
    client = actionlib.SimpleActionClient("/giskard/command", MoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print('waiting for giskard')
    client.wait_for_server()
    print('connected to giskard')

    # Creates a goal to send to the action server.
    action_goal = MoveGoal()
    action_goal.type = MoveGoal.PLAN_AND_EXECUTE

    goal = MoveCmd()

    joint_goal = JointConstraint()

    joint_goal.type = JointConstraint.JOINT
    # this can be any subset of the robots joints
    # joint_goal.goal_state is a normal sensor_msgs/JointState
    joint_goal.goal_state.name = gaya_pose_name
    joint_goal.goal_state.position = gaya_pose_position

    goal.joint_constraints.append(joint_goal)
    action_goal.cmd_seq.append(goal)

    # Sends the goal to the action server.
    client.send_goal(action_goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    result = client.get_result()  # type: MoveResult
    if result.error_codes[0] == MoveResult.SUCCESS:
        print('giskard returned success')
    else:
        print('something went wrong')


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('joint_space_client')
        execute_joint_goal()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
