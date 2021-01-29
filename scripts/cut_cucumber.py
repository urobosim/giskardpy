import rospy
from giskard_msgs.msg import MoveCmd, JointConstraint
from giskard_msgs.msg import MoveAction
from giskard_msgs.msg import MoveGoal

# Brings in the SimpleActionClient
import actionlib
from giskard_msgs.msg import MoveResult


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
    joint_goal.goal_state.name = ["l_upper_arm_roll_joint", "l_shoulder_pan_joint", "l_shoulder_lift_joint",
                                  "l_forearm_roll_joint", "l_elbow_flex_joint", "l_wrist_flex_joint",
                                  "l_wrist_roll_joint"]
    joint_goal.goal_state.position = [1, 0.5, 0.2, 0, -0.8, -1, 0]

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
