from copy import deepcopy

import numpy as np
import pytest
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from giskard_msgs.msg import MoveResult
from numpy import pi
from tf.transformations import quaternion_from_matrix, quaternion_about_axis
import giskardpy.utils.tfwrapper as tf
from giskardpy.utils import logging
from giskardpy.utils.tfwrapper import init as tf_init
from utils_for_tests import PR2, HSR

default_pose = {
    u'arm_flex_joint': 0.0,
    u'arm_lift_joint': 0.0,
    u'arm_roll_joint': 0.0,
    u'head_pan_joint': 0.0,
    u'head_tilt_joint': 0.0,
    u'odom_t': 0.0,
    u'odom_x': 0.0,
    u'odom_y': 0.0,
    u'wrist_flex_joint': 0.0,
    u'wrist_roll_joint': 0.0,
}

folder_name = u'tmp_data/'


@pytest.fixture(scope=u'module')
def ros(request):
    try:
        logging.loginfo(u'deleting tmp test folder')
        # shutil.rmtree(folder_name)
    except Exception:
        pass

    logging.loginfo(u'init ros')
    rospy.init_node(u'tests')
    tf_init(60)

    def kill_ros():
        logging.loginfo(u'shutdown ros')
        rospy.signal_shutdown(u'die')
        try:
            logging.loginfo(u'deleting tmp test folder')
            # shutil.rmtree(folder_name)
        except Exception:
            pass

    request.addfinalizer(kill_ros)


@pytest.fixture(scope=u'module')
def giskard(request, ros):
    c = HSR()
    request.addfinalizer(c.tear_down)
    return c


@pytest.fixture()
def resetted_giskard(giskard):
    """
    :type giskard: HSR
    """
    logging.loginfo(u'resetting giskard')
    giskard.clear_world()
    base_goal = PoseStamped()
    base_goal.header.frame_id = u'map'
    base_goal.pose.orientation.w = 1
    giskard.move_base(base_goal)
    giskard.close_gripper()
    return giskard


@pytest.fixture()
def zero_pose(resetted_giskard):
    """
    :type giskard: PR2
    """
    resetted_giskard.set_joint_goal(default_pose)
    resetted_giskard.allow_all_collisions()
    resetted_giskard.send_and_check_goal()
    return resetted_giskard


@pytest.fixture()
def box_setup(zero_pose):
    """
    :type pocky_pose_setup: PR2
    :rtype: PR2
    """
    p = PoseStamped()
    p.header.frame_id = u'map'
    p.pose.position.x = 1.2
    p.pose.position.y = 0
    p.pose.position.z = 0.1
    p.pose.orientation.w = 1
    zero_pose.add_box(size=[1, 1, 1], pose=p)
    return zero_pose

@pytest.fixture()
def kitchen_setup(zero_pose):
    """
    :type resetted_giskard: GiskardTestWrapper
    :return:
    """
    object_name = u'kitchen'
    zero_pose.add_urdf(object_name, rospy.get_param(u'kitchen_description'),
                              tf.lookup_pose(u'map', u'iai_kitchen/world'), u'/kitchen/joint_states')
    js = {k: 0.0 for k in zero_pose.get_world().get_object(object_name).get_controllable_joints()}
    zero_pose.set_kitchen_js(js)
    return zero_pose

class TestJointGoals(object):
    def test_move_base(self, zero_pose):
        p = PoseStamped()
        p.header.frame_id = u'map'
        p.pose.position.y = -1
        p.pose.orientation = Quaternion(0, 0, 0.47942554, 0.87758256)
        zero_pose.move_base(p)


class TestCartGoals(object):

    def test_rotate_gripper(self, zero_pose):
        """
        :type zero_pose: HSR
        """
        r_goal = PoseStamped()
        r_goal.header.frame_id = zero_pose.tip
        r_goal.pose.orientation = Quaternion(*quaternion_about_axis(pi, [0, 0, 1]))
        zero_pose.set_and_check_cart_goal(r_goal, zero_pose.tip)


class TestCollisionAvoidanceGoals(object):

    def test_self_collision_avoidance(self, zero_pose):
        """
        :type zero_pose: HSR
        """
        r_goal = PoseStamped()
        r_goal.header.frame_id = zero_pose.tip
        r_goal.pose.position.z = 0.5
        r_goal.pose.orientation.w = 1
        zero_pose.set_and_check_cart_goal(r_goal, zero_pose.tip)

    def test_self_collision_avoidance2(self, zero_pose):
        """
        :type zero_pose: HSR
        """
        js = {
            u'arm_flex_joint': 0.0,
            u'arm_lift_joint': 0.0,
            u'arm_roll_joint': -1.52,
            u'head_pan_joint': -0.09,
            u'head_tilt_joint': -0.62,
            u'wrist_flex_joint': -1.55,
            u'wrist_roll_joint': 0.11,
        }
        zero_pose.allow_self_collision()
        zero_pose.send_and_check_joint_goal(js)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = u'hand_palm_link'
        goal_pose.pose.position.x = 0.5
        goal_pose.pose.orientation.w = 1
        zero_pose.set_and_check_cart_goal(goal_pose, zero_pose.tip)

    def test_attached_collision1(self, box_setup):
        """
        :type box_setup: HSR
        """
        box_name = u'asdf'
        box_pose = PoseStamped()
        box_pose.header.frame_id = u'map'
        box_pose.pose.position = Point(0.85, 0.3, .66)
        box_pose.pose.orientation = Quaternion(0, 0, 0, 1)

        box_setup.add_box(box_name, [0.07, 0.04, 0.1], box_pose)
        box_setup.open_gripper()

        grasp_pose = deepcopy(box_pose)
        grasp_pose.pose.position.x -= 0.05
        grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix([[0, 0, 1, 0],
                                                                          [0, -1, 0, 0],
                                                                          [1, 0, 0, 0],
                                                                          [0, 0, 0, 1]]))
        box_setup.set_and_check_cart_goal(grasp_pose, box_setup.tip)
        box_setup.attach_object(box_name, box_setup.tip)

        base_goal = PoseStamped()
        base_goal.header.frame_id = box_setup.default_root
        base_goal.pose.position.x -= 1
        base_goal.pose.orientation.w = 1
        box_setup.move_base(base_goal)

    def test_collision_avoidance(self, zero_pose):
        """
        :type box_setup: HSR
        """
        js = {u'arm_flex_joint': -np.pi / 2}
        zero_pose.send_and_check_joint_goal(js)

        p = PoseStamped()
        p.header.frame_id = u'map'
        p.pose.position.x = 0.9
        p.pose.position.y = 0
        p.pose.position.z = 0.5
        p.pose.orientation.w = 1
        zero_pose.add_box(size=[1, 1, 0.01], pose=p)

        js = {u'arm_flex_joint': 0}
        zero_pose.send_and_check_joint_goal(js, expected_error_codes=[MoveResult.SHAKING])

class TestConstraints(object):
    def test_open_fridge(self, kitchen_setup):
        """
        :type kitchen_setup: HSR
        """
        handle_frame_id = u'iai_kitchen/iai_fridge_door_handle'
        handle_name = u'iai_fridge_door_handle'
        kitchen_setup.open_gripper()

        base_goal = PoseStamped()
        base_goal.header.frame_id = u'map'
        base_goal.pose.position = Point(0.3, -0.5, 0)
        base_goal.pose.orientation.w = 1
        kitchen_setup.move_base(base_goal)

        bar_axis = Vector3Stamped()
        bar_axis.header.frame_id = handle_frame_id
        bar_axis.vector.z = 1

        bar_center = PointStamped()
        bar_center.header.frame_id = handle_frame_id

        tip_grasp_axis = Vector3Stamped()
        tip_grasp_axis.header.frame_id = kitchen_setup.tip
        tip_grasp_axis.vector.x = 1

        kitchen_setup.add_json_goal(u'GraspBar',
                                    root=kitchen_setup.default_root,
                                    tip=kitchen_setup.tip,
                                    tip_grasp_axis=tip_grasp_axis,
                                    bar_center=bar_center,
                                    bar_axis=bar_axis,
                                    bar_length=.65)
        x_gripper = Vector3Stamped()
        x_gripper.header.frame_id = kitchen_setup.tip
        x_gripper.vector.z = 1

        x_goal = Vector3Stamped()
        x_goal.header.frame_id = handle_frame_id
        x_goal.vector.x = -1
        kitchen_setup.align_planes(kitchen_setup.tip, x_gripper, root_normal=x_goal)
        # kitchen_setup.allow_all_collisions()
        # kitchen_setup.add_json_goal(u'AvoidJointLimits', percentage=10)
        kitchen_setup.send_and_check_goal()

        kitchen_setup.add_json_goal(u'Open1Dof',
                                    tip=kitchen_setup.tip,
                                    object_name=u'kitchen',
                                    handle_link=handle_name,
                                    goal_joint_state=1)
        # kitchen_setup.allow_all_collisions()
        kitchen_setup.allow_self_collision()
        # kitchen_setup.add_json_goal(u'AvoidJointLimits')
        kitchen_setup.send_and_check_goal()
        kitchen_setup.set_kitchen_js({u'iai_fridge_door_joint': 1.5})

        kitchen_setup.add_json_goal(u'Open1Dof',
                                    tip=kitchen_setup.tip,
                                    object_name=u'kitchen',
                                    handle_link=handle_name,
                                    goal_joint_state=0)
        # kitchen_setup.allow_all_collisions()
        kitchen_setup.send_and_check_goal()
        kitchen_setup.set_kitchen_js({u'iai_fridge_door_joint': 0})



