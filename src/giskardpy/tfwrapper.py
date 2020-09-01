import PyKDL
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Vector3Stamped, PointStamped, TransformStamped, Pose, Quaternion, Point, \
    Vector3, Twist, TwistStamped
from tf.transformations import quaternion_from_matrix
from tf2_geometry_msgs import do_transform_pose, do_transform_vector3, do_transform_point

from tf2_py._tf2 import ExtrapolationException
from tf2_ros import Buffer, TransformListener

from giskardpy import logging
from giskardpy.utils import suppress_stderr

tfBuffer = None  # type: Buffer
tf_listener = None


def init(tf_buffer_size=15):
    """
    If you want to specify the buffer size, call this function manually, otherwise don't worry about it.
    :param tf_buffer_size: in secs
    :type tf_buffer_size: int
    """
    global tfBuffer, tf_listener
    tfBuffer = Buffer(rospy.Duration(tf_buffer_size))
    tf_listener = TransformListener(tfBuffer)
    rospy.sleep(5.0)


def wait_for_transform(target_frame, source_frame, time, timeout):
    global tfBuller
    return tfBuffer.can_transform(target_frame, source_frame, time, timeout)


def transform_pose(target_frame, pose):
    """
    Transforms a pose stamped into a different target frame.
    :type target_frame: str
    :type pose: PoseStamped
    :return: Transformed pose of None on loop failure
    :rtype: PoseStamped
    """
    global tfBuffer
    if tfBuffer is None:
        init()
    with suppress_stderr():
        transform = tfBuffer.lookup_transform(target_frame,
                                              pose.header.frame_id,  # source frame
                                              pose.header.stamp,
                                              rospy.Duration(5.0))
    new_pose = do_transform_pose(pose, transform)
    return new_pose


def transform_vector(target_frame, vector):
    """
    Transforms a pose stamped into a different target frame.
    :type target_frame: str
    :type vector: Vector3Stamped
    :return: Transformed pose of None on loop failure
    :rtype: Vector3Stamped
    """
    global tfBuffer
    if tfBuffer is None:
        init()
    transform = tfBuffer.lookup_transform(target_frame,
                                          vector.header.frame_id,  # source frame
                                          vector.header.stamp,
                                          rospy.Duration(5.0))
    new_pose = do_transform_vector3(vector, transform)
    return new_pose


def transform_point(target_frame, point):
    """
    Transforms a pose stamped into a different target frame.
    :type target_frame: str
    :type point: PointStamped
    :return: Transformed pose of None on loop failure
    :rtype: PointStamped
    """
    global tfBuffer
    if tfBuffer is None:
        init()
    transform = tfBuffer.lookup_transform(target_frame,
                                          point.header.frame_id,  # source frame
                                          point.header.stamp,
                                          rospy.Duration(5.0))
    new_pose = do_transform_point(point, transform)
    return new_pose


def lookup_transform(target_frame, source_frame, time=None):
    """
    :type target_frame: str
    :type source_frame: str
    :return: Transform from target_frame to source_frame
    :rtype: TransformStamped
    """
    if not time:
        time = rospy.Time()
    global tfBuffer
    if tfBuffer is None:
        init()
    transform = tfBuffer.lookup_transform(target_frame, source_frame, time, rospy.Duration(5.0))
    return transform


def lookup_pose(target_frame, source_frame, time=None):
    """
    :type target_frame: str
    :type source_frame: str
    :return: target_frame <- source_frame
    :rtype: PoseStamped
    """
    p = PoseStamped()
    p.header.frame_id = source_frame
    if time is not None:
        p.header.stamp = time
    p.pose.orientation.w = 1.0
    return transform_pose(target_frame, p)


def lookup_point(target_frame, source_frame, time=None):
    """
    :type target_frame: str
    :type source_frame: str
    :return: target_frame <- source_frame
    :rtype: PointStamped
    """
    t = lookup_transform(target_frame, source_frame, time)
    p = PointStamped()
    p.header.frame_id = t.header.frame_id
    p.point.x = t.transform.translation.x
    p.point.y = t.transform.translation.y
    p.point.z = t.transform.translation.z
    return p


