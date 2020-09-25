import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from py_trees import Status
from tf2_msgs.msg import TFMessage

from giskardpy import logging
from giskardpy.plugin import GiskardBehavior
from giskardpy.utils import normalize_quaternion_msg
from kineverse.model.paths import Path


class TFPlugin(GiskardBehavior):
    """
    TODO
    """

    def __init__(self, name):
        super(TFPlugin, self).__init__(name)
        self.original_links = set(self.get_robot().get_link_names())
        self.tf_pub = rospy.Publisher(u'/tf', TFMessage, queue_size=10)

    def update(self):
        try:
            with self.get_god_map():
                if self.get_world().attached_objects:
                    tf_msg = TFMessage()
                    for object_name in self.get_robot().attached_objects:
                        object_link = self.get_world().get_object(object_name).get_root()
                        robot_link = self.get_robot().get_parent_link_of_link(object_link)
                        fk = self.get_robot().get_fk_pose(robot_link, object_link)
                        tf = TransformStamped()
                        tf.header = fk.header
                        tf.header.stamp = rospy.get_rostime()
                        tf.child_frame_id = object_link
                        tf.transform.translation.x = fk.pose.position.x
                        tf.transform.translation.y = fk.pose.position.y
                        tf.transform.translation.z = fk.pose.position.z
                        tf.transform.rotation = normalize_quaternion_msg(fk.pose.orientation)
                        tf_msg.transforms.append(tf)
                    self.tf_pub.publish(tf_msg)
        except KeyError as e:
            pass
        except UnboundLocalError as e:
            pass
        except ValueError as e:
            pass
        return Status.SUCCESS
