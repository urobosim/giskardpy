import hashlib
from giskardpy import cas_wrapper as w
import py_trees
import rospy
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
from visualization_msgs.msg import Marker, MarkerArray

from giskardpy.tfwrapper import pose_to_kdl, kdl_to_pose
from kineverse.gradients.common_math import to_numpy
from plugin import GiskardBehavior

# TODO ensure one last update after planning

class VisualizationBehavior(GiskardBehavior):
    def setup(self, timeout):
        self.publisher = rospy.Publisher(u'~visualization_marker_array', MarkerArray, queue_size=1)
        self.robot_base = self.get_robot().get_root()
        return super(VisualizationBehavior, self).setup(timeout)

    def update(self):
        markers = []
        # world = self.get_world()
        robot = self.get_robot()
        get_fk = robot.get_fk_pose
        links = [x for x in self.get_robot().get_link_names() if robot.has_link_visuals(x)]
        for i, link_name in enumerate(links):
            marker = robot.link_as_marker(link_name)
            if marker is None:
                continue

            marker.header.frame_id = self.robot_base
            marker.action = Marker.ADD
            marker.id = int(hashlib.md5(link_name).hexdigest()[:6],
                            16)  # FIXME find a better way to give the same link the same id
            marker.ns = u'planning_visualization'
            marker.header.stamp = rospy.Time()

            origin = to_numpy(robot.get_link(link_name).geometry.values()[0].to_parent)
            fk = get_fk(self.robot_base, link_name).pose

            if origin is not None:
                position = w.position_of(origin)
                orientation = quaternion_from_matrix(origin)
                marker.pose.position.x = position[0]
                marker.pose.position.y = position[1]
                marker.pose.position.z = position[2]
                marker.pose.orientation = Quaternion(*orientation)
                marker.pose = kdl_to_pose(pose_to_kdl(fk) * pose_to_kdl(marker.pose))
            else:
                marker.pose = fk
            markers.append(marker)

        self.publisher.publish(markers)
        return py_trees.common.Status.SUCCESS
