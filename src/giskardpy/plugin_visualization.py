import py_trees
import rospy
from kineverse.visualization.bpb_visualizer import ROSBPBVisualizer
from plugin import GiskardBehavior

class VisualizationBehavior(GiskardBehavior):
    def __init__(self, name, ensure_publish=False):
        super(VisualizationBehavior, self).__init__(name)
        self.ensure_publish = ensure_publish

    def setup(self, timeout):
        self.robot_base = self.get_robot().get_root()
        self.visualizer = ROSBPBVisualizer(u'~visualization_marker_array', self.get_world().world_frame)
        self.namespace = u'visualization'
        return super(VisualizationBehavior, self).setup(timeout)

    def update(self):
        subworld = self.get_world().pb_subworld
        if subworld is not None:
            self.visualizer.begin_draw_cycle(self.namespace)
            self.visualizer.draw_world(self.namespace, subworld, a=0.9)
            self.visualizer.render()
        if self.ensure_publish:
            rospy.sleep(0.1)
            self.visualizer.begin_draw_cycle(self.namespace)
            self.visualizer.draw_world(self.namespace, subworld, a=0.9)
            self.visualizer.render()
        return py_trees.common.Status.SUCCESS

    def clear_marker(self):
        self.visualizer.begin_draw_cycle()
        self.visualizer.render()
