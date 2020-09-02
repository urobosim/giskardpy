import py_trees
from kineverse.visualization.bpb_visualizer import ROSBPBVisualizer
from plugin import GiskardBehavior

# TODO ensure one last update after planning

class VisualizationBehavior(GiskardBehavior):
    def setup(self, timeout):
        self.robot_base = self.get_robot().get_root()
        self.visualizer = ROSBPBVisualizer(u'~visualization_marker_array', self.get_world().world_frame)
        self.namespace = u'visualization'
        return super(VisualizationBehavior, self).setup(timeout)

    def update(self):
        subworld = self.get_world().pb_subworld
        if subworld is not None:
            self.visualizer.begin_draw_cycle(self.namespace)
            self.visualizer.draw_world(self.namespace, subworld)
            self.visualizer.render()
        return py_trees.common.Status.SUCCESS
