from py_trees import Status

from giskardpy import identifier
from giskardpy.logging import logwarn
from giskardpy.plugin import GiskardBehavior
from giskardpy.utils import plot_trajectory


class PlotTrajectory(GiskardBehavior):
    def __init__(self, name, order):
        super(PlotTrajectory, self).__init__(name)
        self.order = order
        self.velocity_threshold = 0.0
        self.scaling = 2.5
        self.normalize_position = False
        self.tick_stride = 0.5
        self.path_to_data_folder = self.get_god_map().get_data(identifier.data_folder)

    def update(self):
        trajectory = self.get_god_map().get_data(identifier.trajectory)
        if trajectory:
            sample_period = self.get_god_map().get_data(identifier.sample_period)
            controlled_joints = self.get_robot().controlled_joints
            try:
                plot_trajectory(trajectory, controlled_joints, self.path_to_data_folder, sample_period, self.order,
                                self.velocity_threshold, self.scaling, self.normalize_position, self.tick_stride)
            except Exception:
                logwarn(u'failed to save trajectory pdf')
        return Status.SUCCESS
