from copy import deepcopy

from py_trees import Status

from giskardpy import identifier
from giskardpy.tree.plugin import GiskardBehavior


class LogTrajPlugin(GiskardBehavior):
    def update(self):
        trajectory = self.get_god_map().get_data(identifier.trajectory)
        for world_object in self.get_world().get_objects() + [self.get_robot()]:
            if world_object.get_joint_names():
                current_js = deepcopy(world_object.joint_state)
                time = self.get_god_map().get_data(identifier.time)
                trajectory[world_object.get_name()].set(time, current_js)
        self.get_god_map().set_data(identifier.trajectory, trajectory)
        return Status.RUNNING
