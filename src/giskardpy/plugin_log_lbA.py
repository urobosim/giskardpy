from collections import OrderedDict
import numpy as np
from py_trees import Status

from giskardpy import identifier
from giskardpy.data_types import Trajectory, SingleJointState
from giskardpy.plugin import GiskardBehavior


class LoglbAPlugin(GiskardBehavior):
    def __init__(self, name):
        super(LoglbAPlugin, self).__init__(name)
        self.number_of_joints = len(self.get_robot().controlled_joints)
        self.sample_period = self.get_god_map().get_data(identifier.sample_period)

    def initialise(self):
        self.trajectory = self.get_god_map().get_data(identifier.lbA_trajectory)

    def update(self):
        lbAs = np.array(self.get_god_map().get_data(identifier.lbA))
        names = np.array(self.get_god_map().get_data(identifier.bA_keys))
        filter_ = np.array(['debug' in x for x in names])
        lbAs = lbAs[filter_]
        names = names[filter_]
        if len(names) > 0:
            time = self.get_god_map().get_data(identifier.time)
            last_mjs = None
            if time == 1:
                mjs = OrderedDict()
                for name, lbA in zip(names, lbAs):
                    data_point = SingleJointState(name=name,
                                                  position=0,
                                                  velocity=0)
                    mjs[name] = data_point
                self.trajectory.set(0, mjs)
            if time > 1:
                last_mjs = self.trajectory.get_exact(time-1)
            mjs = OrderedDict()
            for name, lbA in zip(names, lbAs):
                if last_mjs is not None:
                    velocity = lbA - last_mjs[name].position
                else:
                    velocity = 0
                data_point = SingleJointState(name=name,
                                              position=lbA,
                                              velocity=velocity/self.sample_period)
                mjs[name] = data_point
            self.trajectory.set(time, mjs)
        return Status.RUNNING