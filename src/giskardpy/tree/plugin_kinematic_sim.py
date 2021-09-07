from collections import OrderedDict

from py_trees import Status

import giskardpy.identifier as identifier
from giskardpy.data_types import JointStates
from giskardpy.tree.plugin import GiskardBehavior


class KinSimPlugin(GiskardBehavior):
    def __init__(self, name):
        super(KinSimPlugin, self).__init__(name)

    def initialise(self):
        self.sample_period = self.get_god_map().get_data(identifier.sample_period)
        super(KinSimPlugin, self).initialise()

    @profile
    def update(self):
        next_cmds = self.get_god_map().get_data(identifier.qp_solver_solution)
        # TODO this is ugly :)
        active_objects = set([self.get_god_map().expr_to_key[j][4] for j in next_cmds[0]])
        if next_cmds:
            for object_name in active_objects:
                if self.get_world().has_object(object_name):
                    obj = self.get_world().get_object(object_name)
                else:
                    obj = self.get_robot()
                current_js = obj.joint_state
                next_js = JointStates()
                for key, sjs in current_js.items():
                    if obj.is_joint_mimic(key):
                        continue
                    joint_name = str(obj.get_joint_position_symbol(key))
                    vel_cmds = next_cmds[0]
                    if joint_name in vel_cmds:
                        cmd = vel_cmds[joint_name]
                        derivative_cmds = [x[joint_name] for x in next_cmds]
                    else:
                        cmd = 0.0
                        derivative_cmds = []
                    next_js[key].position = sjs.position + cmd * self.sample_period
                    for i, derivative_cmd in enumerate(derivative_cmds):
                        next_js[key].set_derivative(i+1, derivative_cmd)
                obj.joint_state = next_js
        return Status.RUNNING
