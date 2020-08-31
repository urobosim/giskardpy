from collections import OrderedDict
from copy import deepcopy

from py_trees import Status

from giskardpy.data_types import SingleJointState
import giskardpy.identifier as identifier
from giskardpy.plugin import GiskardBehavior


class KinSimPlugin(GiskardBehavior):
    def __init__(self, name):
        """
        :type js_identifier: str
        :type next_cmd_identifier: str
        :type time_identifier: str
        :param sample_period: the time difference in s between each step.
        :type sample_period: float
        """
        super(KinSimPlugin, self).__init__(name)

    def initialise(self):
        self.sample_period = self.get_god_map().get_data(identifier.sample_period)
        super(KinSimPlugin, self).initialise()

    def update(self):
        motor_commands = self.get_god_map().get_data(identifier.cmd)
        # current_js = self.get_god_map().get_data(identifier.joint_states)
        # next_js = None
        if motor_commands:
            for joint_position_identifier, joint_velocity in motor_commands.items():
                joint_velocity_identifier = list(joint_position_identifier[:-1]) + [u'velocity']
                current_position = self.get_god_map().get_data(joint_position_identifier)
                current_position += joint_velocity
                self.get_god_map().set_data(joint_position_identifier, current_position)
                self.get_god_map().set_data(joint_velocity_identifier, joint_velocity/self.sample_period)
                pass
            # next_js = OrderedDict()
            # for joint_name, sjs in current_js.items():
            #     if joint_name in motor_commands:
            #         cmd = motor_commands[joint_name]
            #     else:
            #         cmd = 0.0
            #     self.get_god_map().set_data()
            #     next_js[joint_name] = SingleJointState(sjs.name, sjs.position + cmd,
            #                                                 velocity=cmd/self.sample_period)
        # if next_js is not None:
        #     self.get_god_map().safe_set_data(identifier.joint_states, next_js)
        # else:
        #     self.get_god_map().safe_set_data(identifier.joint_states, current_js)
        return Status.RUNNING
