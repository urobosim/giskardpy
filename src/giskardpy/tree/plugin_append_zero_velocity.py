from collections import OrderedDict, defaultdict
from copy import deepcopy

from py_trees import Status

import giskardpy.identifier as identifier
from giskardpy.data_types import JointStates
from giskardpy.tree.plugin import GiskardBehavior


class AppendZeroVelocity(GiskardBehavior):
    def __init__(self, name):
        """
        :type js_identifier: str
        :type next_cmd_identifier: str
        :type time_identifier: str
        :param sample_period: the time difference in s between each step.
        :type sample_period: float
        """
        super(AppendZeroVelocity, self).__init__(name)

    def initialise(self):
        self.sample_period = self.get_god_map().get_data(identifier.sample_period)
        super(AppendZeroVelocity, self).initialise()

    def update(self):
        # FIXME we do we need this plugin?
        motor_commands = self.get_god_map().get_data(identifier.qp_solver_solution)
        current_js = self.get_god_map().get_data(identifier.joint_states)
        next_js = None
        if motor_commands:
            next_js = JointStates()
            for joint_name, sjs in current_js.items():
                next_js[joint_name].position = sjs.position
        if next_js is not None:
            self.get_god_map().set_data(identifier.joint_states, next_js)
        else:
            self.get_god_map().set_data(identifier.joint_states, current_js)
        return Status.SUCCESS
        #     # next_object_positions = defaultdict # name to position dict
        #     # next_object_velocities = {} # name to velocity dict
        #     # for joint_position_identifier, joint_velocity in motor_commands.items():
        #     #     object_name = joint_position_identifier[-4]
        #     #     joint_velocity_identifier = list(joint_position_identifier[:-1]) + [u'velocity']
        #     #     current_position = self.get_god_map().get_data(joint_position_identifier)
        #     #     current_position += joint_velocity
        #
        #     next_object_state = defaultdict(OrderedDict)  # name to position dict
        #     for joint_position_identifier, joint_velocity in motor_commands.items():
        #         object_name = joint_position_identifier[-4]
        #         joint_name = joint_position_identifier[-2]
        #         # joint_velocity_identifier = list(joint_position_identifier[:-1]) + [u'velocity']
        #
        #         current_position = self.get_god_map().get_data(joint_position_identifier)
        #         next_object_state[object_name][joint_name] = SingleJointState(joint_name,
        #                                                                       current_position + joint_velocity,
        #                                                                       velocity=joint_velocity/self.sample_period)
        #         # next_object_velocities[object_name][joint_name] = joint_velocity/self.sample_period
        #         # self.get_god_map().set_data(joint_position_identifier, current_position)
        #         # self.get_god_map().set_data(joint_velocity_identifier, joint_velocity/self.sample_period)
        #         # pass
        #     for object_name, next_state in next_object_state.items():
        #         self.get_world().get_object(object_name).joint_state = next_state
        #
        #     # next_js = OrderedDict()
        #     # for joint_name, sjs in current_js.items():
        #     #     if joint_name in motor_commands:
        #     #         cmd = motor_commands[joint_name]
        #     #     else:
        #     #         cmd = 0.0
        #     #     self.get_god_map().set_data()
        #     #     next_js[joint_name] = SingleJointState(sjs.name, sjs.position + cmd,
        #     #                                                 velocity=cmd/self.sample_period)
        # # if next_js is not None:
        # #     self.get_god_map().set_data(identifier.joint_states, next_js)
        # # else:
        # #     self.get_god_map().set_data(identifier.joint_states, current_js)
        # # self.get_god_map().set_data(identifier.last_joint_states, current_js)
        # return Status.RUNNING
