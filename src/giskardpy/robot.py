import traceback
from collections import namedtuple, OrderedDict, defaultdict
from copy import deepcopy
from itertools import combinations

from geometry_msgs.msg import PoseStamped

from giskardpy import WORLD_IMPLEMENTATION, cas_wrapper as w
from giskardpy.data_types import SingleJointState, HardConstraint, JointConstraint
from giskardpy.pybullet_world_object import PyBulletWorldObject
from giskardpy.utils import KeyDefaultDict, \
    homo_matrix_to_pose, memoize
from giskardpy.world_object import WorldObject

if WORLD_IMPLEMENTATION == u'pybullet':
    Backend = PyBulletWorldObject
else:
    Backend = WorldObject

class Robot(Backend):
    # def init2(self, base_pose=None, controlled_joints=None, path_to_data_folder=u'', *args, **kwargs):
    #     """
    #     :param urdf:
    #     :type urdf: str
    #     :param joints_to_symbols_map: maps urdfs joint names to symbols
    #     :type joints_to_symbols_map: dict
    #     :param joint_vel_limit: all velocity limits which are undefined or higher than this will be set to this
    #     :type joint_vel_limit: Symbol
    #     """
        # self._fk_expressions = {}
        # self._fks = {}
        # self._evaluated_fks = {}
        # self._joint_to_frame = {}
        # self._joint_position_symbols = KeyDefaultDict(lambda x: w.Symbol(x))  # don't iterate over this map!!
        # self._joint_velocity_symbols = KeyDefaultDict(lambda x: 0)  # don't iterate over this map!!
        # self._joint_velocity_linear_limit = KeyDefaultDict(lambda x: 10000) # don't overwrite urdf limits by default
        # self._joint_velocity_angular_limit = KeyDefaultDict(lambda x: 100000)
        # self._joint_acc_linear_limit = defaultdict(lambda: 100)  # no acceleration limit per default
        # self._joint_acc_angular_limit = defaultdict(lambda: 100)  # no acceleration limit per default
        # self._joint_weights = defaultdict(lambda: 0)
        # super(Robot, self).init2(base_pose, controlled_joints, path_to_data_folder, *args, **kwargs)
        # self.reinitialize()

    # @property
    # def hard_constraints(self):
    #     return self._hard_constraints
    #
    # @property
    # def joint_constraints(self):
    #     return self._joint_constraints

    @Backend.joint_state.setter
    def joint_state(self, value):
        """
        :param joint_state:
        :type joint_state: dict
        :return:
        """
        Backend.joint_state.fset(self, value)
        self.__joint_state_positions = {k: v.position for k, v in self.joint_state.items()}
        # self._evaluated_fks.clear()
        # self.get_fk_np.memo.clear()

    def set_limit_map(self, limit_map):
        self.limit_map = limit_map

    @memoize
    def get_controlled_parent_joint(self, link_name):
        joint = self.get_parent_joint_of_link(link_name)
        while joint not in self.controlled_joints:
            joint = self.get_parent_joint_of_joint(joint)
        return joint

    def get_joint_state_positions(self):
        try:
            return self.__joint_state_positions
        except:
            return {x: 0 for x in self.get_controllable_joints()}

    # def reinitialize(self):
    #     """
    #     :param joint_position_symbols: maps urdfs joint names to symbols
    #     :type joint_position_symbols: dict
    #     """
    #     super(Robot, self).reinitialize()
    #     self._fk_expressions = {}
    #     self._create_frames_expressions()
    #     self._create_constraints()
    #     self.init_fast_fks()

    # def set_joint_position_symbols(self, symbols):
    #     self._joint_position_symbols.update(symbols)
    #
    # def set_joint_velocity_limit_symbols(self, linear, angular):
    #     self._joint_velocity_linear_limit = linear
    #     self._joint_velocity_angular_limit = angular
    #
    # def set_joint_velocity_symbols(self, symbols):
    #     self._joint_velocity_symbols = symbols
    #
    # def set_joint_acceleration_limit_symbols(self, linear, angular):
    #     self._joint_acc_linear_limit = linear
    #     self._joint_acc_angular_limit = angular
    #
    # def set_joint_weight_symbols(self, symbols):
    #     self._joint_weights = symbols
    #
    # def update_joint_symbols(self, position, velocity, weights,
    #                          linear_velocity_limit, angular_velocity_limit,
    #                          linear_acceleration_limit, angular_acceleration_limit):
    #     # self.set_joint_position_symbols(position)
    #     # self.set_joint_velocity_symbols(velocity)
    #     self.set_joint_weight_symbols(weights)
    #     self.set_joint_velocity_limit_symbols(linear_velocity_limit, angular_velocity_limit)
    #     self.set_joint_acceleration_limit_symbols(linear_acceleration_limit, angular_acceleration_limit)
        # self.reinitialize()


    def get_to_parent_frame(self, link_name):
        return self.links[link_name].to_parent


    # JOINT FUNCTIONS

    # @memoize
    # def get_joint_symbols(self):
    #     """
    #     :return: dict mapping urdfs joint name to symbol
    #     :rtype: dict
    #     """
    #     return {joint_name: self.get_joint_position_symbol(joint_name) for joint_name in
    #             self.get_joint_names_controllable()}

    # def get_joint_velocity_limit_expr(self, joint_name):
    #     """
    #     :param joint_name: name of the joint in the urdfs
    #     :type joint_name: str
    #     :return: minimum of default velocity limit and limit specified in urdfs
    #     :rtype: float
    #     """
    #     limit = self.get_joint_velocity_limit(joint_name)
    #     if self.is_joint_prismatic(joint_name):
    #         limit_symbol = self._joint_velocity_linear_limit[joint_name]
    #     else:
    #         limit_symbol = self._joint_velocity_angular_limit[joint_name]
    #     if limit is None:
    #         return limit_symbol
    #     else:
    #         return w.Min(limit, limit_symbol)

    def get_joint_frame(self, joint_name):
        """
        :param joint_name: name of the joint in the urdfs
        :type joint_name: str
        :return: matrix expression describing the transformation caused by this joint
        :rtype: spw.Matrix
        """
        return self.joints[joint_name]

    # def get_joint_position_symbol(self, joint_name):
    #     """
    #     :param joint_name: name of the joint in the urdfs
    #     :type joint_name: str
    #     :rtype: spw.Symbol
    #     """
    #     return self._joint_position_symbols[joint_name]
    #
    # def get_joint_position_symbols(self):
    #     return [self.get_joint_position_symbol(joint_name) for joint_name in self.controlled_joints]

    # def get_joint_velocity_symbol(self, joint_name):
    #     """
    #     :param joint_name: name of the joint in the urdfs
    #     :type joint_name: str
    #     :rtype: spw.Symbol
    #     """
    #     return self._joint_velocity_symbols[joint_name]

    # def get_joint_velocity_symbols(self):
    #     return [self.get_joint_velocity_symbol(joint_name) for joint_name in self.controlled_joints]

    def generate_joint_state(self, f):
        """
        :param f: lambda joint_info: float
        :return:
        """
        js = {}
        for joint_name in self.controlled_joints:
            sjs = SingleJointState()
            sjs.name = joint_name
            sjs.position = f(joint_name)
            js[joint_name] = sjs
        return js

    def link_order(self, link_a, link_b):
        """
        TODO find a better name
        this function is used when deciding for which order to calculate the collisions
        true if link_a < link_b
        :type link_a: str
        :type link_b: str
        :rtype: bool
        """
        try:
            self.get_controlled_parent_joint(link_a)
        except KeyError:
            return False
        try:
            self.get_controlled_parent_joint(link_b)
        except KeyError:
            return True
        return link_a < link_b

    @memoize
    def get_chain_reduced_to_controlled_joints(self, link_a, link_b):
        chain = self.get_chain(link_b, link_a)
        for i, thing in enumerate(chain):
            if i % 2 == 1 and thing in self.controlled_joints:
                new_link_b = chain[i - 1]
                break
        else:
            raise KeyError(u'no controlled joint in chain between {} and {}'.format(link_a, link_b))
        for i, thing in enumerate(reversed(chain)):
            if i % 2 == 1 and thing in self.controlled_joints:
                new_link_a = chain[len(chain) - i]
                break
        else:
            raise KeyError(u'no controlled joint in chain between {} and {}'.format(link_a, link_b))
        return new_link_a, new_link_b


    def reset_cache(self, *args, **kwargs):
        for method_name in dir(self):
            try:
                getattr(self, method_name).memo.clear()
            except:
                pass

