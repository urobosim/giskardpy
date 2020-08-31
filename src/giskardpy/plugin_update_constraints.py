import inspect
import itertools
import json
import traceback
from collections import OrderedDict
from collections import defaultdict
from time import time

from giskard_msgs.msg import MoveCmd
from py_trees import Status

from kineverse.model.paths import Path
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary

import giskardpy.constraints
import giskardpy.identifier as identifier
import kineverse.gradients.common_math as cm
from giskardpy.constraints import SelfCollisionAvoidance, ExternalCollisionAvoidance
from giskardpy.data_types import JointConstraint, HardConstraint, SoftConstraint
from giskardpy.exceptions import InsolvableException, ImplementationException
from giskardpy.logging import loginfo
from giskardpy.plugin_action_server import GetGoal
from kineverse.gradients.diff_logic import DiffSymbol, erase_type
# import kineverse.gradients.gradient_math as gm
from giskardpy import cas_wrapper as w


def allowed_constraint_names():
    return [x[0] for x in inspect.getmembers(giskardpy.constraints) if inspect.isclass(x[1])]


# def generate_controlled_values(constraints,
#                                symbols,
#                                weights={},
#                                bounds={},
#                                default_weight=0.001, default_bounds=(-1e9, 1e9)):
# FIXME not default weight
# FIXME no default bounds
# joint_constraints = OrderedDict()
# to_remove = set()
#
# for k, c in constraints.items():
#     if cm.is_symbol(c.expr) and c.expr in symbols and str(c.expr) not in joint_constraints:
#         weight = default_weight if c.expr not in weights else weights[c.expr]
#         joint_constraints[str(c.expr)] = JointConstraint(c.lower, c.upper, weight)
#         to_remove.add(k)
#
# hard_constraints = OrderedDict(
#     [(k, HardConstraint(c.lower, c.upper, c.expr)) for k, c in constraints.items() if k not in to_remove])
# for s in symbols:
#     if str(s) not in joint_constraints:
#         lower, upper = default_bounds if s not in bounds else bounds[s]
#         weight = default_weight if s not in weights else weights[s]
#         joint_constraints[str(s)] = JointConstraint(lower, upper, weight)
#
# return joint_constraints, hard_constraints


class GoalToConstraints(GetGoal):
    # FIXME no error msg when constraint has missing parameter
    def __init__(self, name, as_name):
        GetGoal.__init__(self, name, as_name)
        self.used_joints = set()

        self.controlled_joints = set()
        self.controllable_links = set()
        self.last_urdf = None

        self.rc_prismatic_velocity = self.get_god_map().get_data(identifier.rc_prismatic_velocity)
        self.rc_continuous_velocity = self.get_god_map().get_data(identifier.rc_continuous_velocity)
        self.rc_revolute_velocity = self.get_god_map().get_data(identifier.rc_revolute_velocity)
        self.rc_other_velocity = self.get_god_map().get_data(identifier.rc_other_velocity)

    def initialise(self):
        self.get_god_map().safe_set_data(identifier.collision_goal_identifier, None)

    def update(self):
        # TODO make this interruptable
        # TODO try catch everything

        move_cmd = self.get_god_map().get_data(identifier.next_move_goal)  # type: MoveCmd
        if not move_cmd:
            return Status.FAILURE

        self.get_god_map().safe_set_data(identifier.constraints_identifier, {})

        self.soft_constraints = {}
        # TODO we only have to update the collision constraints, if the robot changed
        self.add_collision_avoidance_soft_constraints()

        try:
            self.parse_constraints(move_cmd)
        except AttributeError:
            self.raise_to_blackboard(InsolvableException(u'couldn\'t transform goal'))
            traceback.print_exc()
            return Status.SUCCESS
        except InsolvableException as e:
            self.raise_to_blackboard(e)
            traceback.print_exc()
            return Status.SUCCESS
        except Exception as e:
            self.raise_to_blackboard(e)
            traceback.print_exc()
            return Status.SUCCESS

        # self.set_unused_joint_goals_to_current()

        self.get_god_map().safe_set_data(identifier.collision_goal_identifier, move_cmd.collisions)
        self.get_god_map().safe_set_data(identifier.soft_constraint_identifier, self.soft_constraints)
        self.get_blackboard().runtime = time()

        # relevant_symbols = set(sum([list(cm.free_symbols(c.expression)) for c in self.soft_constraints.values()], []))

        # This is not a principled way of determining relevant symbols
        # There is the inherent assumption that the constraints are only defined on positions
        # and also that the robot's positions are always controllable
        # relevant_joints   = controlled_joints.intersection(relevant_symbols)

        # if self.get_god_map().get_data(identifier.check_reachability):
        #     #FIXME support reachability check again
        #     from giskardpy import cas_wrapper as w
        #     joint_constraints = OrderedDict()
        #     # for k in controlled_joints:
        #     #     weight = self.robot._joint_constraints[k].weight
        #     #     if self.get_robot().is_joint_prismatic(k):
        #     #         joint_constraints[(self.robot.get_name(), k)] = JointConstraint(-self.rc_prismatic_velocity,
        #     #                                                                         self.rc_prismatic_velocity, weight)
        #     #     elif self.get_robot().is_joint_continuous(k):
        #     #         joint_constraints[(self.robot.get_name(), k)] = JointConstraint(-self.rc_continuous_velocity,
        #     #                                                                         self.rc_continuous_velocity, weight)
        #     #     elif self.get_robot().is_joint_revolute(k):
        #     #         joint_constraints[(self.robot.get_name(), k)] = JointConstraint(-self.rc_revolute_velocity,
        #     #                                                                         self.rc_revolute_velocity, weight)
        #     #     else:
        #     #         joint_constraints[(self.robot.get_name(), k)] = JointConstraint(-self.rc_other_velocity,
        #     #                                                                         self.rc_other_velocity, weight)
        #     for i, joint_name in enumerate(controlled_joints):
        #         lower_limit, upper_limit = self.get_robot().get_joint_limits(joint_name)
        #         joint_symbol = self.get_robot().get_joint_position_symbol(joint_name)
        #         sample_period = w.Symbol(u'rosparam_general_options_sample_period')  # TODO this should be a parameter
        #         # velocity_limit = self.get_robot().get_joint_velocity_limit_expr(joint_name) * sample_period
        #         if self.get_robot().is_joint_prismatic(joint_name):
        #             velocity_limit = self.rc_prismatic_velocity * sample_period
        #         elif self.get_robot().is_joint_continuous(joint_name):
        #             velocity_limit = self.rc_continuous_velocity * sample_period
        #         elif self.get_robot().is_joint_revolute(joint_name):
        #             velocity_limit = self.rc_revolute_velocity * sample_period
        #         else:
        #             velocity_limit = self.rc_other_velocity * sample_period
        #
        #         weight = self.get_robot()._joint_weights[joint_name]
        #         weight = weight * (1. / (self.rc_prismatic_velocity)) ** 2
        #
        #         if not self.get_robot().is_joint_continuous(joint_name):
        #             joint_constraints[(self.get_robot().get_name(), joint_name)] = JointConstraint(
        #                 lower=w.Max(-velocity_limit, lower_limit - joint_symbol),
        #                 upper=w.Min(velocity_limit, upper_limit - joint_symbol),
        #                 weight=weight)
        #         else:
        #             joint_constraints[(self.get_robot().get_name(), joint_name)] = JointConstraint(
        #                 lower=-velocity_limit,
        #                 upper=velocity_limit,
        #                 weight=weight)
        # else:
        # joint_constraints = OrderedDict(((self.robot.get_name(), k), self.robot._joint_constraints[k]) for k in controlled_joints)
        # km = self.get_god_map().get_data(identifier.km_world)
        # joint_control_space = {DiffSymbol(s) for s in controlled_joints}
        # constraints = km.get_constraints_by_symbols(set(controlled_joints).union(joint_control_space))
        # joint_constraints, hard_constraints = generate_controlled_values(constraints,
        #                                                                  joint_control_space)
        # self.get_god_map().get_data(identifier.joint_cost))
        # pass

        # hard_constraints = OrderedDict(((self.robot.get_name(), k), self.robot._hard_constraints[k]) for k in
        #                                controlled_joints if k in self.robot._hard_constraints)

        # hard_constraints = OrderedDict([((self.robot.get_name(), k), c) for k, c in hard_constraints.items()])

        # self.get_god_map().safe_set_data(identifier.joint_constraint_identifier, joint_constraints)
        # self.get_god_map().safe_set_data(identifier.hard_constraint_identifier, hard_constraints)
        self.add_robot_constraints()

        return Status.SUCCESS

    def add_robot_constraints(self):
        # joint_position_symbols = set(self.get_robot().get_joint_position_symbols())
        world = self.get_god_map().get_data(identifier.world)
        # FIXME the references are fucked
        # FIXME you also still have to check the soft constraints for e.g. kitchen joints
        # joint_position_symbols = set(
        #     sum([list(cm.free_symbols(c.expression)) for c in self.soft_constraints.values()], []))
        joint_position_symbols = self.get_robot().get_controlled_joint_position_symbols()
        # all_joint_position_symbols = [self.get_god_map().to_symbol(identifier.joint_states + [joint_name, u'position']) for joint_name in self.get_robot().get_controllable_joints()]
        # joint_position_symbols = set(self.get_robot().get_joint_position_symbols())
        # for joint_name in self.get_robot().controlled_joints:
        #     s = self.get_god_map().to_symbol(identifier.joint_states + [joint_name, 'position'])
        #     s = self.get_god_map().get_kineverse_symbol(s)
        #     joint_position_symbols.add(s)

        joint_velocity_symbols = {DiffSymbol(s) for s in joint_position_symbols}

        constraints = world.km_model.get_constraints_by_symbols(joint_velocity_symbols.union(joint_position_symbols))
        # joint_position_constraints = world.km_model.get_constraints_by_symbols(joint_position_symbols)

        # FIXME this is a hack to get rid of the path
        # joint_velocity_constraints = {str(erase_type(Path(k)[-1])): c for k, c in joint_velocity_constraints.items()}
        # joint_position_constraints = {str(erase_type(Path(k)[-1])): c for k, c in joint_position_constraints.items()}

        joint_constraints = OrderedDict()
        to_remove = set()
        symbols = set()
        # symbols.extend(all_joint_position_symbols)

        for k, c in sorted(constraints.items()):
            if cm.is_symbol(c.expr) and c.expr in joint_velocity_symbols and str(c.expr) not in joint_constraints:
                joint_name = str(Path(erase_type(c.expr))[-1])
                # weight = default_weight if c.expr not in weights else weights[c.expr]
                # joint_constraints[str(c.expr)] = JointConstraint(c.lower, c.upper, weight)
                to_remove.add(k)
                lower_limit = c.lower
                upper_limit = c.upper

                symbols.update(w.free_symbols(lower_limit))
                symbols.update(w.free_symbols(upper_limit))

                sample_period = self.get_god_map().to_symbol(identifier.sample_period)

                # FIXME support vel limit from param server again
                lower_limit = lower_limit * sample_period
                upper_limit = upper_limit * sample_period

                weight = self.get_god_map().get_data(identifier.joint_cost)[joint_name]
                weight = weight * (1. / (upper_limit)) ** 2

                joint_constraints[joint_name] = JointConstraint(lower=lower_limit,
                                                                upper=upper_limit,
                                                                weight=weight)

        hard_constraints = OrderedDict()
        for k, c in sorted(constraints.items()):
            if k not in to_remove:
                symbols.update(w.free_symbols(c.lower))
                symbols.update(w.free_symbols(c.upper))
                symbols.update(w.free_symbols(c.expr))
                hard_constraints[k] = SoftConstraint(lbA=c.lower,
                                                     ubA=c.upper,
                                                     weight=1,
                                                     expression=c.expr,
                                                     goal_constraint=False,
                                                     lower_slack_limit=0,
                                                     upper_slack_limit=0)

        hard_constraints = OrderedDict([((self.robot.get_name(), k), c) for k, c in hard_constraints.items()])

        for s in joint_velocity_symbols:
            joint_name = str(Path(erase_type(s))[-1])
            if joint_name not in joint_constraints:
                if self.get_robot().is_joint_prismatic(joint_name):
                    limit = self.get_god_map().get_data(identifier.default_joint_velocity_linear_limit)
                else:
                    limit = self.get_god_map().get_data(identifier.default_joint_velocity_angular_limit)
                lower, upper = -limit, limit
                weight = self.get_god_map().get_data(identifier.joint_cost)[joint_name]
                weight = weight * (1. / (upper)) ** 2
                joint_constraints[joint_name] = JointConstraint(lower, upper, weight)

        self.get_god_map().register_symbols(symbols)

        self.get_god_map().safe_set_data(identifier.joint_constraint_identifier, joint_constraints)
        self.soft_constraints.update(hard_constraints)
        # self.get_god_map().safe_set_data(identifier.hard_constraint_identifier, hard_constraints)

    def parse_constraints(self, cmd):
        """
        :type cmd: MoveCmd
        :rtype: dict
        """
        for constraint in itertools.chain(cmd.constraints, cmd.joint_constraints, cmd.cartesian_constraints):
            if constraint.type not in allowed_constraint_names():
                # TODO test me
                raise InsolvableException(u'unknown constraint "{}"'.format(constraint.type))
            try:
                C = eval(u'giskardpy.constraints.{}'.format(constraint.type))
            except NameError as e:
                # TODO return next best constraint type
                raise ImplementationException(u'unsupported constraint type')
            try:
                if hasattr(constraint, u'parameter_value_pair'):
                    params = json.loads(constraint.parameter_value_pair)
                else:
                    params = convert_ros_message_to_dictionary(constraint)
                    del params[u'type']
                c = C(self.god_map, **params)

                soft_constraints = c.get_constraints()
                self.soft_constraints.update(soft_constraints)
            except TypeError as e:
                traceback.print_exc()
                raise ImplementationException(help(c.make_constraints))

    def has_robot_changed(self):
        new_urdf = self.get_robot().get_urdf_str()
        result = self.last_urdf != new_urdf
        self.last_urdf = new_urdf
        return result

    def add_collision_avoidance_soft_constraints(self):
        """
        Adds a constraint for each link that pushed it away from its closest point.
        """
        soft_constraints = {}
        number_of_repeller = self.get_god_map().get_data(identifier.number_of_repeller)
        for joint_name in self.get_robot().controlled_joints:
            child_link = self.get_robot().get_child_link_of_joint(joint_name)
            for i in range(number_of_repeller):
                constraint = ExternalCollisionAvoidance(self.god_map, child_link,
                                                        max_weight_distance=self.get_god_map().get_data(
                                                            identifier.distance_thresholds +
                                                            [joint_name, u'max_weight_distance']),
                                                        low_weight_distance=self.get_god_map().get_data(
                                                            identifier.distance_thresholds +
                                                            [joint_name, u'low_weight_distance']),
                                                        zero_weight_distance=self.get_god_map().get_data(
                                                            identifier.distance_thresholds +
                                                            [joint_name, u'zero_weight_distance']),
                                                        idx=i)
                soft_constraints.update(constraint.get_constraints())

        # TODO turn this into a function
        counter = defaultdict(int)
        num_external = len(soft_constraints)
        loginfo('adding {} external collision avoidance constraints'.format(num_external))
        for link_a, link_b in self.get_robot().get_self_collision_matrix():
            link_a, link_b = self.robot.get_chain_reduced_to_controlled_joints(link_a, link_b)
            if not self.get_robot().link_order(link_a, link_b):
                tmp = link_a
                link_a = link_b
                link_b = tmp
            counter[link_a, link_b] += 1

        for link_a, link_b in counter:
            # TODO turn 2 into parameter
            num_of_constraints = min(2, counter[link_a, link_b])
            for i in range(num_of_constraints):
                max_weight_distance = min(self.get_god_map().get_data(identifier.distance_thresholds +
                                                                      [link_a, u'max_weight_distance']),
                                          self.get_god_map().get_data(identifier.distance_thresholds +
                                                                      [link_b, u'max_weight_distance']))
                low_weight_distance = min(self.get_god_map().get_data(identifier.distance_thresholds +
                                                                      [link_a, u'low_weight_distance']),
                                          self.get_god_map().get_data(identifier.distance_thresholds +
                                                                      [link_b, u'low_weight_distance']))
                zero_weight_distance = min(self.get_god_map().get_data(identifier.distance_thresholds +
                                                                       [link_a, u'zero_weight_distance']),
                                           self.get_god_map().get_data(identifier.distance_thresholds +
                                                                       [link_b, u'zero_weight_distance']))
                constraint = SelfCollisionAvoidance(self.god_map, link_a, link_b,
                                                    max_weight_distance=max_weight_distance,
                                                    low_weight_distance=low_weight_distance,
                                                    zero_weight_distance=zero_weight_distance,
                                                    idx=i)
                soft_constraints.update(constraint.get_constraints())
        loginfo('adding {} self collision avoidance constraints'.format(len(soft_constraints) - num_external))
        self.soft_constraints.update(soft_constraints)
