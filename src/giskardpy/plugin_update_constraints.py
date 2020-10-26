import difflib
import inspect
import itertools
import json
import traceback
from collections import OrderedDict
from collections import defaultdict
from time import time

from giskard_msgs.msg import MoveCmd, CollisionEntry
from py_trees import Status

import giskardpy.constraints
import giskardpy.identifier as identifier
import kineverse.gradients.common_math as cm
# import kineverse.gradients.gradient_math as gm
from giskardpy import cas_wrapper as w
from giskardpy.constraints import SelfCollisionAvoidance, ExternalCollisionAvoidance
from giskardpy.data_types import JointConstraint, SoftConstraint
from giskardpy.exceptions import ImplementationException, UnknownConstraintException, InvalidGoalException, \
    ConstraintInitalizationException, GiskardException
from giskardpy.logging import loginfo
from giskardpy.plugin_action_server import GetGoal
from kineverse.gradients.diff_logic import DiffSymbol, erase_type, Position
from kineverse.model.paths import Path
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary



class GoalToConstraints(GetGoal):
    # FIXME no error msg when constraint has missing parameter
    def __init__(self, name, as_name):
        GetGoal.__init__(self, name, as_name)
        self.used_joints = set()

        self.controlled_joints = set()
        self.controllable_links = set()
        self.last_urdf = None
        self.allowed_constraint_types = {x[0]: x[1] for x in inspect.getmembers(giskardpy.constraints) if
                                         inspect.isclass(x[1])}

        self.rc_prismatic_velocity = self.get_god_map().get_data(identifier.rc_prismatic_velocity)
        self.rc_continuous_velocity = self.get_god_map().get_data(identifier.rc_continuous_velocity)
        self.rc_revolute_velocity = self.get_god_map().get_data(identifier.rc_revolute_velocity)
        self.rc_other_velocity = self.get_god_map().get_data(identifier.rc_other_velocity)

    def initialise(self):
        self.get_god_map().set_data(identifier.collision_goal, None)
        self.clear_blackboard_exception()

    def update(self):
        # TODO make this interruptable
        # TODO try catch everything

        try:
            move_cmd = self.get_god_map().get_data(identifier.next_move_goal)  # type: MoveCmd
        except KeyError:
            return Status.FAILURE
        else:
            if not move_cmd:
                return Status.FAILURE

        self.get_god_map().set_data(identifier.constraints_identifier, {})

        # self.get_robot()._create_constraints(self.get_god_map())

        self.soft_constraints = {}
        if not (self.get_god_map().get_data(identifier.check_reachability)):
            self.get_god_map().set_data(identifier.maximum_collision_threshold, 0)
            self.add_collision_avoidance_soft_constraints(move_cmd.collisions)

        try:
            self.parse_constraints(move_cmd)
        except AttributeError:
            self.raise_to_blackboard(InvalidGoalException(u'couldn\'t transform goal'))
            traceback.print_exc()
            return Status.SUCCESS
        except Exception as e:
            self.raise_to_blackboard(e)
            traceback.print_exc()
            return Status.SUCCESS

        self.get_god_map().set_data(identifier.collision_goal, move_cmd.collisions)
        self.get_god_map().set_data(identifier.soft_constraint_identifier, self.soft_constraints)
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
        self.add_object_constraints()

        return Status.SUCCESS

    def position_identifier_from_velocity_symbol(self, s):
        return self.get_god_map().symbol_to_identifier(Position(erase_type(s)))

    def add_object_constraints(self):
        world = self.get_god_map().get_data(identifier.world)
        # FIXME you also still have to check the soft constraints for e.g. kitchen joints
        joint_position_symbols = set()
        for object_name in self.get_world().get_object_names():
            joint_position_symbols |= set(self.get_world().get_object(object_name).get_joint_position_symbols())

        constraint_symbols = set()
        for constraint_name, constraint in self.soft_constraints.items():
            constraint_symbols |= set(w.free_symbols(constraint.expression))


        joint_position_symbols = joint_position_symbols.intersection(constraint_symbols)
        joint_position_symbols |= self.get_robot().get_controlled_joint_position_symbols()
        self.get_god_map().set_data(identifier.controlled_joint_symbols, joint_position_symbols)

        joint_velocity_symbols = {DiffSymbol(s) for s in joint_position_symbols}

        constraints = world.km_model.get_constraints_by_symbols(joint_velocity_symbols.union(joint_position_symbols))

        joint_constraints = OrderedDict()
        to_remove = set()
        symbols = set()

        sample_period = self.get_god_map().identivier_to_symbol(identifier.sample_period)
        for k, c in sorted(constraints.items()):
            if cm.is_symbol(c.expr) and c.expr in joint_velocity_symbols and str(c.expr) not in joint_constraints:
                joint_name = str(Path(erase_type(c.expr))[-1])
                to_remove.add(k)
                lower_limit = c.lower
                upper_limit = c.upper

                symbols.update(w.free_symbols(lower_limit))
                symbols.update(w.free_symbols(upper_limit))

                # FIXME support vel limit from param server again
                lower_limit = lower_limit * sample_period
                upper_limit = upper_limit * sample_period

                weight = self.get_god_map().get_data(identifier.joint_weight)[joint_name]
                weight = weight * (1. / (upper_limit)) ** 2

                key = self.position_identifier_from_velocity_symbol(c.expr)

                joint_constraints[key] = JointConstraint(lower=lower_limit,
                                                         upper=upper_limit,
                                                         weight=weight,
                                                         linear_weight=0)

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
                                                     upper_slack_limit=0,
                                                     linear_weight=0)

        hard_constraints = OrderedDict([((self.robot.get_name(), k), c) for k, c in hard_constraints.items()])

        for s in joint_velocity_symbols:
            joint_name = str(Path(erase_type(s))[-1])
            key = self.position_identifier_from_velocity_symbol(s)
            if key not in joint_constraints:
                if self.get_robot().is_joint_prismatic(joint_name):
                    limit = self.get_god_map().get_data(identifier.joint_velocity_linear_limit_default)
                else:
                    limit = self.get_god_map().get_data(identifier.joint_velocity_angular_limit_default)
                lower, upper = -limit, limit
                lower *= sample_period
                upper *= sample_period
                weight = self.get_god_map().get_data(identifier.joint_weight)[joint_name]
                weight = weight * (1. / (upper)) ** 2
                joint_constraints[key] = JointConstraint(lower, upper, weight, 0)

        joint_constraints = OrderedDict((key, value) for key, value in sorted(joint_constraints.items()))

        self.get_god_map().register_symbols(symbols)

        self.get_god_map().set_data(identifier.joint_constraint_identifier, joint_constraints)
        self.soft_constraints.update(hard_constraints)
        # self.get_god_map().set_data(identifier.hard_constraint_identifier, hard_constraints)

    def parse_constraints(self, cmd):
        """
        :type cmd: MoveCmd
        :rtype: dict
        """
        loginfo(u'parsing goal message')
        for constraint in itertools.chain(cmd.constraints, cmd.joint_constraints, cmd.cartesian_constraints):
            try:
                loginfo(u'adding constraint of type: \'{}\''.format(constraint.type))
                C = self.allowed_constraint_types[constraint.type]
            except KeyError:
                matches = ''
                for s in self.allowed_constraint_types.keys():
                    sm = difflib.SequenceMatcher(None, str(constraint.type).lower(), s.lower())
                    ratio = sm.ratio()
                    if ratio >= 0.5:
                        matches = matches + s + '\n'
                if matches != '':
                    raise UnknownConstraintException(
                        u'unknown constraint {}. did you mean one of these?:\n{}'.format(constraint.type, matches))
                else:
                    available_constraints = '\n'.join([x for x in self.allowed_constraint_types.keys()]) + '\n'
                    raise UnknownConstraintException(
                        u'unknown constraint {}. available constraint types:\n{}'.format(constraint.type,
                                                                                         available_constraints))

            try:
                if hasattr(constraint, u'parameter_value_pair'):
                    params = json.loads(constraint.parameter_value_pair)
                else:
                    params = convert_ros_message_to_dictionary(constraint)
                    del params[u'type']

                c = C(self.god_map, **params)
            except Exception as e:
                traceback.print_exc()
                doc_string = C.__init__.__doc__
                error_msg = u'Initialization of "{}" constraint failed: \n {} \n'.format(C.__name__, e)
                if doc_string is not None:
                    error_msg = error_msg + doc_string
                if not isinstance(e, GiskardException):
                    raise ConstraintInitalizationException(error_msg)
                raise e
            try:
                soft_constraints = c.get_constraints()
                self.soft_constraints.update(soft_constraints)
            except Exception as e:
                traceback.print_exc()
                if not isinstance(e, GiskardException):
                    raise ConstraintInitalizationException(e)
                raise e
        loginfo(u'done parsing goal message')

    def add_collision_avoidance_soft_constraints(self, collision_cmds):
        """
        Adds a constraint for each link that pushed it away from its closest point.
        :type collision_cmds: list of CollisionEntry
        """
        # FIXME this only catches the most obvious cases
        soft_threshold = None
        for collision_cmd in collision_cmds:
            if collision_cmd.type == CollisionEntry.AVOID_ALL_COLLISIONS or \
                    self.get_world().is_avoid_all_collision(collision_cmd):
                soft_threshold = collision_cmd.min_dist

        if not collision_cmds or not self.get_world().is_allow_all_collision(collision_cmds[-1]):
            self.add_external_collision_avoidance_constraints(soft_threshold_override=soft_threshold)
        if not collision_cmds or (not self.get_world().is_allow_all_collision(collision_cmds[-1]) and
                                  not self.get_world().is_allow_all_self_collision(collision_cmds[-1])):
            self.add_self_collision_avoidance_constraints()

    def add_external_collision_avoidance_constraints(self, soft_threshold_override=None):
        soft_constraints = {}
        number_of_repeller = self.get_god_map().get_data(identifier.external_collision_avoidance_repeller)
        number_of_repeller_eef = self.get_god_map().get_data(identifier.external_collision_avoidance_repeller_eef)
        eef_joints = self.get_robot().get_controlled_leaf_joints()
        maximum_distance = self.get_god_map().get_data(identifier.maximum_collision_threshold)
        # TODO add root joint?
        remaining_joints = [joint_name for joint_name in self.get_robot().controlled_joints
                            if joint_name not in eef_joints]
        for joint_name in remaining_joints:
            child_links = self.get_robot().get_directly_controllable_collision_links(joint_name)
            if child_links:
                for i in range(number_of_repeller):
                    child_link = self.get_robot().get_child_link_of_joint(joint_name)
                    hard_threshold = self.get_god_map().get_data(identifier.external_collision_avoidance_distance +
                                                                [joint_name, u'hard_threshold'])
                    if soft_threshold_override is not None:
                        soft_threshold = soft_threshold_override
                    else:
                        soft_threshold = self.get_god_map().get_data(identifier.external_collision_avoidance_distance +
                                                                    [joint_name, u'soft_threshold'])
                    maximum_distance = max(maximum_distance, soft_threshold)
                    constraint = ExternalCollisionAvoidance(self.god_map, child_link,
                                                            hard_threshold=hard_threshold,
                                                            soft_threshold=soft_threshold,
                                                            idx=i,
                                                            num_repeller=number_of_repeller)
                    soft_constraints.update(constraint.get_constraints())

        for joint_name in eef_joints:
            child_link = self.get_robot().get_child_link_of_joint(joint_name)
            for i in range(number_of_repeller_eef):
                hard_threshold = self.get_god_map().get_data(identifier.external_collision_avoidance_distance +
                                                             [joint_name, u'hard_threshold'])
                if soft_threshold_override is not None:
                    soft_threshold = soft_threshold_override
                else:
                    soft_threshold = self.get_god_map().get_data(identifier.external_collision_avoidance_distance +
                                                                 [joint_name, u'soft_threshold'])
                maximum_distance = max(maximum_distance, soft_threshold)
                constraint = ExternalCollisionAvoidance(self.god_map, child_link,
                                                        hard_threshold=hard_threshold,
                                                        soft_threshold=soft_threshold,
                                                        idx=i,
                                                        num_repeller=number_of_repeller_eef)
                soft_constraints.update(constraint.get_constraints())

        num_external = len(soft_constraints)
        loginfo('adding {} external collision avoidance constraints'.format(num_external))
        self.soft_constraints.update(soft_constraints)
        self.get_god_map().set_data(identifier.maximum_collision_threshold, maximum_distance)

    def add_self_collision_avoidance_constraints(self):
        counter = defaultdict(int)
        soft_constraints = {}
        number_of_repeller = self.get_god_map().get_data(identifier.self_collision_avoidance_repeller)
        maximum_distance = self.get_god_map().get_data(identifier.maximum_collision_threshold)
        for link_a_o, link_b_o in self.get_robot().get_self_collision_matrix():
            link_a, link_b = self.robot.get_chain_reduced_to_controlled_joints(link_a_o, link_b_o)
            if not self.get_robot().link_order(link_a, link_b):
                link_a, link_b = link_b, link_a
            counter[link_a, link_b] += 1

        for link_a, link_b in counter:
            num_of_constraints = min(1, counter[link_a, link_b])
            for i in range(num_of_constraints):
                thresholds = self.get_god_map().get_data(identifier.self_collision_avoidance_distance)
                key = u'{}, {}'.format(link_a, link_b)
                key_r = u'{}, {}'.format(link_b, link_a)
                if key in thresholds:
                    hard_threshold = thresholds[key][u'hard_threshold']
                    soft_threshold = thresholds[key][u'soft_threshold']
                elif key_r in thresholds:
                    hard_threshold = thresholds[key_r][u'hard_threshold']
                    soft_threshold = thresholds[key_r][u'soft_threshold']
                else:
                    # TODO minimum is not the best if i reduce to the links next to the controlled chains
                    #   should probably add symbols that retrieve the values for the current pair
                    hard_threshold = min(thresholds[link_a][u'hard_threshold'],
                                         thresholds[link_b][u'hard_threshold'])
                    soft_threshold = min(thresholds[link_a][u'soft_threshold'],
                                         thresholds[link_b][u'soft_threshold'])
                maximum_distance = max(maximum_distance, soft_threshold)
                constraint = SelfCollisionAvoidance(self.god_map,
                                                    link_a=link_a,
                                                    link_b=link_b,
                                                    hard_threshold=hard_threshold,
                                                    soft_threshold=soft_threshold,
                                                    idx=i,
                                                    num_repeller=number_of_repeller)
                soft_constraints.update(constraint.get_constraints())
        loginfo('adding {} self collision avoidance constraints'.format(len(soft_constraints)))
        self.soft_constraints.update(soft_constraints)
        self.get_god_map().set_data(identifier.maximum_collision_threshold, maximum_distance)
