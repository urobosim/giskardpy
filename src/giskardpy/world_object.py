import errno
import hashlib
import os
import pickle
from itertools import combinations
from time import time

import numpy as np
from geometry_msgs.msg import Pose, Quaternion

from giskardpy import logging, identifier
from giskardpy.data_types import SingleJointState
from giskardpy.urdf_object import URDFObject
from giskardpy.utils import msg_to_kdl
from kineverse.model.paths import Path


class WorldObject(URDFObject):
    def init2(self, base_pose=None, controlled_joints=None, path_to_data_folder=u'',
              calc_self_collision_matrix=True, ignored_pairs=None, added_pairs=None, *args, **kwargs):
        self._path_to_data_folder = path_to_data_folder + u'collision_matrix/'
        self.controlled_joints = controlled_joints
        if not ignored_pairs:
            self._ignored_pairs = set()
        else:
            self._ignored_pairs = {tuple(x) for x in ignored_pairs}
        if not added_pairs:
            self._added_pairs = set()
        else:
            self._added_pairs = {tuple(x) for x in added_pairs}
        self._calc_self_collision_matrix = calc_self_collision_matrix
        if base_pose is None:
            p = Pose()
            p.orientation.w = 1
            self.base_pose = p
        # FIXME using .joint_state creates a chicken egg problem in pybulletworldobject
        self._js = self.get_zero_joint_state()
        self._controlled_links = None
        self._self_collision_matrix = set()
        super(WorldObject, self).init2(*args, **kwargs)

    def dump_state(self):
        state = {}
        for name in vars(self):
            if name.startswith('_'):
                state[name] = getattr(self, name)
        return state

    def load_state(self, state):
        for att_name, value in state.items():
            setattr(self, att_name, value)

    @property
    def joint_state(self):
        return self._js

    @joint_state.setter
    def joint_state(self, value):
        new_js = {}
        new_js.update(self._js)
        self._js = new_js
        self._js.update(value)

    @property
    def base_pose(self):
        return self._base_pose

    @base_pose.setter
    def base_pose(self, value):
        """
        :type value: Pose
        :return:
        """
        orientation_vector = np.array([value.orientation.x,
                                       value.orientation.y,
                                       value.orientation.z,
                                       value.orientation.w])
        self._base_pose = value
        self._base_pose.orientation = Quaternion(*orientation_vector / np.linalg.norm(orientation_vector))
        self._root_T_map = msg_to_kdl(self._base_pose).Inverse()

    @property
    def controlled_joints(self):
        # FIXME reinitialize does not handle newly added or removed controllable joints
        if self._controlled_joints is None:
            self._controlled_joints = self.get_controllable_joints()
        return self._controlled_joints

    @controlled_joints.setter
    def controlled_joints(self, value):
        self._controlled_links = None
        self._controlled_joints = value

    def suicide(self):
        pass

    def __del__(self):
        self.suicide()

    # def reinitialize(self):
    #     self._controlled_links = None
    #     super(WorldObject, self).reinitialize()

    def get_controlled_links(self):
        # FIXME expensive
        if not self._controlled_links:
            self._controlled_links = set()
            for joint_name in self.controlled_joints:
                self._controlled_links.update(self.get_sub_tree_link_names_with_collision(joint_name))
        return self._controlled_links

    def get_god_map(self):
        return self._world.god_map

    def get_self_collision_matrix(self):
        """
        :return: A list of link pairs for which we have to calculate self collisions
        """
        return self._self_collision_matrix

    def get_joint_position_symbol(self, joint_name):
        return self.get_god_map().to_symbol(identifier.km_world + [u'data_tree', u'data_tree',
                                                                       self.get_name(), u'joint_state', joint_name,
                                                                       u'position'])

    # @memoize
    def get_controlled_joint_position_symbols(self):
        return set(self.get_joint_position_symbol(joint_name) for joint_name in self.controlled_joints)

    # @memoize
    def get_joint_position_symbols(self):
        return set(self.get_joint_position_symbol(joint_name) for joint_name in self.get_controllable_joints())

    def calc_collision_matrix(self, link_combinations=None, d=0.05, d2=0.0, num_rnd_tries=2000):
        """
        :param link_combinations: set with link name tuples
        :type link_combinations: set
        :param d: distance threshold to detect links that are always in collision
        :type d: float
        :param d2: distance threshold to find links that are sometimes in collision
        :type d2: float
        :param num_rnd_tries:
        :type num_rnd_tries: int
        :return: set of link name tuples which are sometimes in collision.
        :rtype: set
        """
        # TODO computational expansive because of too many collision checks
        logging.loginfo(u'calculating self collision matrix')
        self._world.reset_pb_subworld()
        t = time()
        np.random.seed(1337)
        always = set()

        # find meaningless self-collisions
        for link_a, link_b in link_combinations:
            if self.are_linked(link_a, link_b) or link_a == link_b:
                always.add((link_a, link_b))
        always = always.difference({tuple(x) for x in self._ignored_pairs})
        rest = link_combinations.difference(always)
        self.joint_state = self.get_zero_joint_state()
        always = always.union(self.check_collisions(rest, d))
        rest = rest.difference(always)

        # find meaningful self-collisions
        self.joint_state = self.get_min_joint_state()

        sometimes = self.check_collisions(rest, d2)
        rest = rest.difference(sometimes)
        self.joint_state = self.get_max_joint_state()

        sometimes2 = self.check_collisions(rest, d2)
        rest = rest.difference(sometimes2)
        sometimes = sometimes.union(sometimes2)
        for i in range(num_rnd_tries):
            self.joint_state = self.get_rnd_joint_state()
            sometimes2 = self.check_collisions(rest, d2)
            if len(sometimes2) > 0:
                rest = rest.difference(sometimes2)
                sometimes = sometimes.union(sometimes2)
        sometimes = sometimes.union(self._added_pairs)
        logging.loginfo(u'calculated self collision matrix in {:.3f}s'.format(time() - t))
        return sometimes

    def get_possible_collisions(self, link):
        # TODO speed up by saving this
        possible_collisions = set()
        for link1, link2 in self.get_self_collision_matrix():
            if link == link1:
                possible_collisions.add(link2)
            elif link == link2:
                possible_collisions.add(link1)
        return possible_collisions

    def check_collisions(self, link_combinations, distance):
        self._world.sync_bullet_world()
        in_collision = set()
        for link_a, link_b in link_combinations:
            if self.in_collision(link_a, link_b, distance):
                in_collision.add((link_a, link_b))
        return in_collision

    def in_collision(self, link_a, link_b, distance):
        return self._world.in_collision(self.get_name(), link_a, self.get_name(), link_b, distance)

    def get_zero_joint_state(self):
        # FIXME 0 might not be a valid joint value
        return self.generate_joint_state(lambda x: 0)

    def get_max_joint_state(self):
        def f(joint_name):
            _, upper_limit = self.get_joint_limits(joint_name)
            if upper_limit is None:
                return np.pi * 2
            return upper_limit

        return self.generate_joint_state(f)

    def get_min_joint_state(self):
        def f(joint_name):
            lower_limit, _ = self.get_joint_limits(joint_name)
            if lower_limit is None:
                return -np.pi * 2
            return lower_limit

        return self.generate_joint_state(f)

    def get_rnd_joint_state(self):
        def f(joint_name):
            lower_limit, upper_limit = self.get_joint_limits(joint_name)
            if lower_limit is None:
                return np.random.random() * np.pi * 2
            lower_limit = max(lower_limit, -10)
            upper_limit = min(upper_limit, 10)
            return (np.random.random() * (upper_limit - lower_limit)) + lower_limit

        return self.generate_joint_state(f)

    def generate_joint_state(self, f):
        """
        :param f: lambda joint_info: float
        :return:
        """
        js = {}
        for joint_name in self.get_controllable_joints():
            sjs = SingleJointState()
            sjs.name = joint_name
            sjs.position = f(joint_name)
            js[joint_name] = sjs
        return js

    def add_self_collision_entries(self, object_name):
        link_pairs = {(object_name, link_name) for link_name in self.get_link_names()}
        link_pairs.remove((object_name, object_name))
        self_collision_with_object = self.calc_collision_matrix(link_pairs)
        self._self_collision_matrix.update(self_collision_with_object)

    def remove_self_collision_entries(self, object_name):
        self._self_collision_matrix = {(link1, link2) for link1, link2 in self.get_self_collision_matrix()
                                       if link1 != object_name and link2 != object_name}

    def init_self_collision_matrix(self):
        self.update_self_collision_matrix(added_links=set(combinations(self.get_link_names_with_collision(), 2)))

    def get_hash(self):
        history = self._world.km_model.get_history_of(Path(self.get_name()))
        return hashlib.md5(str([x.tag for x in history])).hexdigest()

    def update_self_collision_matrix(self, added_links=None, removed_links=None):
        # return
        if not self.load_self_collision_matrix(self._path_to_data_folder):
            if added_links is None:
                added_links = set()
            if removed_links is None:
                removed_links = set()
            self._self_collision_matrix = {x for x in self._self_collision_matrix if x[0] not in removed_links and
                                           x[1] not in removed_links}
            self._self_collision_matrix.update(self.calc_collision_matrix(added_links))
            self.safe_self_collision_matrix(self._path_to_data_folder)

    def load_self_collision_matrix(self, path):
        """
        :rtype: bool
        """
        path = u'{}/{}/{}'.format(path, self.get_name(), self.get_hash())
        if os.path.isfile(path):
            with open(path) as f:
                self._self_collision_matrix = pickle.load(f)
                logging.loginfo(u'loaded self collision matrix {}'.format(path))
                return True
        return False

    def safe_self_collision_matrix(self, path):
        # pass
        path = u'{}/{}/{}'.format(path, self.get_name(), self.get_hash())
        if not os.path.exists(os.path.dirname(path)):
            try:
                dir_name = os.path.dirname(path)
                if dir_name != u'':
                    os.makedirs(dir_name)
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise
        with open(path, u'w') as file:
            logging.loginfo(u'saved self collision matrix {}'.format(path))
            pickle.dump(self._self_collision_matrix, file)

    def as_marker_msg(self, ns=u'', id=1):
        m = super(WorldObject, self).as_marker_msg(ns, id)
        m.pose = self.base_pose
        return m

    # def attach_urdf_object(self, urdf_object, parent_link, pose):
    #     super(WorldObject, self).attach_urdf_object(urdf_object, parent_link, pose)
    #     self.update_self_collision_matrix(added_links=set(product(self.get_links_with_collision(),
    #                                                               urdf_object.get_links_with_collision())))
    #     # TODO set joint state for controllable joints of added urdf?
    #
    # def detach_sub_tree(self, joint_name):
    #     sub_tree = super(WorldObject, self).detach_sub_tree(joint_name)
    #     self.update_self_collision_matrix(removed_links=sub_tree.get_link_names())
    #     return sub_tree

    # def reset(self):
    #     super(WorldObject, self).reset()
    #     self.update_self_collision_matrix()

    def get_fk_expression(self, root_link, tip_link):
        """
        :type root_link: str
        :type tip_link: str
        :return: 4d matrix describing the transformation from root_link to tip_link
        :rtype: spw.Matrix
        """
        root_path = self.get_link_path(root_link)
        tip_path = self.get_link_path(tip_link)
        return self._world.get_fk_expression(root_path, tip_path)

    def get_fk_pose(self, root_link, tip_link):
        root_path = self.get_link_path(root_link)
        tip_path = self.get_link_path(tip_link)
        return self._world.get_fk_pose(root_path, tip_path)

    def get_fk_np(self, root_link, tip_link):
        root_path = self.get_link_path(root_link)
        tip_path = self.get_link_path(tip_link)
        return self._world.get_fk_np(root_path, tip_path)
