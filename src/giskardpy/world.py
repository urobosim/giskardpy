from collections import OrderedDict
from collections import defaultdict
from copy import deepcopy
from itertools import product

import betterpybullet as pb
import numpy as np
import urdf_parser_py.urdf as up
from betterpybullet import ClosestPair
from betterpybullet import ContactPoint
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from giskard_msgs.msg import CollisionEntry, WorldBody

from giskardpy import cas_wrapper as w
from giskardpy import identifier
from giskardpy import logging
from giskardpy.data_types import Collisions, Collision
from giskardpy.exceptions import RobotExistsException, DuplicateNameException, PhysicsWorldException, \
    UnknownBodyException, UnsupportedOptionException, CorruptShapeException
from giskardpy.god_map import GodMap
from giskardpy.input_system import PoseStampedInput
from giskardpy.robot import Robot
from giskardpy.urdf_object import URDFObject, hacky_urdf_parser_fix
from giskardpy.utils import KeyDefaultDict, memoize, homo_matrix_to_pose, suppress_stderr
from kineverse.model.geometry_model import GeometryModel
from kineverse.model.paths import Path, CPath
from kineverse.operations.basic_operations import ExecFunction
from kineverse.operations.urdf_operations import load_urdf, FixedJoint, CreateURDFFrameConnection
from kineverse.urdf_fix import urdf_filler

# TODO: Find propper location for this
def np_inverse_frame(frame):
    """
    :param frame: 4x4 Matrix
    :type frame: Matrix
    :return: 4x4 Matrix
    :rtype: Matrix
    """
    inv = np.eye(4)
    inv[:3, :3] = frame[:3, :3].T
    inv[:3, 3] = np.dot(-inv[:3, :3], frame[:3, 3])
    return inv

class World(object):
    world_frame = u'map'
    world_frame_path = Path(world_frame)

    def __init__(self, god_map, prefix=tuple(), path_to_data_folder=u''):
        self.god_map = god_map  # type: GodMap
        self._fks = {}
        self.__prefix = '/'.join(prefix)
        self._objects_names = []
        self._robot_name = u'robot'
        self.km_model = GeometryModel()
        self.attached_objects = {}
        if path_to_data_folder is None:
            path_to_data_folder = u''
        self._path_to_data_folder = path_to_data_folder
        self.query = None
        self.pb_subworld = None

    # General ----------------------------------------------------------------------------------------------------------

    def soft_reset(self):
        """
        keeps robot and other important objects like ground plane
        """
        self.remove_all_objects()
        self.reset_cache()
        # self.km_model.dispatch_events()
        # if self._robot_name is not None:
        #     self.robot.reset()

    def hard_reset(self):
        """
        removes everything
        """
        self.soft_reset()
        self.remove_robot()

    def sync_bullet_world(self):
        symbols = self.pb_subworld.pose_generator.str_params
        data = self.god_map.get_values(symbols)
        pb.batch_set_transforms(self.pb_subworld.collision_objects, self.pb_subworld.pose_generator.call2(data))

    def in_collision(self, body_a, link_a, body_b, link_b, distance):
        obj_a = self.pb_subworld.named_objects[self.get_link_path(body_a, link_a)]
        obj_b = self.pb_subworld.named_objects[self.get_link_path(body_b, link_b)]
        result = self.pb_subworld.world.get_distance(obj_a, obj_b, distance)
        return len(result) > 0 and result[0].distance < distance

    def get_closest_distances(self, object_name, link_name, object_b, distance):
        obj_a = self.pb_subworld.named_objects[self.get_link_path(object_name, link_name)]
        world_object_b = self.get_object(object_b)
        body_bs = [self.pb_subworld.named_objects[path] for path in world_object_b.get_link_paths() if
                   path in self.pb_subworld.named_objects]
        contacts = self.pb_subworld.world.get_closest_filtered(obj_a, body_bs, distance)
        result = OrderedDict()
        for contact in contacts:  # type: ClosestPair
            path = Path(self.pb_subworld.object_name_map[contact.obj_b])
            link_b_name = path[-1]
            for p in contact.points:  # type: ContactPoint
                if object_name == object_b:
                    if link_b_name == link_name:
                        continue
                    if object_name == self.robot.get_name():
                        if (link_name, link_b_name) not in self.robot.get_self_collision_matrix() and \
                                (link_b_name, link_name) not in self.robot.get_self_collision_matrix():
                            continue
                result[path] = p.distance
        return OrderedDict(sorted([(key, value) for key, value in result.items()], key=lambda x: x[1]))

    def init_collision_avoidance_data_structures(self, cut_off_distances):
        self.reverse_map_a = {}
        self.reverse_map_b = {}
        self.query = defaultdict(set)

        for (robot_link, body_b, link_b), distance in cut_off_distances.items():
            distance *= 3
            obj_a = self.pb_subworld.named_objects[self.robot.get_link_path(robot_link)]
            self.reverse_map_a[obj_a] = robot_link
            if link_b == CollisionEntry.ALL:
                obj_bs = set()
                for path in self.get_object(body_b).get_link_paths():
                    if path in self.pb_subworld.named_objects:
                        obj_b = self.pb_subworld.named_objects[path]
                        obj_bs.add((obj_b, distance))
                        self.reverse_map_b[obj_b] = (body_b, link_b)
                self.query[obj_a] |= obj_bs
            else:
                path = self.get_object(body_b).get_link_path(link_b)
                if path in self.pb_subworld.named_objects:
                    obj_b = self.pb_subworld.named_objects[path]
                    self.reverse_map_b[obj_b] = (body_b, link_b)
                    self.query[obj_a].add((obj_b, distance))

        for key in self.query.keys():
            self.query[key] = list(self.query[key])

    @profile
    def check_collisions(self, cut_off_distances, collision_list_size=20):
        self.sync_bullet_world()
        collisions = Collisions(self)
        if self.query is None:
            self.init_collision_avoidance_data_structures(cut_off_distances)

        result = self.pb_subworld.world.get_closest_filtered_POD_batch(self.query)
        for obj_a, contacts in result.items():
            map_T_a = obj_a.np_transform
            link_a = self.reverse_map_a[obj_a]
            for contact in contacts:  # type: ClosestPair
                map_T_b = contact.obj_b.np_transform
                # b_T_map = contact.obj_b.np_inv_transform
                body_b, link_b = self.reverse_map_b[contact.obj_b]
                # b_T_map = self.get_fk_np(self.robot.get_link_path(link_b), 'map')
                for p in contact.points:  # type: ContactPoint
                    map_P_a = map_T_a.dot(p.point_a)
                    map_P_b = map_T_b.dot(p.point_b)

                    c = Collision(link_a, body_b, link_b, map_P_a, map_P_b, p.normal_world_b, p.distance)
                    collisions.add(c)
        return collisions

    # Objects ----------------------------------------------------------------------------------------------------------

    def add_object(self, urdf, name=None):
        name = self.add_thing(urdf, name)
        self._objects_names.append(name)
        logging.loginfo(u'--> added {} to world'.format(name))

    def add_thing(self, urdf, name=None, **kwargs):
        """
        :type object_: URDFObject
        """
        if isinstance(urdf, WorldBody):
            name, urdf = self.world_body_to_urdf_str(urdf)
        with suppress_stderr():
            urdf_obj = urdf_filler(up.URDF.from_xml_string(hacky_urdf_parser_fix(urdf)), fill_with_visual=False)
        if name is None:
            name = urdf_obj.name

        if self.km_model.has_data(name):
            raise DuplicateNameException(u'Something with name \'{}\' already exists'.format(name))

        root_pose = PoseStampedInput(self.god_map.identivier_to_symbol,
                                     translation_prefix=[self.__prefix, u'km_model', u'data_tree', u'data_tree', name,
                                                         u'base_pose',
                                                         u'position'],
                                     rotation_prefix=[self.__prefix, u'km_model', u'data_tree', u'data_tree', name,
                                                      u'base_pose',
                                                      u'orientation']).get_frame()

        limit_map = load_urdf(ks=self.km_model,
                              prefix=Path(str(name)),
                              urdf=urdf_obj,
                              reference_frame=self.world_frame,
                              joint_prefix=Path(
                                  [self.__prefix, u'km_model', u'data_tree', u'data_tree', name, u'joint_state']),
                              limit_prefix=Path(
                                  [self.__prefix, u'km_model', u'data_tree', u'data_tree', name, u'joint_limits']),
                              robot_class=Robot,
                              root_transform=root_pose,
                              limit_symbols=True,
                              name_override=name)
        obj = self.km_model.get_data(name)  # type: Robot
        obj.init2(world=self, limit_map=limit_map,
                  path_to_data_folder=self.god_map.get_data(identifier.data_folder), **kwargs)

        self.km_model.register_on_model_changed(Path(name), obj.reset_cache)
        self.km_model.register_on_model_changed(Path(name), self.init_fast_fks)
        self.reset_cache()
        obj.get_joint_position_symbols()  # FIXME used to call to_symbol

        return name

    def world_body_to_urdf_str(self, world_body):
        links = []
        joints = []
        urdf_name = world_body.name
        if world_body.type == world_body.PRIMITIVE_BODY or world_body.type == world_body.MESH_BODY:
            if world_body.shape.type == world_body.shape.BOX:
                geometry = up.Box(world_body.shape.dimensions)
            elif world_body.shape.type == world_body.shape.SPHERE:
                geometry = up.Sphere(world_body.shape.dimensions[0])
            elif world_body.shape.type == world_body.shape.CYLINDER:
                geometry = up.Cylinder(world_body.shape.dimensions[world_body.shape.CYLINDER_RADIUS],
                                       world_body.shape.dimensions[world_body.shape.CYLINDER_HEIGHT])
            elif world_body.shape.type == world_body.shape.CONE:
                raise TypeError(u'primitive shape cone not supported')
            elif world_body.type == world_body.MESH_BODY:
                geometry = up.Mesh(world_body.mesh)
            else:
                raise CorruptShapeException(u'primitive shape \'{}\' not supported'.format(world_body.shape.type))
            # FIXME test if this works on 16.04
            try:
                link = up.Link(urdf_name)
                link.add_aggregate(u'visual', up.Visual(geometry,
                                                        material=up.Material(u'green', color=up.Color(0, 1, 0, 1))))
                link.add_aggregate(u'collision', up.Collision(geometry))
            except AssertionError:
                link = up.Link(world_body.name,
                               visual=up.Visual(geometry, material=up.Material(u'green', color=up.Color(0, 1, 0, 1))),
                               collision=up.Collision(geometry))
            links.append(link)
        elif world_body.type == world_body.URDF_BODY:
            result_str = world_body.urdf
            urdf_name = world_body.name
            return urdf_name, result_str
        else:
            raise CorruptShapeException(u'world body type \'{}\' not supported'.format(world_body.type))

        r = up.Robot(urdf_name)
        r.version = u'1.0'
        for link in links:
            r.add_link(link)
        for joint in joints:
            r.add_joint(joint)
        return urdf_name, r.to_xml_string()

    def set_object_pose(self, name, pose):
        """
        :type pose: Pose
        :return:
        """
        self.get_object(name).base_pose = pose

    def get_object(self, name):
        """
        :type name: Union[str, unicode]
        :rtype: Robot
        """
        return self.km_model.get_data(name)

    def get_objects(self):
        return [self.get_object(name) for name in self._objects_names]

    def get_object_names(self):
        """
        :rtype: list
        """
        return self._objects_names

    def has_object(self, name):
        """
        Checks for objects with the same name.
        :type name: Union[str, unicode]
        :rtype: bool
        """
        return name in self.get_object_names()

    def set_object_joint_state(self, name, joint_state):
        """
        :type name: Union[str, unicode]
        :param joint_state: joint name -> SingleJointState
        :type joint_state: dict
        """
        self.get_object(name).joint_state = joint_state

    def remove_object(self, name):
        if self.has_object(name):
            self.__remove_object(name)
        else:
            raise UnknownBodyException(u'can\'t remove object \'{}\', because it doesn\' exist'.format(name))
        self._objects_names.remove(name)
        self.reset_cache()
        self.km_model.clean_structure()
        self.km_model.dispatch_events()

    def __remove_object(self, name):
        self.__remove_thing(name)
        logging.loginfo(u'<-- removed object {} from world'.format(name))

    def __remove_thing(self, name):
        self.km_model.deregister_on_model_changed(self.km_model.get_data(name).reset_cache)
        operations = self.km_model.get_history_of(Path(name))
        for tagged_operation in reversed(operations):
            self.km_model.remove_operation(tagged_operation.tag)

    def remove_all_objects(self):
        for attach_object_name in self.robot.attached_objects:
            self.detach(attach_object_name)
        for object_name in self.get_object_names():
            # I'm not using remove object, because has object ignores hidden objects in pybullet world
            self.__remove_object(object_name)
            logging.loginfo(u'<-- removed object {} from world'.format(object_name))
        self._objects_names = []
        self.reset_cache()
        self.km_model.clean_structure()
        self.km_model.dispatch_events()

    # Robot ------------------------------------------------------------------------------------------------------------

    def add_robot(self, robot_urdf, base_pose, controlled_joints, ignored_pairs, added_pairs):
        """
        :type robot: giskardpy.world_object.WorldObject
        :type controlled_joints: list
        :type base_pose: PoseStamped
        """
        if self.has_robot():
            raise RobotExistsException(u'A robot is already loaded')
        self.add_thing(robot_urdf,
                       name=self._robot_name,
                       base_pose=base_pose,
                       controlled_joints=controlled_joints,
                       ignored_pairs=ignored_pairs,
                       added_pairs=added_pairs)
        logging.loginfo(u'--> added {} to world'.format(self._robot_name))

    @property
    def robot(self):
        """
        :rtype: Robot
        """
        return self.get_object(self._robot_name)

    def has_robot(self):
        """
        :rtype: bool
        """
        try:
            self.robot
        except:
            return False
        return True

    def set_robot_joint_state(self, joint_state):
        """
        Set the current joint state readings for a robot in the world.
        :param joint_state: joint name -> SingleJointState
        :type joint_state: dict
        """
        self.set_object_joint_state(self._robot_name, joint_state)

    def remove_robot(self):
        self.__remove_thing(self._robot_name)
        logging.loginfo(u'<-- removed robot {} from world'.format(self._robot_name))
        self._robot_name = None
        self.reset_cache()
        # self.km_model.clean_structure()
        # self.km_model.dispatch_events()

    def reset_pb_subworld(self):
        self.pb_subworld = self.km_model.get_active_geometry(self.km_model._symbol_co_map.keys())
        symbols = self.pb_subworld.free_symbols
        for symbol in symbols:
            self.god_map.register_symbol(symbol)

    @memoize
    def get_split_chain(self, root_path, tip_path, joints=True, links=True, fixed=True):
        if root_path == tip_path:
            return [], [], []
        root_chain = self.get_simple_chain(self.world_frame, root_path, False, True, True)
        tip_chain = self.get_simple_chain(self.world_frame, tip_path, False, True, True)
        for i in range(min(len(root_chain), len(tip_chain))):
            if root_chain[i] != tip_chain[i]:
                break
        else:  # if not break
            i += 1
        connection = tip_chain[i - 1]
        root_chain = self.get_simple_chain(connection, root_path, joints, links, fixed)
        if links:
            root_chain = root_chain[1:]
        root_chain.reverse()
        tip_chain = self.get_simple_chain(connection, tip_path, joints, links, fixed)
        if links:
            tip_chain = tip_chain[1:]
        return root_chain, [connection] if links else [], tip_chain

    @memoize
    def get_chain(self, root, tip, joints=True, links=True, fixed=True):
        """
        :type root: str
        :type tip: str
        :type joints: bool
        :type links: bool
        :type fixed: bool
        :rtype: list
        """
        root_chain, connection, tip_chain = self.get_split_chain(root, tip, joints, links, fixed)
        return root_chain + connection + tip_chain

    @memoize
    def get_simple_chain(self, root_path, tip_path, joints=True, links=True, fixed=True):
        """
        :type root_path: Path
        :type tip_path: Path
        :type joints: bool
        :type links: bool
        :type fixed: bool
        :rtype: list
        """
        chain = []
        if links:
            chain.append(tip_path)
        link_path = tip_path
        while link_path != root_path:
            link = self.km_model.get_data(link_path)
            joint = link.parent_joint
            parent = link.parent

            if joints:
                if fixed or not self.km_model.get_data(joint).type == 'fixed':  # fixme
                    chain.append(joint)
            if links:
                chain.append(parent)
            link_path = parent
        chain.reverse()
        return chain

    def get_fk_expression(self, root_path, tip_path):
        """
        :type root_link: str
        :type tip_link: str
        :return: 4d matrix describing the transformation from root_link to tip_link
        :rtype: spw.Matrix
        """
        fk = w.eye(4)
        root_chain, _, tip_chain = self.get_split_chain(root_path, tip_path, joints=False, links=True)
        for frame_name in root_chain:
            fk = w.dot(fk, w.inverse_frame(self.km_model.get_data(frame_name).to_parent))
        for frame_name in tip_chain:
            fk = w.dot(fk, self.km_model.get_data(frame_name).to_parent)
        # FIXME there is some reference fuckup going on, but i don't know where; deepcopy is just a quick fix
        return deepcopy(fk)

    def get_fk_pose(self, root_path, tip_path):
        # try:
        homo_m = self.get_fk_np(root_path, tip_path)
        p = PoseStamped()
        p.header.frame_id = root_path[-1]
        p.pose = homo_matrix_to_pose(homo_m)
        # except Exception as e:
        #     traceback.print_exc()
        #     pass
        return p

    @profile
    def get_fk_np(self, root_path, tip_path):
        data = self.god_map.get_values(self._fks[root_path, tip_path].str_params)
        return self._fks[root_path, tip_path].call2(data)

    def get_attached_object_names(self):
        return [path[-1] for path in self.attached_objects]

    def init_fast_fks(self, *args, **kwargs):
        def f(key):
            root, tip = key
            fk = self.get_fk_expression(root, tip)
            m = w.speed_up(fk, w.free_symbols(fk))
            return m

        self._fks = KeyDefaultDict(f)

    def get_link_path(self, object_name, link_name):
        """
        :type object_name: str
        :type link_name: str
        :rtype: Path
        """
        path = Path(object_name) + ('links', link_name)
        if object_name == self._robot_name:
            if path in self.attached_objects:
                return self.attached_objects[path]
        return path

    def get_joint_path(self, object_name, link_name):
        return Path(object_name) + ('joints', link_name)

    def attach_existing_obj_to_robot(self, name, link):
        """
        :param name: name of the existing object
        :type name: name
        """
        object_ = self.get_object(name)
        object_root = object_.get_root()

        joint_path = self.get_joint_path(self._robot_name, name)
        parent_path = self.get_link_path(self._robot_name, link)
        child_path = self.get_link_path(name, object_root)

        casadi_pose = w.Matrix(self.get_fk_np(parent_path, child_path))

        joint_operation = ExecFunction(joint_path,
                                       FixedJoint,
                                       CPath(parent_path),
                                       CPath(child_path),
                                       casadi_pose)

        self.km_model.apply_operation('create {}'.format(joint_path), joint_operation)

        self.km_model.apply_operation('connect {} {}'.format(parent_path, child_path),
                                      CreateURDFFrameConnection(joint_path, parent_path, child_path))
        self.km_model.apply_operation('attach {} to {}'.format(child_path, parent_path),
                                      ExecFunction(Path([self._robot_name, 'attached_objects']),
                                                   lambda attached_objects, new_object: attached_objects.union(
                                                       {new_object}),
                                                   Path([self._robot_name, 'attached_objects']),
                                                   str(name)))
        self.reset_cache()
        self.get_object(name).base_pose = Pose(orientation=Quaternion(w=1))
        # self.km_model.clean_structure()
        # self.km_model.dispatch_events()
        self._objects_names.remove(name)
        self.attached_objects[self.get_link_path(self._robot_name, name)] = child_path

        self.robot.update_self_collision_matrix(added_links=set(product(self.robot.get_links_with_collision(),
                                                                        object_.get_links_with_collision())))
        logging.loginfo(u'--> attached object {} on link {}'.format(name, link))

    def detach(self, joint_name, from_obj=None):
        if joint_name not in self.robot.get_joint_names():
            raise UnknownBodyException(u'"{}" has not link "{}"'.format(self.robot.get_name(), joint_name))
        if from_obj is None:
            from_obj = self._robot_name
        elif from_obj != self._robot_name:
            raise UnsupportedOptionException(u'only detach from robot supported')
        o = self.get_object(from_obj)
        joint_path = self.get_joint_path(from_obj, joint_name)
        parent_path = o.get_parent_path_of_joint(joint_name)
        child_path = o.get_child_path_of_joint(joint_name)

        child_name = child_path[0]
        if child_name != self._robot_name:
            child_obj = self.get_object(child_name)
            o_in_w = self.get_fk_pose(Path(self.world_frame), child_obj.get_link_path(child_obj.get_root())).pose
        try:
            self.km_model.remove_operation('attach {} to {}'.format(child_path, parent_path))
            self.km_model.remove_operation('connect {} {}'.format(parent_path, child_path))
            self.km_model.remove_operation('create {}'.format(joint_path))
        except Exception as e:
            raise UnknownBodyException(u'can\'t detach: {}\n{}'.format(joint_name, e))

        self.reset_cache()
        if child_name != self._robot_name:
            o = self.get_object(from_obj)
            child_obj = self.get_object(child_name)
            child_obj.base_pose = o_in_w
            logging.loginfo(u'<-- detached {} from link {}'.format(joint_name, parent_path[-1]))
            o.update_self_collision_matrix(removed_links=[child_obj.get_link_names()])

        self._objects_names.append(str(child_path[:-2]))
        # fixme remove pr2 arm

    def get_robot_collision_matrix(self, min_dist):
        robot_name = self.robot.get_name()
        collision_matrix = self.robot.get_self_collision_matrix()
        collision_matrix2 = {}
        for link1, link2 in collision_matrix:
            # FIXME should I use the minimum of both distances?
            if self.robot.link_order(link1, link2):
                collision_matrix2[link1, robot_name, link2] = min_dist[link1]
            else:
                collision_matrix2[link2, robot_name, link1] = min_dist[link1]
        return collision_matrix2

    def collision_goals_to_collision_matrix(self, collision_goals, min_dist):
        """
        :param collision_goals: list of CollisionEntry
        :type collision_goals: list
        :return: dict mapping (robot_link, body_b, link_b) -> min allowed distance
        :rtype: dict
        """
        collision_goals = self.verify_collision_entries(collision_goals)
        min_allowed_distance = {}
        for collision_entry in collision_goals:  # type: CollisionEntry
            if self.is_avoid_all_self_collision(collision_entry):
                min_allowed_distance.update(self.get_robot_collision_matrix(min_dist))
                continue
            assert len(collision_entry.robot_links) == 1
            assert len(collision_entry.link_bs) == 1
            key = (collision_entry.robot_links[0], collision_entry.body_b, collision_entry.link_bs[0])
            r_key = (collision_entry.link_bs[0], collision_entry.body_b, collision_entry.robot_links[0])
            if self.is_allow_collision(collision_entry):
                if self.all_link_bs(collision_entry):
                    for key2 in list(min_allowed_distance.keys()):
                        if key[0] == key2[0] and key[1] == key2[1]:
                            del min_allowed_distance[key2]
                elif key in min_allowed_distance:
                    del min_allowed_distance[key]
                elif r_key in min_allowed_distance:
                    del min_allowed_distance[r_key]

            elif self.is_avoid_collision(collision_entry):
                min_allowed_distance[key] = min_dist[key[0]]
            else:
                raise Exception('todo')
        return min_allowed_distance

    def verify_collision_entries(self, collision_goals):
        for ce in collision_goals:  # type: CollisionEntry
            if ce.type in [CollisionEntry.ALLOW_ALL_COLLISIONS,
                           CollisionEntry.AVOID_ALL_COLLISIONS]:
                # logging.logwarn(u'ALLOW_ALL_COLLISIONS and AVOID_ALL_COLLISIONS deprecated, use AVOID_COLLISIONS and'
                #               u'ALLOW_COLLISIONS instead with ALL constant instead.')
                if ce.type == CollisionEntry.ALLOW_ALL_COLLISIONS:
                    ce.type = CollisionEntry.ALLOW_COLLISION
                else:
                    ce.type = CollisionEntry.AVOID_COLLISION

        for ce in collision_goals:  # type: CollisionEntry
            if CollisionEntry.ALL in ce.robot_links and len(ce.robot_links) != 1:
                raise PhysicsWorldException(u'ALL used in robot_links, but it\'s not the only entry')
            if CollisionEntry.ALL in ce.link_bs and len(ce.link_bs) != 1:
                raise PhysicsWorldException(u'ALL used in link_bs, but it\'s not the only entry')
            if ce.body_b == CollisionEntry.ALL and not self.all_link_bs(ce):
                raise PhysicsWorldException(u'if body_b == ALL, link_bs has to be ALL as well')

        self.are_entries_known(collision_goals)

        for ce in collision_goals:
            if not ce.robot_links:
                ce.robot_links = [CollisionEntry.ALL]
            if not ce.link_bs:
                ce.link_bs = [CollisionEntry.ALL]

        for i, ce in enumerate(reversed(collision_goals)):
            if self.is_avoid_all_collision(ce):
                collision_goals = collision_goals[len(collision_goals) - i - 1:]
                break
            if self.is_allow_all_collision(ce):
                collision_goals = collision_goals[len(collision_goals) - i:]
                break
        else:
            ce = CollisionEntry()
            ce.type = CollisionEntry.AVOID_COLLISION
            ce.robot_links = [CollisionEntry.ALL]
            ce.body_b = CollisionEntry.ALL
            ce.link_bs = [CollisionEntry.ALL]
            ce.min_dist = -1
            collision_goals.insert(0, ce)

        # split body bs
        collision_goals = self.split_body_b(collision_goals)

        # split robot links
        collision_goals = self.robot_related_stuff(collision_goals)

        # split link_bs
        collision_goals = self.split_link_bs(collision_goals)

        return collision_goals

    def are_entries_known(self, collision_goals):
        robot_name = self.robot.get_name()
        robot_links = set(self.robot.get_link_names())
        for collision_entry in collision_goals:
            if not (collision_entry.body_b == robot_name or
                    collision_entry.body_b in self.get_object_names() or
                    self.all_body_bs(collision_entry)):
                raise UnknownBodyException(u'body b \'{}\' unknown'.format(collision_entry.body_b))
            if not self.all_robot_links(collision_entry):
                for robot_link in collision_entry.robot_links:
                    if robot_link not in robot_links:
                        raise UnknownBodyException(u'robot link \'{}\' unknown'.format(robot_link))
            if collision_entry.body_b == robot_name:
                for robot_link in collision_entry.link_bs:
                    if robot_link != CollisionEntry.ALL and robot_link not in robot_links:
                        raise UnknownBodyException(
                            u'link b \'{}\' of body \'{}\' unknown'.format(robot_link, collision_entry.body_b))
            elif not self.all_body_bs(collision_entry) and not self.all_link_bs(collision_entry):
                object_links = self.get_object(collision_entry.body_b).get_link_names()
                for link_b in collision_entry.link_bs:
                    if link_b not in object_links:
                        raise UnknownBodyException(
                            u'link b \'{}\' of body \'{}\' unknown'.format(link_b, collision_entry.body_b))

    def split_link_bs(self, collision_goals):
        # FIXME remove the side effects of these three methods
        i = 0
        while i < len(collision_goals):
            collision_entry = collision_goals[i]
            if self.is_avoid_all_self_collision(collision_entry):
                i += 1
                continue
            if self.all_link_bs(collision_entry):
                if collision_entry.body_b == self.robot.get_name():
                    new_ces = []
                    link_bs = self.robot.get_possible_collisions(list(collision_entry.robot_links)[0])
                elif [x for x in collision_goals[i:] if
                      x.robot_links == collision_entry.robot_links and
                      x.body_b == collision_entry.body_b and not self.all_link_bs(x)]:
                    new_ces = []
                    link_bs = self.get_object(collision_entry.body_b).get_link_names_with_collision()
                else:
                    i += 1
                    continue
                collision_goals.remove(collision_entry)
                for link_b in link_bs:
                    ce = CollisionEntry()
                    ce.type = collision_entry.type
                    ce.robot_links = collision_entry.robot_links
                    ce.body_b = collision_entry.body_b
                    ce.min_dist = collision_entry.min_dist
                    ce.link_bs = [link_b]
                    new_ces.append(ce)
                for new_ce in new_ces:
                    collision_goals.insert(i, new_ce)
                i += len(new_ces)
                continue
            elif len(collision_entry.link_bs) > 1:
                collision_goals.remove(collision_entry)
                for link_b in collision_entry.link_bs:
                    ce = CollisionEntry()
                    ce.type = collision_entry.type
                    ce.robot_links = collision_entry.robot_links
                    ce.body_b = collision_entry.body_b
                    ce.link_bs = [link_b]
                    ce.min_dist = collision_entry.min_dist
                    collision_goals.insert(i, ce)
                i += len(collision_entry.link_bs)
                continue
            i += 1
        return collision_goals

    def robot_related_stuff(self, collision_goals):
        i = 0
        # FIXME detached objects get returned by get controlled links
        controlled_robot_links = self.robot.get_controlled_links()
        while i < len(collision_goals):
            collision_entry = collision_goals[i]
            if self.is_avoid_all_self_collision(collision_entry):
                i += 1
                continue
            if self.all_robot_links(collision_entry):
                collision_goals.remove(collision_entry)

                new_ces = []
                for robot_link in controlled_robot_links:
                    ce = CollisionEntry()
                    ce.type = collision_entry.type
                    ce.robot_links = [robot_link]
                    ce.body_b = collision_entry.body_b
                    ce.min_dist = collision_entry.min_dist
                    ce.link_bs = collision_entry.link_bs
                    new_ces.append(ce)

                for new_ce in new_ces:
                    collision_goals.insert(i, new_ce)
                i += len(new_ces)
                continue
            elif len(collision_entry.robot_links) > 1:
                collision_goals.remove(collision_entry)
                for robot_link in collision_entry.robot_links:
                    ce = CollisionEntry()
                    ce.type = collision_entry.type
                    ce.robot_links = [robot_link]
                    ce.body_b = collision_entry.body_b
                    ce.min_dist = collision_entry.min_dist
                    ce.link_bs = collision_entry.link_bs
                    collision_goals.insert(i, ce)
                i += len(collision_entry.robot_links)
                continue
            i += 1
        return collision_goals

    def split_body_b(self, collision_goals):
        i = 0
        while i < len(collision_goals):
            collision_entry = collision_goals[i]
            if self.all_body_bs(collision_entry):
                collision_goals.remove(collision_entry)
                new_ces = []
                for body_b in [self.robot.get_name()] + self.get_object_names():
                    ce = CollisionEntry()
                    ce.type = collision_entry.type
                    ce.robot_links = collision_entry.robot_links
                    ce.min_dist = collision_entry.min_dist
                    ce.body_b = body_b
                    ce.link_bs = collision_entry.link_bs
                    new_ces.append(ce)
                for new_ce in reversed(new_ces):
                    collision_goals.insert(i, new_ce)
                i += len(new_ces)
                continue
            i += 1
        return collision_goals

    def all_robot_links(self, collision_entry):
        return CollisionEntry.ALL in collision_entry.robot_links and len(collision_entry.robot_links) == 1

    def all_link_bs(self, collision_entry):
        return CollisionEntry.ALL in collision_entry.link_bs and len(collision_entry.link_bs) == 1 or \
               not collision_entry.link_bs

    def all_body_bs(self, collision_entry):
        return collision_entry.body_b == CollisionEntry.ALL

    def is_avoid_collision(self, collision_entry):
        return collision_entry.type in [CollisionEntry.AVOID_COLLISION, CollisionEntry.AVOID_ALL_COLLISIONS]

    def is_allow_collision(self, collision_entry):
        return collision_entry.type in [CollisionEntry.ALLOW_COLLISION, CollisionEntry.ALLOW_ALL_COLLISIONS]

    def is_avoid_all_self_collision(self, collision_entry):
        """
        :type collision_entry: CollisionEntry
        :return: bool
        """
        return self.is_avoid_collision(collision_entry) \
               and self.all_robot_links(collision_entry) \
               and collision_entry.body_b == self.robot.get_name() \
               and self.all_link_bs(collision_entry)

    def is_allow_all_self_collision(self, collision_entry):
        """
        :type collision_entry: CollisionEntry
        :return: bool
        """
        return self.is_allow_collision(collision_entry) \
               and self.all_robot_links(collision_entry) \
               and collision_entry.body_b == self.robot.get_name() \
               and self.all_link_bs(collision_entry)

    def is_avoid_all_collision(self, collision_entry):
        """
        :type collision_entry: CollisionEntry
        :return: bool
        """
        return self.is_avoid_collision(collision_entry) \
               and self.all_robot_links(collision_entry) \
               and self.all_body_bs(collision_entry) \
               and self.all_link_bs(collision_entry)

    def is_allow_all_collision(self, collision_entry):
        """
        :type collision_entry: CollisionEntry
        :return: bool
        """
        return self.is_allow_collision(collision_entry) \
               and self.all_robot_links(collision_entry) \
               and self.all_body_bs(collision_entry) \
               and self.all_link_bs(collision_entry)

    def reset_cache(self, *args, **kwargs):
        states = {}
        for key, value in self.km_model.data_tree.data_tree.items():
            if isinstance(value, Robot):
                states[key] = self.get_object(key).dump_state()

        self.km_model.clean_structure()
        self.km_model.dispatch_events()
        for method_name in dir(self):
            try:
                getattr(self, method_name).memo.clear()
            except:
                pass
        for object_name, state in states.items():
            if self.km_model.has_data(object_name):
                self.get_object(object_name).load_state(state)
