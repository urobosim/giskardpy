import itertools
from copy import deepcopy
import kineverse.gradients.gradient_math as gm
import urdf_parser_py.urdf as up
from betterpybullet import ClosestPair
from betterpybullet import CollisionObject
from betterpybullet import ContactPoint
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from giskard_msgs.msg import CollisionEntry, WorldBody
import betterpybullet as pb
from giskardpy import cas_wrapper as w
from giskardpy import identifier
from giskardpy import logging
from giskardpy.data_types import Collisions, Collision
from giskardpy.exceptions import RobotExistsException, DuplicateNameException, PhysicsWorldException, \
    UnknownBodyException, UnsupportedOptionException, CorruptShapeException
from giskardpy.input_system import PoseStampedInput
from giskardpy.robot import Robot
from giskardpy.urdf_object import URDFObject, hacky_urdf_parser_fix
from giskardpy.utils import KeyDefaultDict, memoize, homo_matrix_to_pose
from kineverse.model.geometry_model import GeometryModel
from kineverse.model.paths import Path
from kineverse.operations.basic_operations import ExecFunction
from kineverse.operations.urdf_operations import load_urdf, FixedJoint, CreateURDFFrameConnection
from kineverse.urdf_fix import urdf_filler


class World(object):
    world_frame = u'map'

    def __init__(self, god_map, prefix=tuple(), path_to_data_folder=u''):
        self.god_map = god_map
        self._fks = {}
        self.__prefix = '/'.join(prefix)
        self._objects_names = []
        self._robot_name = u'robot'
        self.km_model = GeometryModel()
        self.attached_objects = {}
        if path_to_data_folder is None:
            path_to_data_folder = u''
        self._path_to_data_folder = path_to_data_folder

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

    def getClosestPoints(self, body_a, body_b, distance, link_a, link_b=None):
        obj_a = self.pb_subworld.named_objects[str(self.get_link_path(body_a, link_a))]
        if link_b is None:
            link_bs = self.get_object(body_b).links.keys()
        else:
            link_bs = [link_b]
        result = []
        obj_bs = {}
        for link_b in link_bs:
            link_b_path = str(self.get_link_path(body_b, link_b))
            if link_b_path in self.pb_subworld.named_objects:
                obj_bs[self.pb_subworld.named_objects[link_b_path]] = link_b
        input = {obj_a: distance}
        query_result = self.pb_subworld.closest_distances(input)
        map_T_a = obj_a.transform
        for o, contacts in query_result.items():
            for contact in contacts:  # type: ClosestPair
                if contact.obj_b in obj_bs:
                    map_T_b = contact.obj_b.transform
                    for p in contact.points:  # type: ContactPoint
                        c = Collision(link_a, body_b, obj_bs[contact.obj_b], p.point_a, p.point_b, p.normal_world_b,
                                      p.distance)
                        c.set_position_on_a_in_map(map_T_a * p.point_a)
                        c.set_position_on_b_in_map(map_T_b * p.point_b)
                        c.set_contact_normal_in_b(map_T_b.inv() * p.normal_world_b)
                        result.append(c)
        return result

    def sync_bullet_world(self):
        symbols = self.pb_subworld.pose_generator.str_params
        data = dict(zip(symbols, self.god_map.get_values(symbols)))

        pb.batch_set_transforms(self.pb_subworld.collision_objects, self.pb_subworld.pose_generator(**data))
        self.pb_subworld._state.update(data)

    def check_collisions(self, cut_off_distances):
        self.sync_bullet_world()
        collisions = Collisions(self)
        robot_name = self.robot.get_name()
        for (robot_link, body_b, link_b), distance in cut_off_distances.items():
            # if robot_name == body_b:
            #     object_id = self.robot.get_pybullet_id()
            #     link_b_id = self.robot.get_pybullet_link_id(link_b)
            # else:
            #     object_id = self.__get_pybullet_object_id(body_b)
            #     if link_b != CollisionEntry.ALL:
            #         link_b_id = self.get_object(body_b).get_pybullet_link_id(link_b)

            # robot_link_id = self.robot.get_pybullet_link_id(robot_link)
            if body_b == robot_name or link_b != CollisionEntry.ALL:
                for collision in self.getClosestPoints(self._robot_name, body_b,
                                                       distance * 3,
                                                       robot_link, link_b):
                    collisions.add(collision)
            else:
                for collision in self.getClosestPoints(self._robot_name, body_b,
                                                       distance * 3,
                                                       robot_link):
                    collisions.add(collision)
            # if len(contacts) > 0:
            #     try:
            #         body_b_object = self.get_object(body_b)
            #     except KeyError:
            #         body_b_object = self.robot
            #     for contact in contacts:  # type: ContactInfo
            #         # if link_b == CollisionEntry.ALL:
            #         #     link_b = body_b_object.pybullet_link_id_to_name(contact.link_index_b)
            #         # if self.__should_flip_collision(contact.position_on_a, robot_link):
            #         #     flipped_normal = [-contact.contact_normal_on_b[0],
            #         #                       -contact.contact_normal_on_b[1],
            #         #                       -contact.contact_normal_on_b[2]]
            #         #     collision = Collision(robot_link, body_b, link_b,
            #         #                           contact.position_on_b, contact.position_on_a,
            #         #                           flipped_normal, contact.contact_distance)
            #         #     collisions.add(collision)
            #         # else:
            #             # collision = Collision(robot_link, body_b, link_b,
            #             #                       contact.position_on_a, contact.position_on_b,
            #             #                       contact.contact_normal_on_b, contact.contact_distance)
            #             collisions.add(collision)
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
        urdf_obj = urdf_filler(up.URDF.from_xml_string(hacky_urdf_parser_fix(urdf)))
        if name is None:
            name = urdf_obj.name

        if self.km_model.has_data(name):
            raise DuplicateNameException(u'Something with name \'{}\' already exists'.format(name))

        root_pose = PoseStampedInput(lambda path: Path(path).to_symbol(),
                                     translation_prefix=Path(
                                         [self.__prefix, u'km_model', u'data_tree', u'data_tree', name, u'base_pose',
                                          u'position']),
                                     rotation_prefix=Path(
                                         [self.__prefix, u'km_model', u'data_tree', u'data_tree', name, u'base_pose',
                                          u'orientation'])).get_frame()

        limit_map = load_urdf(ks=self.km_model,
                              prefix=Path(str(name)),
                              urdf=urdf_obj,
                              reference_frame=self.world_frame,
                              joint_prefix=Path(
                                  [self.__prefix, u'km_model', u'data_tree', u'data_tree', name, u'joint_state']),
                              limit_prefix=Path(
                                  [self.__prefix, u'km_model', u'data_tree', u'data_tree', name, u'limits']),
                              robot_class=Robot,
                              root_transform=root_pose,
                              name_override=name)
        obj = self.km_model.get_data(name)
        obj.init2(world=self, limit_map=limit_map, **kwargs)

        self.km_model.register_on_model_changed(Path(name), obj.reset_cache)
        self.km_model.register_on_model_changed(Path(name), self.init_fast_fks)
        self.reset_cache()
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
        :type name: str
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
        :type name: str
        :rtype: bool
        """
        return name in self.get_object_names()

    def set_object_joint_state(self, name, joint_state):
        """
        :type name: str
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
        for object_name in self._objects_names:
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

    def get_fk_np(self, root_path, tip_path):
        world_joint_state = self.robot.get_joint_state_positions()
        world_joint_state = {
            str(Path(identifier.joint_states + [joint_name, u'position']).to_symbol()): world_joint_state[joint_name]
            for joint_name in world_joint_state}
        data_tree_path = [self.__prefix, u'km_model', u'data_tree', u'data_tree']
        object_path = data_tree_path + [self._robot_name]
        base_position_path = object_path + [u'base_pose', u'position']
        base_orientation_path = object_path + [u'base_pose', u'orientation']
        base_pose = {str(Path(base_position_path + [u'x']).to_symbol()): self.robot.base_pose.position.x,
                     str(Path(base_position_path + [u'y']).to_symbol()): self.robot.base_pose.position.y,
                     str(Path(base_position_path + [u'z']).to_symbol()): self.robot.base_pose.position.z,
                     str(Path(base_orientation_path + [u'x']).to_symbol()): self.robot.base_pose.orientation.x,
                     str(Path(base_orientation_path + [u'y']).to_symbol()): self.robot.base_pose.orientation.y,
                     str(Path(base_orientation_path + [u'z']).to_symbol()): self.robot.base_pose.orientation.z,
                     str(Path(base_orientation_path + [u'w']).to_symbol()): self.robot.base_pose.orientation.w}
        world_joint_state.update(base_pose)
        for object_name in itertools.chain(self.get_object_names(), self.get_attached_object_names()):
            object_joint_state = self.get_object(object_name).get_joint_state_positions()
            object_joint_state = {
                str(Path(identifier.world + [object_name, joint_name, u'position']).to_symbol()): object_joint_state[
                    joint_name] for joint_name in object_joint_state}
            object_path = data_tree_path + [object_name]
            base_position_path = object_path + [u'base_pose', u'position']
            base_orientation_path = object_path + [u'base_pose', u'orientation']
            base_pose = {
                str(Path(base_position_path + [u'x']).to_symbol()): self.get_object(object_name).base_pose.position.x,
                str(Path(base_position_path + [u'y']).to_symbol()): self.get_object(object_name).base_pose.position.y,
                str(Path(base_position_path + [u'z']).to_symbol()): self.get_object(object_name).base_pose.position.z,
                str(Path(base_orientation_path + [u'x']).to_symbol()): self.get_object(
                    object_name).base_pose.orientation.x,
                str(Path(base_orientation_path + [u'y']).to_symbol()): self.get_object(
                    object_name).base_pose.orientation.y,
                str(Path(base_orientation_path + [u'z']).to_symbol()): self.get_object(
                    object_name).base_pose.orientation.z,
                str(Path(base_orientation_path + [u'w']).to_symbol()): self.get_object(
                    object_name).base_pose.orientation.w}
            world_joint_state.update(base_pose)
            world_joint_state.update(object_joint_state)
        return self._fks[root_path, tip_path](**world_joint_state)

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
        object_root = self.get_object(name).get_root()

        joint_path = self.get_joint_path(self._robot_name, name)
        parent_path = self.get_link_path(self._robot_name, link)
        child_path = self.get_link_path(name, object_root)

        casadi_pose = w.Matrix(self.get_fk_np(parent_path, child_path))

        joint_operation = ExecFunction(joint_path,
                                       FixedJoint,
                                       str(parent_path),
                                       str(child_path),
                                       casadi_pose)

        self.km_model.apply_operation('create {}'.format(joint_path), joint_operation)

        self.km_model.apply_operation('connect {} {}'.format(parent_path, child_path),
                                      CreateURDFFrameConnection(joint_path, parent_path, child_path))
        self.km_model.apply_operation('attach {} to {}'.format(child_path, parent_path),
                                      ExecFunction(Path([self._robot_name, 'attached_objects']),
                                                   lambda attached_objects, new_object: attached_objects.union({new_object}),
                                                   Path([self._robot_name, 'attached_objects']),
                                                   str(name)))
        self.reset_cache()
        self.get_object(name).base_pose = Pose(orientation=Quaternion(w=1))
        # self.km_model.clean_structure()
        # self.km_model.dispatch_events()
        self._objects_names.remove(name)
        self.attached_objects[self.get_link_path(self._robot_name, name)] = child_path

        logging.loginfo(u'--> attached object {} on link {}'.format(name, link))

    def detach(self, joint_name, from_obj=None):
        if from_obj is None:
            from_obj = self._robot_name
        elif from_obj != self._robot_name:
            raise UnsupportedOptionException(u'only detach from robot supported')
        o = self.get_object(from_obj)
        joint_path = self.get_joint_path(from_obj, joint_name)
        parent_path = o.get_parent_path_of_joint(joint_name)
        child_path = o.get_child_path_of_joint(joint_name)
        try:
            self.km_model.remove_operation('attach {} to {}'.format(child_path, parent_path))
            self.km_model.remove_operation('connect {} {}'.format(parent_path, child_path))
            self.km_model.remove_operation('create {}'.format(joint_path))
        except Exception as e:
            raise UnknownBodyException(u'can\'t detach: {}\n{}'.format(joint_name, e))

        self.reset_cache()
        # self.km_model.clean_structure()
        # self.km_model.dispatch_events()

        try:
            del self.attached_objects[self.get_link_path(from_obj, o.get_name())]
        except KeyError:
            pass

        self._objects_names.append(str(child_path[:-2]))
        # fixme remove pr2 arm

        # if from_obj is None or self.robot.get_name() == from_obj:
        # this only works because attached simple objects have joint names equal to their name
        # p = self.robot.get_fk_pose(self.robot.get_root(), joint_name)
        # p_map = kdl_to_pose(self.robot.root_T_map.Inverse() * msg_to_kdl(p))
        #
        # parent_link = self.robot.get_parent_link_of_joint(joint_name)
        # cut_off_obj = self.robot.detach_sub_tree(joint_name)
        # logging.loginfo(u'<-- detached {} from link {}'.format(joint_name, parent_link))
        # else:

        # wo = WorldObject.from_urdf_object(cut_off_obj)  # type: WorldObject
        # wo.base_pose = p_map
        # self.add_object(wo)

    def get_robot_collision_matrix(self, min_dist):
        robot_name = self.robot.get_name()
        collision_matrix = self.robot.get_self_collision_matrix()
        collision_matrix2 = {}
        for link1, link2 in collision_matrix:
            # FIXME should I use the minimum of both distances?
            if self.robot.link_order(link1, link2):
                collision_matrix2[link1, robot_name, link2] = min_dist[link1][u'zero_weight_distance']
            else:
                collision_matrix2[link2, robot_name, link1] = min_dist[link1][u'zero_weight_distance']
        return collision_matrix2

    def collision_goals_to_collision_matrix(self, collision_goals, min_dist):
        """
        :param collision_goals: list of CollisionEntry
        :type collision_goals: list
        :return: dict mapping (robot_link, body_b, link_b) -> min allowed distance
        :rtype: dict
        """
        collision_goals = self.verify_collision_entries(collision_goals, min_dist)
        min_allowed_distance = {}
        for collision_entry in collision_goals:  # type: CollisionEntry
            if self.is_avoid_all_self_collision(collision_entry):
                min_allowed_distance.update(self.get_robot_collision_matrix(min_dist))
                continue
            assert len(collision_entry.robot_links) == 1
            assert len(collision_entry.link_bs) == 1
            key = (collision_entry.robot_links[0], collision_entry.body_b, collision_entry.link_bs[0])
            if self.is_allow_collision(collision_entry):
                if self.all_link_bs(collision_entry):
                    for key2 in list(min_allowed_distance.keys()):
                        if key[0] == key2[0] and key[1] == key2[1]:
                            del min_allowed_distance[key2]
                elif key in min_allowed_distance:
                    del min_allowed_distance[key]

            elif self.is_avoid_collision(collision_entry):
                min_allowed_distance[key] = min_dist[key[0]][u'zero_weight_distance']
            else:
                raise Exception('todo')
        return min_allowed_distance

    def verify_collision_entries(self, collision_goals, min_dist):
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

    def get_controlled_robot_links(self):
        return self.get_controlled_object_links(self._robot_name)

    def get_controlled_object_links(self, object_name):
        obj = self.get_object(object_name)
        symbols = set()
        for link in obj.links.values():
            symbols |= gm.free_symbols(link.to_parent)
        links = set()
        for link_path in self.km_model.get_active_geometry_raw(symbols):
            links.add(Path(link_path)[-1])
        return links

    def robot_related_stuff(self, collision_goals):
        i = 0
        controlled_robot_links = self.get_controlled_robot_links()
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

    def is_avoid_all_collision(self, collision_entry):
        """
        :type collision_entry: CollisionEntry
        :return: bool
        """
        return self.is_avoid_collision(collision_entry) \
               and self.all_robot_links(collision_entry) \
               and self.all_body_bs(collision_entry) \
               and self.all_link_bs(collision_entry)

    def is_avoid_all_self_collision(self, collision_entry):
        """
        :type collision_entry: CollisionEntry
        :return: bool
        """
        return self.is_avoid_collision(collision_entry) \
               and self.all_robot_links(collision_entry) \
               and collision_entry.body_b == self.robot.get_name() \
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
