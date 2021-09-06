from __future__ import division

from copy import deepcopy

from geometry_msgs.msg import Pose, Quaternion, Point
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

import kineverse.gradients.gradient_math as gm
from giskardpy.model.utils import cube_volume, cube_surface, sphere_volume, cylinder_volume
from giskardpy.utils.tfwrapper import normalize
from giskardpy.utils.utils import memoize
from kineverse.model.geometry_model import ArticulatedObject, GEOM_TYPE_MESH, GEOM_TYPE_BOX, GEOM_TYPE_CYLINDER, \
    GEOM_TYPE_SPHERE
from kineverse.model.paths import Path


def robot_name_from_urdf_string(urdf_string):
    return urdf_string.split('robot name="')[1].split('"')[0]


def hacky_urdf_parser_fix(urdf_str):
    # TODO this function is inefficient but the tested urdfs's aren't big enough for it to be a problem
    fixed_urdf = ''
    delete = False
    black_list = ['transmission', 'gazebo']
    black_open = ['<{}'.format(x) for x in black_list]
    black_close = ['</{}'.format(x) for x in black_list]
    for line in urdf_str.split('\n'):
        if len([x for x in black_open if x in line]) > 0:
            delete = True
        if len([x for x in black_close if x in line]) > 0:
            delete = False
            continue
        if not delete:
            fixed_urdf += line + '\n'
    return fixed_urdf


FIXED_JOINT = u'fixed'
REVOLUTE_JOINT = u'revolute'
CONTINUOUS_JOINT = u'continuous'
PRISMATIC_JOINT = u'prismatic'
JOINT_TYPES = [FIXED_JOINT, REVOLUTE_JOINT, CONTINUOUS_JOINT, PRISMATIC_JOINT]
MOVABLE_JOINT_TYPES = [REVOLUTE_JOINT, CONTINUOUS_JOINT, PRISMATIC_JOINT]
ROTATIONAL_JOINT_TYPES = [REVOLUTE_JOINT, CONTINUOUS_JOINT]
TRANSLATIONAL_JOINT_TYPES = [PRISMATIC_JOINT]
LIMITED_JOINTS = [PRISMATIC_JOINT, REVOLUTE_JOINT]


class URDFObject(ArticulatedObject):

    @profile
    def __init__(self, name):
        super(URDFObject, self).__init__(name)
        self.attached_objects = set()

    def init2(self, world=None, limit_map=None, *args, **kwargs):
        """
        :type world: giskardpy.world.World
        :param limit_map: maps joint name to dict, which maps derivative to dict, which maps lower/upper to number
        :type limit_map: dict
        :type args: list
        :type kwargs: dict
        :return:
        """
        self._world = world
        self._link_to_marker = {}
        if limit_map is not None:
            self._limits = limit_map
        else:
            self._limits = {}

    def update_joint_limits(self, new_linear_limits, new_angular_limits):
        # TODO do minimum with old values?
        for joint_name in self.joint_limits:
            for order in range(1, len(new_linear_limits)+1):
                if self.is_joint_prismatic(joint_name):
                    try:
                        self._limits[joint_name][order] = min(new_linear_limits[order][joint_name],
                                                              self._limits[joint_name][order])
                    except KeyError:
                        self._limits[joint_name][order] = new_linear_limits[order][joint_name]

                elif self.is_joint_rotational(joint_name):
                    try:
                        self._limits[joint_name][order] = min(new_angular_limits[order][joint_name],
                                                              self._limits[joint_name][order])
                    except KeyError:
                        self._limits[joint_name][order] = new_angular_limits[order][joint_name]

    @property
    def joint_limits(self):
        return self._limits

    def reset_cache(self, *args, **kwargs):
        for method_name in dir(self):
            try:
                getattr(self, method_name).memo.clear()
            except:
                pass

    @memoize
    def get_name(self):
        """
        :rtype: str
        """
        return self.name

    # JOINT FUNCTIONS

    @memoize
    def get_joint_names(self):
        """
        :rtype: list
        """
        return sum([self._world.get_object(object_name).get_joint_names() for object_name in self.attached_objects],
                   [joint_name for joint_name in self.joints.keys()])

    @memoize
    def get_split_chain(self, root, tip, joints=True, links=True, fixed=True):
        if root == tip:
            return [], [], []
        root_chain = self.get_simple_chain(self.get_root(), root, False, True, True)
        tip_chain = self.get_simple_chain(self.get_root(), tip, False, True, True)
        for i in range(min(len(root_chain), len(tip_chain))):
            if root_chain[i] != tip_chain[i]:
                break
        else:
            i += 1
        connection = tip_chain[i - 1]
        root_chain = self.get_simple_chain(connection, root, joints, links, fixed)
        if links:
            root_chain = root_chain[1:]
        root_chain.reverse()
        tip_chain = self.get_simple_chain(connection, tip, joints, links, fixed)
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
        if root == tip:
            if links:
                return [root]
            else:
                return []
        root_chain, connection, tip_chain = self.get_split_chain(root, tip, joints, links, fixed)
        return root_chain + connection + tip_chain

    @memoize
    def get_simple_chain(self, root, tip, joints=True, links=True, fixed=True):
        """
        :type root: str
        :type tip: str
        :type joints: bool
        :type links: bool
        :type fixed: bool
        :rtype: list
        """
        chain = []
        if links:
            chain.append(tip)
        link = tip
        while link != root:
            joint = self.get_parent_joint_of_link(link)
            parent = self.get_parent_link_of_link(link)
            if joints:
                # parent_joint = self.get_parent_joint_of_joint(joint)
                if fixed or not self.is_joint_fixed(joint):
                    chain.append(joint)
            if links:
                chain.append(parent)
            link = parent
        chain.reverse()
        return chain

    @memoize
    def get_connecting_link(self, link1, link2):
        return self.get_split_chain(link1, link2, joints=False)[1][0]

    @memoize
    def get_joint_names_from_chain(self, root_link, tip_link):
        """
        :rtype root: str
        :rtype tip: str
        :rtype: list
        """
        return self.get_chain(root_link, tip_link, True, False, True)

    @memoize
    def get_joint_names_from_chain_controllable(self, root_link, tip_link):
        """
        :rtype root: str
        :rtype tip: str
        :rtype: list
        """
        return self.get_chain(root_link, tip_link, True, False, False)

    @memoize
    def get_joint_names_controllable(self):
        """
        :return: returns the names of all movable joints which are not mimic.
        :rtype: list
        """
        return [joint_name for joint_name in self.get_joint_names() if self.is_joint_controllable(joint_name)]

    @memoize
    def get_all_joint_limits(self):
        """
        :return: dict mapping joint names to tuple containing lower and upper limits
        :rtype: dict
        """
        return {joint_name: self.get_joint_position_limits(joint_name) for joint_name in self.get_joint_names()
                if self.is_joint_controllable(joint_name)}

    @memoize
    def get_joint_position_limits(self, joint_name):
        """
        Returns joint limits specified in the safety controller entry if given, else returns the normal limits.
        :param joint_name: name of the joint in the urdfs
        :type joint_name: str
        :return: lower limit, upper limit or None if not applicable
        :rtype: float, float
        """
        # joint = self.get_joint(joint_name)
        try:
            return (self._limits[joint_name][0][u'lower'], self._limits[joint_name][0][u'upper'])
        except KeyError:
            return (None, None)

    @memoize
    def get_joint_velocity_limit(self, joint_name):
        return self._limits[joint_name][1]

    @memoize
    def get_joint_axis(self, joint_name):
        joint = self.get_joint(joint_name)
        return joint.axis

    @memoize
    def is_joint_controllable(self, name):
        """
        :param name: name of the joint in the urdfs
        :type name: str
        :return: True if joint type is revolute, continuous or prismatic
        :rtype: bool
        """
        joint = self.get_joint(name)
        return joint.type in MOVABLE_JOINT_TYPES and not self.is_joint_mimic(name)

    @memoize
    def is_joint_mimic(self, name):
        """
        :param name: name of the joint in the urdfs
        :type name: str
        :rtype: bool
        """
        joint = self.get_joint(name)
        expression = joint.position
        if joint.type not in MOVABLE_JOINT_TYPES:
            return False
        if gm.is_symbol(expression):
            return not name in str(expression)
        return True
        # return joint.type in MOVABLE_JOINT_TYPES and joint.mimic is not None

    @memoize
    def get_mimiced_joint_name(self, joint_name):
        return self.get_joint(joint_name).mimic.joint

    @memoize
    def get_mimic_multiplier(self, joint_name):
        multiplier = self.get_joint(joint_name).mimic.multiplier
        if multiplier is None:
            return 1
        return multiplier

    @memoize
    def get_mimic_offset(self, joint_name):
        offset = self.get_joint(joint_name).mimic.offset
        if offset is None:
            return 0
        return offset

    @memoize
    def has_joint(self, name):
        return name in self.get_joint_names()

    @memoize
    def has_link(self, name):
        return name in self.get_link_names()

    @memoize
    def is_joint_continuous(self, name):
        """
        :param name: name of the joint in the urdfs
        :type name: str
        :rtype: bool
        """
        return self.get_joint_type(name) == CONTINUOUS_JOINT

    @memoize
    def is_joint_revolute(self, name):
        """
        :param name: name of the joint in the urdfs
        :type name: str
        :rtype: bool
        """
        return self.get_joint_type(name) == REVOLUTE_JOINT

    @memoize
    def is_joint_fixed(self, name):
        """
        :param name: name of the joint in the urdfs
        :type name: str
        :rtype: bool
        """
        return self.get_joint_type(name) == FIXED_JOINT

    @memoize
    def get_joint_type(self, name):
        return self.get_joint(name).type

    @memoize
    def is_joint_type_supported(self, name):
        return self.get_joint_type(name) in JOINT_TYPES

    @memoize
    def is_joint_rotational(self, name):
        return self.get_joint_type(name) in ROTATIONAL_JOINT_TYPES

    @memoize
    def is_joint_prismatic(self, name):
        return self.get_joint_type(name) == PRISMATIC_JOINT

    # LINK FUNCTIONS

    @memoize
    def get_link_names(self):
        """
        :rtype: dict
        """
        return sum([self._world.get_object(object_name).get_link_names() for object_name in self.attached_objects],
                   [link_name for link_name in self.links.keys()])

    @memoize
    def get_sub_tree_link_names_with_collision(self, root_joint):
        """
        returns a set of links with
        :type: str
        :param volume_threshold: links with simple geometric shape and less volume than this will be ignored
        :type volume_threshold: float
        :param surface_treshold:
        :type surface_treshold: float
        :return: all links connected to root
        :rtype: list
        """
        sub_tree = self.get_links_from_sub_tree(root_joint)
        return [link_name for link_name in sub_tree if self.has_link_collision(link_name)]

    @memoize
    def get_link_names_with_collision(self):
        return [link_name for link_name in self.get_link_names() if self.has_link_collision(link_name)]

    @memoize
    def get_links_from_sub_tree(self, joint_name):
        return self.get_sub_tree_at_joint(joint_name).get_link_names()

    @memoize
    def get_links_with_collision(self):
        return [x for x in self.get_link_names() if self.has_link_collision(x)]

    @memoize
    def get_sub_tree_at_joint(self, joint_name):
        """
        :type joint_name: str
        :rtype: URDFObject
        """
        tree_links = []
        tree_joints = []
        joints = [joint_name]
        for joint in joints:
            child_link = self.get_child_link_of_joint(joint)
            for j in self.get_child_joints_of_link(child_link):
                joints.append(j)
                tree_joints.append(self.get_joint(j))
            if len(self.get_child_links_of_link(child_link)) > 0:
                for j, l in self._urdf_robot.child_map[child_link]:
                    joints.append(j)
                    tree_joints.append(self.get_joint(j))
            tree_links.append(self.get_link(child_link))

        return URDFObject.from_parts(joint_name, tree_links, tree_joints)

    @memoize
    def get_joint(self, joint_name):
        try:
            return self.joints[joint_name]
        except KeyError:
            return self.check_attachments(self.get_joint,
                                          u'joint "{}" not present in "{}"'.format(joint_name, self.get_name()),
                                          joint_name=joint_name)

    @memoize
    def get_link(self, link_name):
        """
        :param link_name:
        :return:
        :rtype: kineverse.model.geometry_model.RigidBody
        """
        try:
            return self.links[link_name]
        except KeyError:
            return self.check_attachments(self.get_link,
                                          u'link "{}" not present in "{}"'.format(link_name, self.get_name()),
                                          link_name=link_name)

    def split_at_link(self, link_name):
        pass

    @memoize
    def has_link_collision(self, link_name, volume_threshold=1.001e-6, surface_threshold=0.00061):
        """
        :type link: str
        :param volume_threshold: m**3, ignores simple geometry shapes with a volume less than this
        :type volume_threshold: float
        :param surface_threshold: m**2, ignores simple geometry shapes with a surface area less than this
        :type surface_threshold: float
        :return: True if collision geometry is mesh or simple shape with volume/surface bigger than thresholds.
        :rtype: bool
        """
        link = self.get_link(link_name)
        if link.collision is not None:
            geo = link.collision
            if geo[0].type == u'box':
                return cube_volume(geo[0].scale[0],
                                   geo[0].scale[1],
                                   geo[0].scale[2]) > volume_threshold or \
                       cube_surface(geo[0].scale[0],
                                    geo[0].scale[1],
                                    geo[0].scale[2]) > surface_threshold
            if geo[0].type == u'sphere':
                return sphere_volume(geo[0].scale[0] / 2) > volume_threshold or \
                       sphere_volume(geo[0].scale[0] / 2) > surface_threshold
            if geo[0].type == u'cylinder':
                return cylinder_volume(geo[0].scale[0] / 2,
                                       geo[0].scale[2]) > volume_threshold or \
                       cylinder_volume(geo[0].scale[0] / 2,
                                       geo[0].scale[2]) > surface_threshold
            if geo[0].type == u'mesh':
                return True
        return False

    @memoize
    def get_root(self):
        for link_name in self.get_link_names():
            try:
                parent = self.get_parent_link_of_link(link_name)
            except KeyError:
                return link_name
            if parent not in self.get_link_names():
                # TODO is this dead code?
                return link_name

    @memoize
    def get_first_child_links_with_collision(self, link_name):
        while not self.has_link_collision(link_name):
            children = self.get_child_links_of_link(link_name)
            children_with_collision = [x for x in children if self.has_link_collision(x)]
            if len(children_with_collision) > 1 or len(children) > 1:
                raise TypeError(u'first collision link is not unique')
            elif len(children_with_collision) == 1:
                link_name = children_with_collision[0]
                break
            else:
                link_name = children[0]
        return link_name

    @memoize
    def get_non_base_movement_root(self):
        l = self.get_root()
        result = self.__get_non_base_movement_root_helper(l)
        if result is None:
            result = l
        return result

    def __get_non_base_movement_root_helper(self, link_name):
        if self.has_link_collision(link_name):
            parent_joint = self.get_parent_joint_of_link(link_name)
            if self.is_joint_controllable(parent_joint):
                return link_name
            else:
                return None
        else:
            for child in self.get_child_links_of_link(link_name):
                child_result = self.__get_non_base_movement_root_helper(child)
                if child_result is None:
                    parent_joint = self.get_parent_joint_of_link(link_name)
                    if parent_joint is not None and self.is_joint_controllable(parent_joint):
                        return link_name
                    else:
                        return None
                return child_result

    # def attach_urdf_object(self, urdf_object, parent_link, pose, round_to=3):
    #     """
    #     Rigidly attach another object to the robot.
    #     :param urdf_object: Object that shall be attached to the robot.
    #     :type urdf_object: URDFObject
    #     :param parent_link_name: Name of the link to which the object shall be attached.
    #     :type parent_link_name: str
    #     :param pose: Hom. transform between the reference frames of the parent link and the object.
    #     :type pose: Pose
    #     """
    #     if urdf_object.get_name() in self.get_link_names():
    #         raise DuplicateNameException(
    #             u'\'{}\' already has link with name \'{}\'.'.format(self.get_name(), urdf_object.get_name()))
    #     if urdf_object.get_name() in self.get_joint_names():
    #         raise DuplicateNameException(
    #             u'\'{}\' already has joint with name \'{}\'.'.format(self.get_name(), urdf_object.get_name()))
    #     if parent_link not in self.get_link_names():
    #         raise UnknownBodyException(
    #             u'can not attach \'{}\' to non existent parent link \'{}\' of \'{}\''.format(urdf_object.get_name(),
    #                                                                                          parent_link,
    #                                                                                          self.get_name()))
    #     if len(set(urdf_object.get_link_names()).intersection(set(self.get_link_names()))) != 0:
    #         raise DuplicateNameException(u'can not merge urdfs that share link names')
    #     if len(set(urdf_object.get_joint_names()).intersection(set(self.get_joint_names()))) != 0:
    #         raise DuplicateNameException(u'can not merge urdfs that share joint names')
    #
    #     origin = up.Pose([np.round(pose.position.x, round_to),
    #                       np.round(pose.position.y, round_to),
    #                       np.round(pose.position.z, round_to)],
    #                      euler_from_quaternion([np.round(pose.orientation.x, round_to),
    #                                             np.round(pose.orientation.y, round_to),
    #                                             np.round(pose.orientation.z, round_to),
    #                                             np.round(pose.orientation.w, round_to)]))
    #
    #     joint = up.Joint(self.robot_name_to_root_joint(urdf_object.get_name()),
    #                      parent=parent_link,
    #                      child=urdf_object.get_root(),
    #                      joint_type=FIXED_JOINT,
    #                      origin=origin)
    #     self._urdf_robot.add_joint(joint)
    #     for j in urdf_object._urdf_robot.joints:
    #         self._urdf_robot.add_joint(j)
    #     for l in urdf_object._urdf_robot.links:
    #         self._urdf_robot.add_link(l)
    #     try:
    #         del self._link_to_marker[urdf_object.get_name()]
    #     except:
    #         pass
    #     self.reinitialize()

    @memoize
    def get_joint_origin(self, joint_name):
        origin = self.get_joint(joint_name).origin
        p = Pose()
        p.position.x = origin.xyz[0]
        p.position.y = origin.xyz[1]
        p.position.z = origin.xyz[2]
        p.orientation = Quaternion(*quaternion_from_euler(*origin.rpy))
        return p

    # def detach_sub_tree(self, joint_name):
    #     """
    #     :rtype: URDFObject
    #     """
    #     try:
    #         sub_tree = self.get_sub_tree_at_joint(joint_name)
    #     except KeyError:
    #         raise KeyError(u'can\'t detach at unknown joint: {}'.format(joint_name))
    #     for link in sub_tree.get_link_names():
    #         self._urdf_robot.remove_aggregate(self.get_link(link))
    #     for joint in chain([joint_name], sub_tree.get_joint_names()):
    #         self._urdf_robot.remove_aggregate(self.get_joint(joint))
    #     self.reinitialize()
    #     return sub_tree

    # def reset(self):
    #     """
    #     Detaches all object that have been attached to the robot.
    #     """
    #     self._urdf_robot = up.URDF.from_xml_string(self.original_urdf)
    #     self.reinitialize()

    # def __str__(self):
    #     return self.get_urdf_str()

    def __hash__(self):
        return hash(id(self))

    # def reinitialize(self):
    #     self._urdf_robot = up.URDF.from_xml_string(self.get_urdf_str())
    #     self.reset_cache()

    def robot_name_to_root_joint(self, name):
        # TODO should this really be a class function?
        return u'{}'.format(name)

    @memoize
    def get_parent_link_of_link(self, link_name):
        link = self.get_link(link_name)
        parent = Path(link.parent)[-1]
        if parent in self.links:
            return parent

    @memoize
    def get_movable_parent_joint(self, link_name):
        joint = self.get_parent_joint_of_link(link_name)
        while not self.is_joint_controllable(joint):
            joint = self.get_parent_joint_of_joint(joint)
        return joint

    @memoize
    def get_child_links_of_link(self, link_name):
        child_links = []
        for link_name_ in self.get_link_names():
            parent_link = self.get_parent_link_of_link(link_name_)
            if parent_link == link_name:
                child_links.append(link_name_)
        return child_links

    @memoize
    def get_parent_joint_of_link(self, link_name):
        return Path(self.get_link(link_name).parent_joint)[-1]

    @memoize
    def get_parent_joint_of_joint(self, joint_name):
        return self.get_parent_joint_of_link(self.get_parent_link_of_joint(joint_name))

    @memoize
    def get_child_joints_of_link(self, link_name):
        child_joints = []
        for joint_name in self.get_joint_names():
            parent_link = self.get_parent_link_of_joint(joint_name)
            if parent_link == link_name:
                child_joints.append(joint_name)
        return child_joints

    @memoize
    def get_parent_link_of_joint(self, joint_name):
        return self.get_parent_path_of_joint(joint_name)[-1]

    def get_parent_path_of_joint(self, joint_name):
        joint = self.get_joint(joint_name)
        return Path(joint.parent)

    @memoize
    def get_child_link_of_joint(self, joint_name):
        return self.get_child_path_of_joint(joint_name)[-1]

    @memoize
    def get_child_path_of_joint(self, joint_name):
        joint = self.get_joint(joint_name)
        return Path(joint.child)

    def check_attachments(self, f, error_msg, **kwargs):
        for object_name in self.attached_objects:
            try:
                obj = self._world.get_object(object_name)
                try:
                    return getattr(obj, f.im_func.func_name)(**kwargs)
                except AttributeError:
                    return getattr(obj, f.__name__)(**kwargs)
            except KeyError:
                pass
        raise KeyError(error_msg)

    @memoize
    def are_linked(self, link_a, link_b):
        return link_a == self.get_parent_link_of_link(link_b) or \
               (link_b == self.get_parent_link_of_link(link_a))

    @memoize
    def get_controllable_joints(self):
        # TODO test me pls
        return [joint_name for joint_name in self.get_joint_names() if self.is_joint_controllable(joint_name)]

    @memoize
    def has_link_visuals(self, link_name):
        visual = self.get_link(link_name).geometry
        return visual is not None

    def get_leaves(self):
        leaves = []
        for link_name in self.get_link_names():
            if not self.get_child_links_of_link(link_name):
                leaves.append(link_name)
        return leaves

    def as_marker_msg(self, ns=u'', id=1):
        """
        :param ns:
        :param id:
        :rtype: Marker
        """
        if len(self.get_link_names()) > 1:
            raise TypeError(u'only urdfs objects with a single link can be turned into marker')
        marker = Marker()
        marker.ns = u'{}/{}'.format(ns, self.get_name())
        marker.id = id
        geometry = list(self.links[self.get_link_names()[0]].geometry.values())[0]
        if geometry.type == GEOM_TYPE_MESH:
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = geometry.mesh
            if geometry.scale is None:
                marker.scale.x = 1.0
                marker.scale.z = 1.0
                marker.scale.y = 1.0
            else:
                marker.scale.x = geometry.scale[0]
                marker.scale.z = geometry.scale[1]
                marker.scale.y = geometry.scale[2]
            marker.mesh_use_embedded_materials = True
        elif geometry.type == GEOM_TYPE_BOX:
            marker.type = Marker.CUBE
            marker.scale.x = geometry.scale[0]
            marker.scale.y = geometry.scale[1]
            marker.scale.z = geometry.scale[2]
        elif geometry.type == GEOM_TYPE_CYLINDER:
            marker.type = Marker.CYLINDER
            marker.scale.x = geometry.scale[0]
            marker.scale.y = geometry.scale[0]
            marker.scale.z = geometry.scale[2]
        elif geometry.type == GEOM_TYPE_SPHERE:
            marker.type = Marker.SPHERE
            marker.scale.x = geometry.scale[0]
            marker.scale.y = geometry.scale[0]
            marker.scale.z = geometry.scale[0]
        else:
            # TODO implement mesh
            raise Exception(u'world body type {} can\'t be converted to marker'.format(geometry.__class__.__name__))
        marker.color = ColorRGBA(0, 1, 0, 0.5)
        return marker

    def has_non_identity_visual_offset(self, link_name):
        """
        Returns true, if the link with the given link name has a visual origin/offset,
        which equals not the identity pose. Otherwise, false.
        """
        if self.has_link_visuals(link_name) and self.get_urdf_link(link_name).visual.origin:
            visual_offset = self.get_urdf_link(link_name).visual.origin
            return visual_offset.rpy != [0, 0, 0] or visual_offset.xyz != [0, 0, 0]
        return False

    def get_visual_pose(self, link_name):
        """
        Returns the origin/offset of the visual in the link object with the given link name.
        If no visual origin was found, None is returned.
        """
        if self.has_link_visuals(link_name) and self.get_urdf_link(link_name).visual.origin:
            visual_offset = self.get_urdf_link(link_name).visual.origin
            position = Point(*visual_offset.xyz)
            orientation = normalize(Quaternion(*quaternion_from_euler(*visual_offset.rpy)))
            return Pose(position, orientation)
        else:
            return None

    def link_as_marker(self, link_name):
        if link_name not in self._link_to_marker:
            marker = Marker()
            geometry = self.get_link(link_name).geometry.values()[0]

            if geometry.type == GEOM_TYPE_MESH:
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = geometry.mesh
                if geometry.scale is None:
                    marker.scale.x = 1.0
                    marker.scale.z = 1.0
                    marker.scale.y = 1.0
                else:
                    marker.scale.x = geometry.scale[0]
                    marker.scale.z = geometry.scale[1]
                    marker.scale.y = geometry.scale[2]
                marker.mesh_use_embedded_materials = True
            elif geometry.type == GEOM_TYPE_BOX:
                marker.type = Marker.CUBE
                marker.scale.x = geometry.scale[0]
                marker.scale.y = geometry.scale[1]
                marker.scale.z = geometry.scale[2]
            elif geometry.type == GEOM_TYPE_CYLINDER:
                marker.type = Marker.CYLINDER
                marker.scale.x = geometry.scale[0]
                marker.scale.y = geometry.scale[0]
                marker.scale.z = geometry.scale[2]
            elif geometry.type == GEOM_TYPE_SPHERE:
                marker.type = Marker.SPHERE
                marker.scale.x = geometry.scale[0]
                marker.scale.y = geometry.scale[0]
                marker.scale.z = geometry.scale[0]
            else:
                return None

            marker.header.frame_id = self.get_root()
            marker.action = Marker.ADD
            marker.color.a = 0.5
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0

            marker.scale.x *= 0.99
            marker.scale.y *= 0.99
            marker.scale.z *= 0.99

            marker.pose = Pose()
            if self.has_non_identity_visual_offset(link_name):
                marker.pose = self.get_visual_pose(link_name)
            self._link_to_marker[link_name] = marker

        return deepcopy(self._link_to_marker[link_name])

    def get_link_path(self, link_name):
        """
        :type object_name: str
        :type link_name: str
        :rtype: Path
        """
        if link_name in self.links:
            path = Path(self.get_name()) + ('links', link_name)
            return path
        if link_name in self.attached_objects:
            obj = self._world.get_object(link_name)
            return obj.get_link_path(obj.get_root())
        for object_name in self.attached_objects:
            try:
                obj = self._world.get_object(object_name)
                return obj.get_link_path(link_name)
            except KeyError:
                pass
        raise KeyError(u'link {} is not present in object {}'.format(link_name, self.get_name()))

    @memoize
    def get_link_path_str(self, link_name):
        return str(self.get_link_path(link_name))

    @memoize
    def get_link_paths(self):
        return {self.get_link_path(link_name) for link_name in self.get_link_names()}

    @memoize
    def get_link_path_strs(self):
        return {str(self.get_link_path(link_name)) for link_name in self.get_link_names()}

    def get_joint_path(self, joint_name):
        if joint_name in self.joints:
            path = Path(self.get_name()) + ('joints', joint_name)
            return path
        for object_name in self.attached_objects:
            try:
                obj = self._world.get_object(object_name)
                return obj.get_joint_path(obj.get_root())
            except KeyError:
                pass
        raise KeyError(u'joint {} is not present in object {}'.format(joint_name, self.get_name()))
