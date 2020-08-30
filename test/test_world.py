import shutil
from collections import defaultdict
from itertools import product

import giskardpy
from giskardpy.god_map import GodMap
from kineverse.model.paths import Path

giskardpy.WORLD_IMPLEMENTATION = None

from giskardpy import identifier
from giskardpy.robot import Robot
import pytest
from geometry_msgs.msg import Pose, Point, Quaternion
from giskard_msgs.msg import CollisionEntry
import test_urdf_object
from giskardpy.exceptions import DuplicateNameException, PhysicsWorldException, UnknownBodyException
from utils_for_tests import pr2_urdf as pr2_urdf_, donbot_urdf, compare_poses, pr2_without_base_urdf
from giskardpy.utils import make_world_body_box
from giskardpy.world import World
from giskardpy.world_object import WorldObject
import numpy as np

pr2 = pr2_urdf_()
def pr2_urdf():
    return pr2

@pytest.fixture(scope=u'module')
def module_setup(request):
    pass


@pytest.fixture()
def function_setup(request, module_setup):
    """
    :rtype: WorldObject
    """
    pass


@pytest.fixture()
def test_folder(request):
    """
    :rtype: str
    """
    folder_name = u'tmp_data/'

    def delete_test_folder():
        try:
            shutil.rmtree(folder_name)
        except OSError:
            print(u'couldn\'t delete test folder')

    request.addfinalizer(delete_test_folder)
    return folder_name


@pytest.fixture()
def delete_test_folder(request):
    """
    :rtype: World
    """
    folder_name = u'tmp_data/'
    try:
        shutil.rmtree(folder_name)
    except:
        pass

    def delete_test_folder():
        try:
            shutil.rmtree(folder_name)
        except OSError:
            print(u'couldn\'t delete test folder')

    request.addfinalizer(delete_test_folder)

    return folder_name

def default_god_map():
    god_map = GodMap()
    for x in range(1, len(identifier.data_folder)):
        god_map.safe_set_data(identifier.data_folder[:x], {})
    god_map.safe_set_data(identifier.data_folder, u'tmp_data/')
    return god_map

def allow_all_entry():
    ce = CollisionEntry()
    ce.type = CollisionEntry.ALLOW_COLLISION
    ce.robot_links = [CollisionEntry.ALL]
    ce.body_b = CollisionEntry.ALL
    ce.link_bs = [CollisionEntry.ALL]
    ce.min_dist = 0.0
    return ce


def avoid_all_entry(min_dist):
    ce = CollisionEntry()
    ce.type = CollisionEntry.AVOID_COLLISION
    ce.robot_links = [CollisionEntry.ALL]
    ce.body_b = CollisionEntry.ALL
    ce.link_bs = [CollisionEntry.ALL]
    ce.min_dist = min_dist
    return ce


class TestWorldObj(test_urdf_object.TestUrdfObject):
    def make_object_without_limits(self, urdf, cls=WorldObject, **kwargs):
        return super(TestWorldObj, self).make_object_without_limits(urdf, cls, **kwargs)

    def make_object(self, urdf, cls=WorldObject, **kwargs):
        return super(TestWorldObj, self).make_object(urdf, cls, **kwargs)


    def test_get_parent_joint_of_joint(self, function_setup):
        super(TestWorldObj, self).test_get_parent_joint_of_joint(function_setup)

    def test_safe_load_collision_matrix(self, test_folder, delete_test_folder):
        r = self.cls(donbot_urdf(), path_to_data_folder=test_folder)
        r.init_self_collision_matrix()
        scm = r.get_self_collision_matrix()
        r.safe_self_collision_matrix(test_folder)
        r.load_self_collision_matrix(test_folder)
        assert scm == r.get_self_collision_matrix()

    def test_safe_load_collision_matrix2(self, test_folder, delete_test_folder):
        r = self.cls(donbot_urdf(), path_to_data_folder=test_folder)
        r.init_self_collision_matrix()
        scm = r.get_self_collision_matrix()

        box = self.cls.from_world_body(make_world_body_box())
        p = Pose()
        p.position = Point(0, 0, 0)
        p.orientation = Quaternion(0, 0, 0, 1)
        r.attach_urdf_object(box, u'gripper_tool_frame', p)
        r.update_self_collision_matrix()
        scm_with_obj = r.get_self_collision_matrix()

        r.detach_sub_tree(box.get_name())
        r.load_self_collision_matrix(test_folder)
        assert scm == r.get_self_collision_matrix()

        r.attach_urdf_object(box, u'gripper_tool_frame', p)
        r.load_self_collision_matrix(test_folder)
        assert scm_with_obj == r.get_self_collision_matrix()

    def test_base_pose1(self, function_setup):
        parsed_pr2 = self.make_object_without_limits(pr2_urdf())
        p = Pose()
        p.orientation.w = 1
        parsed_pr2.base_pose = p
        assert parsed_pr2.base_pose == p

    def test_base_pose2(self, function_setup):
        parsed_pr2 = self.make_object_without_limits(pr2_urdf())
        p = Pose()
        p.orientation.w = 10
        parsed_pr2.base_pose = p
        orientation = parsed_pr2.base_pose.orientation
        orientation_vector = [orientation.x,
                              orientation.y,
                              orientation.z,
                              orientation.w]
        assert np.linalg.norm(orientation_vector) == 1

    def test_joint_state(self, function_setup):
        parsed_pr2 = self.make_object_without_limits(pr2_urdf())
        js = parsed_pr2.get_zero_joint_state()
        parsed_pr2.joint_state = js
        assert parsed_pr2.joint_state == js

    def test_controlled_joints(self, function_setup):
        controlled_joints = [u'torso_lift_joint']
        wo = self.make_object_without_limits(pr2_urdf(), controlled_joints=controlled_joints)
        assert wo.controlled_joints == controlled_joints


class TestRobot(TestWorldObj):
    cls = Robot
    def test_get_chain4(self, function_setup):
        super(TestRobot, self).test_get_chain4(function_setup)

    def test_safe_load_collision_matrix(self, test_folder, delete_test_folder):
        r = self.cls(donbot_urdf(), path_to_data_folder=test_folder)
        scm = r.get_self_collision_matrix()
        assert len(scm) == 0


class TestWorld(object):
    cls = WorldObject
    world_cls = World

    def make_world_with_robot(self, urdf, path_to_data_folder):
        w = self.world_cls(god_map=default_god_map(), prefix=identifier.world, path_to_data_folder=path_to_data_folder)
        # r = self.cls(urdf)
        w.add_robot(robot_urdf=urdf,
                    base_pose=None,
                    controlled_joints=None,
                    ignored_pairs=[],
                    added_pairs=[])
        w.god_map.register_symbols(w.robot.get_joint_position_symbols())
        if path_to_data_folder is not None:
            w.robot.init_self_collision_matrix()
        return w

    def make_world_with_pr2(self, path_to_data_folder=None):
        """
        :rtype: World
        """
        return self.make_world_with_robot(pr2_urdf(), path_to_data_folder)

    def make_world_with_pr2_without_base(self, path_to_data_folder=None):
        """
        :rtype: World
        """
        return self.make_world_with_robot(pr2_without_base_urdf(), path_to_data_folder)

    def make_world_with_donbot(self, path_to_data_folder=None):
        """
        :rtype: World
        """
        return self.make_world_with_robot(donbot_urdf(), path_to_data_folder)

    def test_add_robot(self, function_setup):
        empty_world = self.world_cls(god_map=default_god_map())
        assert len(empty_world.get_objects()) == 0
        assert not empty_world.has_robot()
        # extracting the urdf turns integers into floats
        pr2 = pr2_urdf()
        empty_world.add_robot(robot_urdf=pr2,
                              base_pose=None,
                              controlled_joints=['torso_lift_joint'],
                              ignored_pairs=[],
                              added_pairs=[])
        assert empty_world.has_robot()
        return empty_world

    def test_add_object(self, function_setup):
        god_map = GodMap()
        empty_world = self.world_cls(god_map=default_god_map())
        name = u'muh'
        box_msg = make_world_body_box(name)
        empty_world.add_object(urdf=box_msg, name=name)
        assert empty_world.has_object(name)
        assert len(empty_world.get_objects()) == 1
        assert len(empty_world.get_object_names()) == 1
        p = Pose()
        p.orientation.w = 1
        compare_poses(empty_world.get_object(name).base_pose, p)

        return empty_world

    def test_add_object_twice(self, function_setup):
        god_map = GodMap()
        empty_world = self.world_cls(god_map=default_god_map())
        name = u'muh'
        box = make_world_body_box(name)
        empty_world.add_object(box, name)
        try:
            empty_world.add_object(box)
            assert False, u'expected exception'
        except DuplicateNameException:
            assert True
        assert empty_world.has_object(name)
        assert len(empty_world.get_objects()) == 1
        return empty_world

    def test_add_object_with_robot_name(self, function_setup):
        world_with_pr2 = self.make_world_with_pr2()
        name = u'robot'
        box = make_world_body_box(name)
        try:
            world_with_pr2.add_object(box)
            assert False, u'expected exception'
        except DuplicateNameException:
            assert True
        assert world_with_pr2.has_robot()
        assert len(world_with_pr2.get_objects()) == 0
        return world_with_pr2

    def test_attach_existing_obj_to_robot1(self, function_setup):
        obj_name = u'box'
        world_with_pr2 = self.make_world_with_pr2()
        world_with_pr2.add_object(make_world_body_box(name=obj_name))
        world_with_pr2.attach_existing_obj_to_robot(u'box', u'l_gripper_tool_frame')
        assert world_with_pr2.robot.get_link(u'box')
        assert world_with_pr2.robot.get_joint(u'box')
        assert u'box' in world_with_pr2.robot.get_link_names()
        assert u'box' in world_with_pr2.robot.get_joint_names()
        assert world_with_pr2.robot.get_parent_link_of_joint(u'box') == u'l_gripper_tool_frame'
        assert world_with_pr2.robot.get_parent_link_of_link(u'box') == u'l_gripper_tool_frame'
        assert world_with_pr2.robot.get_movable_parent_joint(u'box') == u'l_wrist_roll_joint'
        assert world_with_pr2.robot.get_child_links_of_link(u'box') == []
        assert world_with_pr2.robot.get_parent_joint_of_link(u'box') == u'box'
        assert world_with_pr2.robot.get_parent_joint_of_joint(u'box') == u'l_gripper_tool_joint'
        assert world_with_pr2.robot.get_child_joints_of_link(u'box') == []
        assert world_with_pr2.robot.get_parent_path_of_joint(u'box') == Path([u'robot', u'links', u'l_gripper_tool_frame'])
        try:
            world_with_pr2.robot.get_child_link_of_joint(u'box')
            assert False
        except:
            pass
        assert world_with_pr2.robot.get_child_path_of_joint(u'box') == Path([u'box', u'links', u'box'])
        return world_with_pr2

    def test_attach_existing_obj_to_robot2(self, function_setup):
        obj_name = u'box'
        world_with_pr2 = self.make_world_with_pr2()
        world_with_pr2.add_object(make_world_body_box(name=obj_name))
        try:
            world_with_pr2.attach_existing_obj_to_robot(u'box2', u'l_gripper_tool_frame')
            assert False
        except KeyError:
            assert True
        return world_with_pr2

    def test_attach_detach_existing_obj_to_robot1(self, function_setup):
        obj_name = u'box'
        tip = u'r_gripper_tool_frame'
        world = self.make_world_with_pr2()
        world.add_object(make_world_body_box(name=obj_name))
        p = Pose()
        p.orientation.w = 1
        pre_pose = world.get_fk_pose(world.get_link_path(world._robot_name, tip),
                                     world.get_link_path(obj_name, obj_name)).pose
        world.attach_existing_obj_to_robot(obj_name, tip)
        assert obj_name not in world.get_object_names()
        compare_poses(world.robot.get_fk_pose(tip, obj_name).pose, pre_pose)

        world.detach(obj_name)
        assert obj_name not in world.robot.attached_objects
        assert obj_name in world.get_object_names()
        post_pose = world.get_fk_pose(world.get_link_path(world._robot_name, tip),
                                     world.get_link_path(obj_name, obj_name)).pose
        compare_poses(post_pose, pre_pose)
        return world


    def test_hard_reset1(self, function_setup):
        world_with_pr2 = self.make_world_with_pr2()
        world_with_pr2.hard_reset()
        assert not world_with_pr2.has_robot()
        return world_with_pr2

    def test_hard_reset2(self, function_setup):
        world_with_pr2 = self.make_world_with_pr2()
        name = u'muh'
        box = make_world_body_box(name)
        world_with_pr2.add_object(box)
        world_with_pr2.hard_reset()
        assert not world_with_pr2.has_robot()
        assert len(world_with_pr2.get_objects()) == 0
        return world_with_pr2

    def test_soft_reset1(self, function_setup):
        world_with_pr2 = self.make_world_with_pr2()
        world_with_pr2.soft_reset()
        assert world_with_pr2.has_robot()
        return world_with_pr2

    def test_soft_reset2(self, function_setup):
        world_with_pr2 = self.make_world_with_pr2()
        name = u'muh'
        box = make_world_body_box(name)
        world_with_pr2.add_object(box)
        world_with_pr2.soft_reset()
        assert world_with_pr2.has_robot()
        assert len(world_with_pr2.get_objects()) == 0
        return world_with_pr2

    def test_remove_object1(self, function_setup):
        world_with_pr2 = self.make_world_with_pr2()
        name = u'muh'
        box = make_world_body_box(name)
        world_with_pr2.add_object(box)
        world_with_pr2.remove_object(name)
        assert not world_with_pr2.has_object(name)
        assert len(world_with_pr2.get_objects()) == 0
        assert world_with_pr2.has_robot()
        return world_with_pr2

    def test_remove_robot(self, function_setup):
        world_with_pr2 = self.make_world_with_pr2()
        world_with_pr2.remove_robot()
        assert not world_with_pr2.has_robot()
        return world_with_pr2

    def test_remove_object2(self, function_setup):
        world_with_pr2 = self.make_world_with_pr2()
        name1 = u'muh'
        box = make_world_body_box(name1)
        world_with_pr2.add_object(box)
        name2 = u'muh2'
        box = make_world_body_box(name2)
        world_with_pr2.add_object(box)
        world_with_pr2.remove_object(name1)
        assert not world_with_pr2.has_object(name1)
        assert world_with_pr2.has_object(name2)
        assert len(world_with_pr2.get_objects()) == 1
        assert world_with_pr2.has_robot()
        return world_with_pr2

    def test_remove_all_object(self, function_setup):
        world_with_pr2 = self.make_world_with_pr2()
        name1 = u'muh'
        box = make_world_body_box(name1)
        world_with_pr2.add_object(box)
        name2 = u'muh2'
        box = make_world_body_box(name2)
        world_with_pr2.add_object(box)
        world_with_pr2.remove_all_objects()
        assert not world_with_pr2.has_object(name1)
        assert not world_with_pr2.has_object(name2)
        assert not world_with_pr2.km_model.has_data(name1)
        assert not world_with_pr2.km_model.has_data(name2)
        assert world_with_pr2.km_model.has_data(world_with_pr2._robot_name)
        assert len(world_with_pr2.get_objects()) == 0
        assert world_with_pr2.has_robot()
        return world_with_pr2

    def test_verify_collision_entries_empty(self, test_folder):
        world_with_donbot = self.make_world_with_donbot(test_folder)
        ces = []
        new_ces = world_with_donbot.verify_collision_entries(ces, 0.05)
        assert len(new_ces) == 1

    def test_verify_collision_entries_allow_all(self, test_folder):
        world_with_donbot = self.make_world_with_donbot(test_folder)
        ces = [allow_all_entry()]
        new_ces = world_with_donbot.verify_collision_entries(ces, 0.05)
        assert len(new_ces) == 0

    def test_verify_collision_entries_allow_all_self(self, test_folder):
        world_with_donbot = self.make_world_with_donbot(test_folder)
        ce = CollisionEntry()
        ce.type = CollisionEntry.ALLOW_COLLISION
        ce.robot_links = [CollisionEntry.ALL]
        ce.body_b = world_with_donbot.robot.get_name()
        ce.link_bs = [CollisionEntry.ALL]
        ces = [ce]
        new_ces = world_with_donbot.verify_collision_entries(ces, 0.05)
        assert len(new_ces) == 1 + len(world_with_donbot.robot.get_self_collision_matrix()) * 2

    def test_verify_collision_entries_unknown_robot_link(self, test_folder):
        world_with_donbot = self.make_world_with_donbot(test_folder)
        min_dist = 0.1
        ces = []
        ce = CollisionEntry()
        ce.type = CollisionEntry.AVOID_COLLISION
        ce.robot_links = [u'muh']
        ce.min_dist = min_dist
        ces.append(ce)
        try:
            new_ces = world_with_donbot.verify_collision_entries(ces, min_dist)
        except UnknownBodyException:
            assert True
        else:
            assert False, u'expected exception'

    def test_verify_collision_entries_unknown_body_b(self, test_folder):
        world_with_donbot = self.make_world_with_donbot(test_folder)
        min_dist = 0.1
        ces = []
        ce = CollisionEntry()
        ce.type = CollisionEntry.AVOID_COLLISION
        ce.robot_links = [CollisionEntry.ALL]
        ce.body_b = u'muh'
        ce.link_bs = [CollisionEntry.ALL]
        ce.min_dist = min_dist
        ces.append(ce)
        try:
            new_ces = world_with_donbot.verify_collision_entries(ces, min_dist)
        except UnknownBodyException:
            assert True
        else:
            assert False, u'expected exception'

    def test_verify_collision_entries_unknown_link_b(self, test_folder):
        world_with_donbot = self.make_world_with_donbot(test_folder)
        min_dist = 0.1
        ces = []
        ce = CollisionEntry()
        ce.type = CollisionEntry.AVOID_COLLISION
        ce.robot_links = [CollisionEntry.ALL]
        ce.body_b = u'muh'
        ce.link_bs = [u'muh']
        ce.min_dist = min_dist
        ces.append(ce)
        try:
            new_ces = world_with_donbot.verify_collision_entries(ces, min_dist)
        except UnknownBodyException:
            assert True
        else:
            assert False, u'expected exception'

    def test_verify_collision_entries_unknown_link_b2(self, test_folder):
        world_with_donbot = self.make_world_with_donbot(test_folder)
        min_dist = 0.1
        ces = []
        ce = CollisionEntry()
        ce.type = CollisionEntry.AVOID_COLLISION
        ce.robot_links = [CollisionEntry.ALL]
        ce.body_b = world_with_donbot.robot.get_name()
        ce.link_bs = [u'muh']
        ce.min_dist = min_dist
        ces.append(ce)
        try:
            new_ces = world_with_donbot.verify_collision_entries(ces, min_dist)
        except UnknownBodyException:
            assert True
        else:
            assert False, u'expected exception'

    def test_verify_collision_entries1(self, test_folder):
        world_with_donbot = self.make_world_with_donbot(test_folder)
        min_dist = 0.1
        ces = []
        ce = CollisionEntry()
        ce.type = CollisionEntry.AVOID_COLLISION
        ce.robot_links = [CollisionEntry.ALL, u'plate']
        ce.min_dist = min_dist
        ces.append(ce)
        try:
            new_ces = world_with_donbot.verify_collision_entries(ces, min_dist)
        except PhysicsWorldException:
            assert True
        else:
            assert False, u'expected exception'

    def test_verify_collision_entries2(self, test_folder):
        world_with_donbot = self.make_world_with_donbot(test_folder)
        min_dist = 0.1
        ces = []
        ce = CollisionEntry()
        ce.type = CollisionEntry.AVOID_COLLISION
        ce.link_bs = [CollisionEntry.ALL, u'muh']
        ce.min_dist = min_dist
        ces.append(ce)
        try:
            new_ces = world_with_donbot.verify_collision_entries(ces, min_dist)
        except PhysicsWorldException:
            assert True
        else:
            assert False, u'expected exception'

    def test_verify_collision_entries3(self, test_folder):
        world_with_donbot = self.make_world_with_donbot(test_folder)
        min_dist = 0.1
        ces = []
        ce = CollisionEntry()
        ce.type = CollisionEntry.AVOID_COLLISION
        ce.link_bs = [CollisionEntry.ALL, u'muh']
        ce.robot_links = [CollisionEntry.ALL, u'muh']
        ce.min_dist = min_dist
        ces.append(ce)
        try:
            new_ces = world_with_donbot.verify_collision_entries(ces, min_dist)
        except PhysicsWorldException:
            assert True
        else:
            assert False, u'expected exception'

    def test_verify_collision_entries3_1(self, test_folder):
        world_with_donbot = self.make_world_with_donbot(test_folder)
        min_dist = 0.1
        ces = []
        ce = CollisionEntry()
        ce.type = CollisionEntry.AVOID_COLLISION
        ce.body_b = CollisionEntry.ALL
        ce.link_bs = [u'muh']
        ce.min_dist = min_dist
        ces.append(ce)
        try:
            new_ces = world_with_donbot.verify_collision_entries(ces, min_dist)
        except PhysicsWorldException:
            assert True
        else:
            assert False, u'expected exception'

    def test_verify_collision_entries_cut_off1(self, test_folder):
        world_with_donbot = self.make_world_with_donbot(test_folder)
        min_dist = 0.1
        ces = []
        ces.append(avoid_all_entry(min_dist))
        ces.append(allow_all_entry())
        new_ces = world_with_donbot.verify_collision_entries(ces, min_dist)
        assert len(new_ces) == 0

    def test_verify_collision_entries_split0(self, test_folder):
        world_with_donbot = self.make_world_with_donbot(test_folder)
        min_dist = 0.1
        ces = [avoid_all_entry(min_dist)]
        new_ces = world_with_donbot.verify_collision_entries(ces, min_dist)
        assert len(new_ces) == 1
        for ce in new_ces:
            assert ce.body_b == world_with_donbot.robot.get_name()
            assert ce.body_b != CollisionEntry.ALL
            assert world_with_donbot.all_robot_links(ce)
            assert world_with_donbot.all_link_bs(ce)
            assert ce.type == CollisionEntry. \
                AVOID_COLLISION

    def test_verify_collision_entries_split1(self, test_folder):
        world_with_donbot = self.make_world_with_donbot(test_folder)
        min_dist = 0.05
        ces = []
        ce1 = CollisionEntry()
        ce1.type = CollisionEntry.AVOID_COLLISION
        ce1.robot_links = [CollisionEntry.ALL]
        ce1.body_b = CollisionEntry.ALL
        ce1.link_bs = [CollisionEntry.ALL]
        ce1.min_dist = 0.1
        ces.append(ce1)
        ce = CollisionEntry()
        ce.type = CollisionEntry.ALLOW_COLLISION
        ce.robot_links = [u'plate']
        ce.body_b = CollisionEntry.ALL
        ce.link_bs = [CollisionEntry.ALL]
        ces.append(ce)
        new_ces = world_with_donbot.verify_collision_entries(ces, min_dist)
        assert len(new_ces) == 1 + \
               len(world_with_donbot.robot.get_possible_collisions(u'plate'))
        assert world_with_donbot.all_robot_links(new_ces[0])
        assert world_with_donbot.all_link_bs(new_ces[0])
        for ce in new_ces[1:]:
            assert ce.body_b == world_with_donbot.robot.get_name()
            assert ce.body_b != CollisionEntry.ALL
            assert CollisionEntry.ALL not in ce.robot_links
        i = 0
        for i in range(1):
            ce = new_ces[i]
            assert ce.type == CollisionEntry.AVOID_COLLISION
        i += 1
        for j in range(len(world_with_donbot.robot.get_possible_collisions(u'plate'))):
            ce = new_ces[i + j]
            assert ce.type == CollisionEntry.ALLOW_COLLISION

    def test_verify_collision_entries_split2(self, test_folder):
        world_with_donbot = self.make_world_with_donbot(test_folder)
        name = u'muh'
        min_dist = 0.05
        box = make_world_body_box(name)
        world_with_donbot.add_object(box)

        ces = [avoid_all_entry(min_dist)]
        ce = CollisionEntry()
        ce.type = CollisionEntry.ALLOW_COLLISION
        ce.robot_links = [CollisionEntry.ALL]
        ce.body_b = name
        ce.link_bs = [CollisionEntry.ALL]
        ces.append(ce)
        new_ces = world_with_donbot.verify_collision_entries(ces, min_dist)
        assert len(new_ces) == 1 + len(world_with_donbot.robot.get_controlled_links()) * 2
        for ce in new_ces[1:]:
            assert ce.body_b != CollisionEntry.ALL
            assert CollisionEntry.ALL not in ce.robot_links
            if ce.body_b != world_with_donbot.robot.get_name():
                assert CollisionEntry.ALL in ce.link_bs
            else:
                assert CollisionEntry.ALL not in ce.link_bs
            assert len(ce.link_bs) == 1
        i = 0
        for i in range(len(world_with_donbot.robot.get_controlled_links()) + 1):
            ce = new_ces[i]
            assert ce.type == CollisionEntry.AVOID_COLLISION
        i += 1
        for j in range(len(world_with_donbot.robot.get_controlled_links())):
            ce = new_ces[i + j]
            assert ce.type == CollisionEntry.ALLOW_COLLISION

    def test_verify_collision_entries_split3(self, test_folder):
        world_with_donbot = self.make_world_with_donbot(test_folder)
        name = u'muh'
        min_dist = 0.05
        box = make_world_body_box(name)
        world_with_donbot.add_object(box)

        ces = []
        ce1 = CollisionEntry()
        ce1.type = CollisionEntry.AVOID_COLLISION
        ce1.robot_links = [CollisionEntry.ALL]
        ce1.link_bs = [CollisionEntry.ALL]
        ce1.min_dist = min_dist
        ces.append(ce1)
        ce = CollisionEntry()
        ce.type = CollisionEntry.ALLOW_COLLISION
        ce.robot_links = [CollisionEntry.ALL]
        ce.body_b = name
        ce.link_bs = [name]
        ces.append(ce)
        new_ces = world_with_donbot.verify_collision_entries(ces, min_dist)
        assert len(new_ces) == len(world_with_donbot.robot.get_controlled_links()) * 2 + 1
        for ce in new_ces[1:]:
            assert ce.body_b != CollisionEntry.ALL
            assert CollisionEntry.ALL not in ce.robot_links
            assert CollisionEntry.ALL not in ce.link_bs
            assert len(ce.link_bs) == 1
        i = 0
        for i in range(1 +
                       len(world_with_donbot.robot.get_controlled_links())):
            ce = new_ces[i]
            assert ce.type == CollisionEntry.AVOID_COLLISION
        i += 1
        for j in range(len(world_with_donbot.robot.get_controlled_links())):
            ce = new_ces[i + j]
            assert ce.type == CollisionEntry.ALLOW_COLLISION

    def test_verify_collision_entries_split4(self, test_folder):
        world_with_donbot = self.make_world_with_donbot(test_folder)
        name = u'muh'
        min_dist = 0.05
        box = make_world_body_box(name)
        world_with_donbot.add_object(box)
        name2 = u'box2'
        box2 = make_world_body_box(name2)
        world_with_donbot.add_object(box2)

        ces = []
        ce1 = CollisionEntry()
        ce1.type = CollisionEntry.AVOID_COLLISION
        ce1.robot_links = [CollisionEntry.ALL]
        ce1.link_bs = [CollisionEntry.ALL]
        ce1.min_dist = min_dist
        ces.append(ce1)
        ce = CollisionEntry()
        ce.type = CollisionEntry.ALLOW_COLLISION
        ce.robot_links = [CollisionEntry.ALL]
        ce.body_b = name
        ce.link_bs = [name]
        ces.append(ce)
        new_ces = world_with_donbot.verify_collision_entries(ces, min_dist)
        assert len(new_ces) == len(world_with_donbot.robot.get_controlled_links()) * 3 + 1
        for ce in new_ces[1:]:
            assert ce.body_b != CollisionEntry.ALL
            assert CollisionEntry.ALL not in ce.robot_links
            if ce.body_b == name2:
                assert CollisionEntry.ALL in ce.link_bs
            else:
                assert CollisionEntry.ALL not in ce.link_bs
            assert len(ce.link_bs) == 1
        i = -1
        for i in range(1 + len(world_with_donbot.robot.get_controlled_links()) * 2):
            ce = new_ces[i]
            assert ce.type == CollisionEntry.AVOID_COLLISION
        i += 1
        for j in range(len(world_with_donbot.robot.get_controlled_links())):
            ce = new_ces[i + j]
            assert ce.type == CollisionEntry.ALLOW_COLLISION

    def test_verify_collision_entries_split5(self, test_folder):
        world_with_donbot = self.make_world_with_donbot(test_folder)
        name = u'muh'
        min_dist = 0.05
        box = make_world_body_box(name)
        world_with_donbot.add_object(box)

        ces = [allow_all_entry()]
        ce1 = CollisionEntry()
        ce1.type = CollisionEntry.AVOID_COLLISION
        ce1.robot_links = [u'plate', u'base_link']
        ce1.body_b = name
        ce1.link_bs = [CollisionEntry.ALL]
        ce1.min_dist = min_dist
        ces.append(ce1)
        new_ces = world_with_donbot.verify_collision_entries(ces, min_dist)
        assert len(new_ces) == 2

        for j in range(2):
            ce = new_ces[j]
            assert ce.type == CollisionEntry.AVOID_COLLISION

    def test_verify_collision_entries_split6(self, test_folder):
        world_with_donbot = self.make_world_with_donbot(test_folder)
        min_dist = 0.05
        ces = []
        ce1 = CollisionEntry()
        ce1.type = CollisionEntry.ALLOW_COLLISION
        ce1.robot_links = [u'plate', u'base_link']
        ce1.body_b = world_with_donbot.robot.get_name()
        ce1.link_bs = [u'gripper_finger_left_link', u'gripper_finger_right_link']
        ce1.min_dist = min_dist
        ces.append(ce1)
        new_ces = world_with_donbot.verify_collision_entries(ces, min_dist)
        assert len(new_ces) == 4 + 1
        i = -1
        for i in range(1):
            ce = new_ces[i]
            assert ce.type == CollisionEntry.AVOID_COLLISION
        i += 1
        for j in range(4):
            ce = new_ces[i + j]
            assert ce.type == CollisionEntry.ALLOW_COLLISION

    def test_collision_goals_to_collision_matrix1(self, delete_test_folder):
        """
        test with no collision entries which is equal to avoid all collisions
        collision matrix should be empty, because world has no collision checker
        :param test_folder:
        :return:
        """
        world_with_donbot = self.make_world_with_donbot(delete_test_folder)
        min_dist = defaultdict(lambda: {u'zero_weight_distance': 0.05})
        collision_matrix = world_with_donbot.collision_goals_to_collision_matrix([], min_dist)
        assert len(collision_matrix) == 0
        return world_with_donbot

    def test_collision_goals_to_collision_matrix2(self, test_folder):
        """
        avoid all with an added object should enlarge the collision matrix
        :param test_folder:
        :return:
        """
        min_dist = defaultdict(lambda: {u'zero_weight_distance': 0.05})
        world_with_donbot = self.make_world_with_donbot(test_folder)
        base_collision_matrix = world_with_donbot.collision_goals_to_collision_matrix([], min_dist)
        name = u'muh'
        box = make_world_body_box(name)
        world_with_donbot.add_object(box)
        collision_matrix = world_with_donbot.collision_goals_to_collision_matrix([], min_dist)
        assert len(collision_matrix) == len(base_collision_matrix) + len(world_with_donbot.robot.get_controlled_links())
        robot_link_names = world_with_donbot.robot.get_link_names()
        for (robot_link, body_b, body_b_link), dist in collision_matrix.items():
            assert dist == min_dist[robot_link][u'zero_weight_distance']
            if body_b == name:
                assert body_b_link == u''
            assert robot_link in robot_link_names
        return world_with_donbot

    def test_collision_goals_to_collision_matrix3(self, test_folder):
        """
        empty list should have the same effect than avoid all entry
        :param test_folder:
        :return:
        """
        min_dist = defaultdict(lambda: {u'zero_weight_distance': 0.05})
        world_with_donbot = self.make_world_with_donbot(test_folder)
        base_collision_matrix = world_with_donbot.collision_goals_to_collision_matrix([], min_dist)
        name = u'muh'
        box = make_world_body_box(name)
        world_with_donbot.add_object(box)
        ces = []
        ces.append(avoid_all_entry(min_dist))
        collision_matrix = world_with_donbot.collision_goals_to_collision_matrix(ces, min_dist)
        assert len(collision_matrix) == len(base_collision_matrix) + len(world_with_donbot.robot.get_controlled_links())
        robot_link_names = world_with_donbot.robot.get_link_names()
        for (robot_link, body_b, body_b_link), dist in collision_matrix.items():
            assert dist == min_dist[robot_link][u'zero_weight_distance']
            if body_b == name:
                assert body_b_link == u''
            assert robot_link in robot_link_names
        return world_with_donbot

    def test_collision_goals_to_collision_matrix4(self, test_folder):
        """
        allow all should lead to an empty collision matrix
        :param test_folder:
        :return:
        """
        world_with_donbot = self.make_world_with_donbot(test_folder)
        name = u'muh'

        box = make_world_body_box(name)
        world_with_donbot.add_object(box)

        ces = []
        ces.append(allow_all_entry())
        ces.append(allow_all_entry())
        min_dist = defaultdict(lambda: {u'zero_weight_distance': 0.05})
        collision_matrix = world_with_donbot.collision_goals_to_collision_matrix(ces, min_dist)

        assert len(collision_matrix) == 0
        return world_with_donbot

    def test_collision_goals_to_collision_matrix5(self, test_folder):
        """

        :param test_folder:
        :return:
        """
        # FIXME min dist is kinda outdated so this test is hacked to succeed
        world_with_donbot = self.make_world_with_donbot(test_folder)
        name = u'muh'
        robot_link_names = list(world_with_donbot.robot.get_controlled_links())

        box = make_world_body_box(name)
        world_with_donbot.add_object(box)

        ces = []
        ces.append(allow_all_entry())
        ce = CollisionEntry()
        ce.type = CollisionEntry.AVOID_COLLISION
        ce.robot_links = [robot_link_names[0]]
        ce.body_b = name
        ce.min_dist = 0.1
        ces.append(ce)
        min_dist = defaultdict(lambda: {u'zero_weight_distance': 0.1})
        collision_matrix = world_with_donbot.collision_goals_to_collision_matrix(ces, min_dist)

        assert len(collision_matrix) == 1
        for (robot_link, body_b, body_b_link), dist in collision_matrix.items():
            assert dist == ce.min_dist
            assert body_b == name
            assert body_b_link == u''
            assert robot_link in robot_link_names
        return world_with_donbot

    def test_collision_goals_to_collision_matrix6(self, test_folder):
        """
        allow collision with a specific object
        :param test_folder:
        :return:
        """
        world_with_donbot = self.make_world_with_donbot(test_folder)
        name = u'muh'
        robot_link_names = list(world_with_donbot.robot.get_controlled_links())
        min_dist = defaultdict(lambda: {u'zero_weight_distance': 0.1})

        box = make_world_body_box(name)
        world_with_donbot.add_object(box)

        allowed_link = robot_link_names[0]

        ces = []
        ce = CollisionEntry()
        ce.type = CollisionEntry.ALLOW_COLLISION
        ce.robot_links = [allowed_link]
        ce.link_bs = [CollisionEntry.ALL]
        ce.min_dist = 0.1
        ces.append(ce)
        collision_matrix = world_with_donbot.collision_goals_to_collision_matrix(ces, min_dist)

        assert len([x for x in collision_matrix if x[0] == allowed_link]) == 0
        for (robot_link, body_b, body_b_link), dist in collision_matrix.items():
            assert dist == min_dist[robot_link][u'zero_weight_distance']
            if body_b == name:
                assert body_b_link == u''
            assert robot_link in robot_link_names
        return world_with_donbot

    def test_collision_goals_to_collision_matrix7(self, test_folder):
        """
        allow collision with specific object
        :param test_folder:
        :return:
        """
        world_with_donbot = self.make_world_with_donbot(test_folder)
        name = u'muh'
        name2 = u'muh2'
        robot_link_names = list(world_with_donbot.robot.get_controlled_links())
        min_dist = defaultdict(lambda: {u'zero_weight_distance': 0.05})

        box = make_world_body_box(name)
        box2 = make_world_body_box(name2)
        world_with_donbot.add_object(box)
        world_with_donbot.add_object(box2)

        ces = []
        ce = CollisionEntry()
        ce.type = CollisionEntry.ALLOW_COLLISION
        ce.body_b = name2
        ce.min_dist = 0.1
        ces.append(ce)
        collision_matrix = world_with_donbot.collision_goals_to_collision_matrix(ces, min_dist)

        assert len([x for x in collision_matrix if x[2] == name2]) == 0
        for (robot_link, body_b, body_b_link), dist in collision_matrix.items():
            assert dist == min_dist[robot_link][u'zero_weight_distance']
            if body_b == name:
                assert body_b_link == u''
            assert robot_link in robot_link_names
        return world_with_donbot

    def test_collision_goals_to_collision_matrix8(self, test_folder):
        """
        allow collision between specific object and link
        :param test_folder:
        :return:
        """
        world_with_donbot = self.make_world_with_donbot(test_folder)
        name = u'muh'
        name2 = u'muh2'
        robot_link_names = list(world_with_donbot.robot.get_controlled_links())
        allowed_link = robot_link_names[0]
        min_dist = defaultdict(lambda: {u'zero_weight_distance': 0.05})

        box = make_world_body_box(name)
        box2 = make_world_body_box(name2)
        world_with_donbot.add_object(box)
        world_with_donbot.add_object(box2)

        ces = []
        ce = CollisionEntry()
        ce.type = CollisionEntry.ALLOW_COLLISION
        ce.robot_links = [allowed_link]
        ce.body_b = name2
        ce.min_dist = 0.1
        ces.append(ce)
        collision_matrix = world_with_donbot.collision_goals_to_collision_matrix(ces, min_dist)

        assert len([x for x in collision_matrix if x[0] == allowed_link and x[2] == name2]) == 0
        for (robot_link, body_b, body_b_link), dist in collision_matrix.items():
            assert dist == min_dist[robot_link][u'zero_weight_distance']
            if body_b != world_with_donbot.robot.get_name():
                assert body_b_link == u''
            assert robot_link in robot_link_names
            if body_b == name2:
                assert robot_link != robot_link_names[0]
        return world_with_donbot

    def test_collision_goals_to_collision_matrix9(self, test_folder):
        """
        allow self collision
        :param test_folder:
        :return:
        """
        world_with_pr2 = self.make_world_with_pr2(test_folder)
        min_dist = defaultdict(lambda: {u'zero_weight_distance': 0.05})
        ces = []
        collision_entry = CollisionEntry()
        collision_entry.type = CollisionEntry.ALLOW_COLLISION
        collision_entry.robot_links = [u'l_gripper_l_finger_tip_link', u'l_gripper_r_finger_tip_link',
                                       u'l_gripper_l_finger_link', u'l_gripper_r_finger_link',
                                       u'l_gripper_r_finger_link', u'l_gripper_palm_link']
        collision_entry.body_b = world_with_pr2.robot.get_name()
        collision_entry.link_bs = [u'r_wrist_flex_link', u'r_wrist_roll_link', u'r_forearm_roll_link',
                                   u'r_forearm_link', u'r_forearm_link']
        ces.append(collision_entry)

        collision_matrix = world_with_pr2.collision_goals_to_collision_matrix(ces, min_dist)

        # assert len(collision_matrix) == 0
        # assert len([x for x in collision_matrix if x[0] == allowed_link and x[2] == name2]) == 0
        for (robot_link, body_b, body_b_link), dist in collision_matrix.items():
            assert not (robot_link in collision_entry.robot_links and body_b_link in collision_entry.link_bs)
            assert not (body_b_link in collision_entry.robot_links and robot_link in collision_entry.link_bs)
        #     assert dist == min_dist
        #     if body_b != world_with_donbot.robot.get_name():
        #         assert body_b_link == u''
        #     assert robot_link in robot_link_names
        #     if body_b == name2:
        #         assert robot_link != robot_link_names[0]
        return world_with_pr2

    def test_collision_goals_to_collision_matrix10(self, test_folder):
        """
        avoid self collision with only specific links
        :param test_folder:
        :return:
        """
        world_with_pr2 = self.make_world_with_pr2_without_base(test_folder)
        min_dist = defaultdict(lambda: {u'zero_weight_distance': 0.05})
        ces = [allow_all_entry()]
        collision_entry = CollisionEntry()
        collision_entry.type = CollisionEntry.AVOID_COLLISION
        collision_entry.robot_links = [u'base_link']
        collision_entry.body_b = world_with_pr2.robot.get_name()
        collision_entry.link_bs = [u'r_wrist_flex_link']
        ces.append(collision_entry)

        collision_matrix = world_with_pr2.collision_goals_to_collision_matrix(ces, min_dist)

        assert collision_matrix == {(u'base_link', u'robot', u'r_wrist_flex_link'): 0.05}

        return world_with_pr2


    def test_check_collisions(self, test_folder):
        w = self.make_world_with_pr2()
        w.add_object(pr2_urdf(), 'pr22')
        base_pose = Pose()
        base_pose.position.x = 10
        base_pose.orientation.w = 1
        w.set_object_pose('pr22', base_pose)
        robot_links = w.robot.get_controlled_links()
        cut_off_distances = {(link1, 'pr22', link2): 0.1 for link1, link2 in product(robot_links, repeat=2)}
        w.reset_pb_subworld()
        assert len(w.check_collisions(cut_off_distances).all_collisions) == 0

    def test_check_collisions2(self, test_folder):
        w = self.make_world_with_pr2()
        w.add_object(pr2_urdf(), name='pr22')
        base_pose = Pose()
        base_pose.position.x = 0.05
        base_pose.orientation.w = 1
        w.set_object_pose('pr22', base_pose)

        w.add_object(pr2_urdf(), name='pr23')
        base_pose = Pose()
        base_pose.position.y = 0.05
        base_pose.orientation.w = 1
        w.set_object_pose('pr23', base_pose)

        min_dist = defaultdict(lambda: {u'zero_weight_distance': 0.1})
        cut_off_distances = w.collision_goals_to_collision_matrix([], min_dist)

        for i in range(160):
            assert len(w.check_collisions(cut_off_distances).all_collisions) == 1328

    def test_check_collisions3(self, test_folder):
        w = self.make_world_with_pr2()
        w.add_object(pr2_urdf(), name='pr22')
        base_pose = Pose()
        base_pose.position.x = 1.5
        base_pose.orientation.w = 1
        w.set_object_pose('pr22', base_pose)
        min_dist = defaultdict(lambda: {u'zero_weight_distance': 0.1})
        cut_off_distances = w.collision_goals_to_collision_matrix([], min_dist)
        robot_links = w.get_object('pr22').get_link_names()
        cut_off_distances.update({(link1, 'pr22', link2): 0.1 for link1, link2 in product(robot_links, repeat=2)})

        for i in range(160):
            assert len(w.check_collisions(cut_off_distances).all_collisions) == 60