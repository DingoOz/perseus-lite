"""Tests for the URDF/SDF -> MJCF model builder."""

import math
import os
import tempfile

import numpy as np
from perseus_lite_mujoco import model_builder as mb
import pytest

URDF = """<robot name="r">
  <link name="base_link"/>
  <link name="body">
    <inertial><origin xyz="0.1 0 0"/><mass value="2.0"/>
      <inertia ixx="0.01" iyy="0.02" izz="0.03" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><origin xyz="0 0 0.1"/><geometry><box size="0.4 0.2 0.1"/></geometry></collision>
  </link>
  <link name="lump">
    <inertial><mass value="1.0"/>
    <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <link name="wheel">
    <inertial><mass value="0.1"/>
    <inertia ixx="1e-4" iyy="1e-4" izz="2e-4" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><cylinder radius="0.07" length="0.05"/></geometry></collision>
  </link>
  <joint name="body_joint" type="fixed"><parent link="base_link"/><child link="body"/></joint>
  <joint name="lump_joint" type="fixed"><origin xyz="-0.2 0 0"/><parent link="body"/>
    <child link="lump"/></joint>
  <joint name="wheel_joint" type="continuous"><origin xyz="0 0.2 0.07" rpy="-1.5707963 0 0"/>
    <parent link="base_link"/><child link="wheel"/><axis xyz="0 0 1"/></joint>
</robot>"""

SDF = """<sdf version="1.8"><world name="w">
  <light name="sun" type="directional"><direction>0 0 -1</direction></light>
  <model name="ground"><static>true</static><link name="l">
    <collision name="c"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane>
    </geometry></collision>
  </link></model>
  <model name="rock"><pose>1 2 0.05 0 0 0.5</pose><link name="l">
    <collision name="c"><geometry><box><size>0.2 0.2 0.1</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>0.2 0.2 0.1</size></box></geometry>
      <material><diffuse>0.5 0.4 0.3 1</diffuse></material></visual>
  </link></model>
  <include><uri>model://tag</uri><name>tag0</name><pose>2 0 0.25 0 0 3.14159</pose></include>
  <include><uri>https://fuel.example/remote</uri></include>
</world></sdf>"""

TAG_SDF = """<sdf version="1.8"><model name="tag"><static>true</static><link name="link">
  <visual name="face"><geometry><plane><normal>1 0 0</normal><size>0.2 0.2</size></plane>
    </geometry>
    <material><pbr><metal><albedo_map>model://tag/materials/textures/tag.png</albedo_map>
    </metal></pbr></material>
  </visual></link></model></sdf>"""


def test_rpy_and_quaternion_agree():
    r = mb.rpy_matrix(0.3, -0.2, 1.1)
    w, x, y, z = mb.matrix_to_quat(r)
    assert w * w + x * x + y * y + z * z == pytest.approx(1.0)
    # Rebuild the matrix from the quaternion.
    q = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )
    assert np.allclose(q, r, atol=1e-9)


def test_fixed_links_lump_with_exact_inertia():
    robot = mb.build_robot(URDF)
    assert robot.base.mass == pytest.approx(3.0)
    # COM: 2 kg at x=0.1, 1 kg at x=-0.2 -> 0.0
    assert robot.base.com == pytest.approx([0.0, 0.0, 0.0])
    # Parallel axis about the combined COM: izz = 0.03 + 2*0.1^2 + 0.001 + 1*0.2^2
    assert robot.base.inertia[2, 2] == pytest.approx(0.03 + 0.02 + 0.001 + 0.04)
    assert robot.base.inertia[0, 0] == pytest.approx(0.01 + 0.001)
    assert len(robot.base.geoms) == 1
    assert robot.base.geoms[0].size == pytest.approx((0.2, 0.1, 0.05))


def test_continuous_joints_become_wheels_with_axes_in_their_frame():
    robot = mb.build_robot(URDF)
    assert [w.joint_name for w in robot.wheels] == ['wheel_joint']
    wheel = robot.wheels[0]
    assert wheel.pose[:3, 3] == pytest.approx([0.0, 0.2, 0.07])
    # The joint axis (z in the wheel frame) is +y in the base frame.
    assert wheel.pose[:3, :3] @ wheel.joint_axis == pytest.approx(
        [0.0, 1.0, 0.0], abs=1e-6
    )
    assert wheel.geoms[0].kind == 'cylinder'
    assert 'wheel' in robot.frames


def test_world_conversion_reads_models_includes_and_textures():
    with tempfile.TemporaryDirectory() as root:
        os.makedirs(os.path.join(root, 'tag', 'materials', 'textures'))
        with open(os.path.join(root, 'tag', 'model.sdf'), 'w') as handle:
            handle.write(TAG_SDF)
        open(
            os.path.join(root, 'tag', 'materials', 'textures', 'tag.png'), 'wb'
        ).close()
        world = os.path.join(root, 'world.sdf')
        with open(world, 'w') as handle:
            handle.write(SDF)

        def resolve(uri):
            return (
                os.path.join(root, uri.replace('model://', '', 1))
                if uri.startswith('model://')
                else None
            )

        geoms, lights, notes = mb.build_world(world, resolve)
    kinds = {g.name: g for g in geoms}
    assert kinds['ground/c'].kind == 'plane'
    rock = kinds['rock/c']
    assert rock.size == pytest.approx((0.1, 0.1, 0.05))
    assert rock.pose[:3, 3] == pytest.approx([1, 2, 0.05])
    assert rock.rgba == pytest.approx((0.5, 0.4, 0.3, 1.0))
    face = kinds['tag0/face']
    assert face.texture.endswith('tag.png') and not face.collide
    # The tag faces -x (yaw pi): the plane's +z must point along -x.
    assert face.pose[:3, 2] == pytest.approx([-1, 0, 0], abs=1e-4)
    assert len(lights) == 1
    assert any('remote' in note for note in notes)


def test_mjcf_compiles_and_robot_drives_straight():
    mujoco = pytest.importorskip('mujoco')
    robot = mb.build_robot(URDF)
    # Mirror the single wheel to make a stable 4-wheel cart.
    for x, y, name in [
        (0.15, 0.2, 'fl'),
        (-0.15, 0.2, 'rl'),
        (0.15, -0.2, 'fr'),
        (-0.15, -0.2, 'rr'),
    ]:
        wheel = mb.Body(
            f'{name}_wheel',
            mb.transform((x, y, 0.07), (-math.pi / 2, 0, 0)),
            joint_axis=np.array([0, 0, 1.0]),
            joint_name=f'{name}_wheel_joint',
        )
        mb.accumulate_inertia(wheel, 0.1, np.zeros(3), np.diag([1e-4, 1e-4, 2e-4]))
        wheel.geoms.append(mb.Geom('cylinder', (0.07, 0.025), np.eye(4)))
        robot.wheels.append(wheel)
    robot.wheels = robot.wheels[1:]
    ground = [mb.WorldGeom('ground', 'box', (5, 5, 0.05), mb.transform((0, 0, -0.05)))]
    params = mb.SimParams(lidar_link='none', imu_link='none', camera_link='none')
    model = mujoco.MjModel.from_xml_string(
        mb.build_mjcf(robot, ground, [], (0, 0, 0.1, 0), params)
    )
    data = mujoco.MjData(model)
    for _ in range(500):
        mujoco.mj_step(model, data)
    assert data.qpos[2] == pytest.approx(0.0, abs=0.01)  # resting on its wheels
    data.ctrl[:] = 2.0  # rad/s on every wheel: 0.14 m/s
    for _ in range(1500):
        mujoco.mj_step(model, data)
    assert data.qpos[0] == pytest.approx(0.14 * 3.0, rel=0.15)
    assert abs(data.qpos[1]) < 0.05


def test_camera_fovy():
    # 73 deg horizontal on 640x480 is ~58 deg vertical.
    assert mb.camera_fovy_deg(1.274, 640, 480) == pytest.approx(57.9, abs=0.2)
