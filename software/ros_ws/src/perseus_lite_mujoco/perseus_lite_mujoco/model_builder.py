"""
Build a MuJoCo (MJCF) model of Perseus Lite in a world, from the same sources Gazebo uses.

- Robot: the xacro-generated URDF (robot_description). Links joined by fixed joints are
  lumped into one free-floating body (masses, centres of mass and inertia tensors
  combined exactly); continuous joints (the wheels) become hinge bodies with velocity
  actuators. URDF box/cylinder/sphere collisions become geoms; mesh collisions are
  skipped (none of the lite's touch the ground).
- World: a Gazebo SDF world's static models with box, cylinder, sphere or plane
  geometry, including <include>d models (e.g. the generated AprilTags) with their
  PBR albedo textures.

Keeping one source of truth means a world or URDF change applies to both simulators.
"""

from dataclasses import dataclass, field
import math
import os
from typing import Callable, Dict, List, Optional, Tuple
import xml.etree.ElementTree as ET

import numpy as np

# Collision bitmasks: robot geoms touch the world but not each other.
ROBOT_CONTYPE = 2
ROBOT_CONAFFINITY = 1
WORLD_CONTYPE = 1
WORLD_CONAFFINITY = 1
# Geom groups: lidar rays only see WORLD_GROUP (never the robot's own body or
# visual-only faces). The renderer shows groups 0-2 by default, so visual-only
# textures (VISUAL_GROUP) stay visible to the camera.
WORLD_GROUP = 0
VISUAL_GROUP = 1
ROBOT_GROUP = 2


# ---------------------------------------------------------------------------------
# Small rigid-transform helpers (4x4 homogeneous)
# ---------------------------------------------------------------------------------


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF/SDF fixed-axis roll-pitch-yaw to a rotation matrix (Rz Ry Rx)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def transform(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)) -> np.ndarray:
    """4x4 transform from a translation and roll-pitch-yaw."""
    t = np.eye(4)
    t[:3, :3] = rpy_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def matrix_to_quat(r: np.ndarray) -> Tuple[float, float, float, float]:
    """Rotation matrix to a (w, x, y, z) quaternion, MuJoCo's order."""
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        w, x = 0.25 * s, (r[2, 1] - r[1, 2]) / s
        y, z = (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w, x = (r[2, 1] - r[1, 2]) / s, 0.25 * s
        y, z = (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w, x = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s
        y, z = 0.25 * s, (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w, x = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s
        y, z = (r[1, 2] + r[2, 1]) / s, 0.25 * s
    return (w, x, y, z)


def fmt(values) -> str:
    """Space-separated numbers for MJCF attributes."""
    return ' '.join(f'{float(v):.6g}' for v in values)


def parse_origin(element: Optional[ET.Element]) -> np.ndarray:
    """URDF <origin xyz rpy> to a 4x4 transform (identity if absent)."""
    if element is None:
        return np.eye(4)
    xyz = [float(v) for v in (element.get('xyz') or '0 0 0').split()]
    rpy = [float(v) for v in (element.get('rpy') or '0 0 0').split()]
    return transform(xyz, rpy)


def parse_sdf_pose(text: Optional[str]) -> np.ndarray:
    """SDF <pose>x y z roll pitch yaw</pose> to a 4x4 transform."""
    if not text or not text.strip():
        return np.eye(4)
    v = [float(x) for x in text.split()]
    v += [0.0] * (6 - len(v))
    return transform(v[:3], v[3:6])


# ---------------------------------------------------------------------------------
# Robot: URDF -> lumped base + wheel bodies
# ---------------------------------------------------------------------------------


@dataclass
class Geom:
    """A primitive geom in its parent body's frame."""

    kind: str  # box | cylinder | sphere
    size: Tuple[float, ...]  # MJCF size (half extents / radius, half-length)
    pose: np.ndarray  # 4x4 in the body frame


@dataclass
class Body:
    """A rigid body of the MuJoCo robot."""

    name: str
    pose: np.ndarray  # in the parent body (or world for the base)
    mass: float = 0.0
    com: np.ndarray = field(default_factory=lambda: np.zeros(3))
    inertia: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))
    geoms: List[Geom] = field(default_factory=list)
    joint_axis: Optional[np.ndarray] = None  # hinge axis in this body's frame
    joint_name: str = ''


@dataclass
class RobotModel:
    """The lumped robot: one base body and its hinge children, plus named frames."""

    base: Body
    wheels: List[Body]
    frames: Dict[str, np.ndarray]  # link name -> pose in the base body frame
    skipped: List[str]


def urdf_geom(
    geometry: ET.Element, pose: np.ndarray, link: str, skipped: List[str]
) -> Optional[Geom]:
    """URDF collision geometry to a Geom, or None (recorded in skipped)."""
    child = list(geometry)[0]
    if child.tag == 'box':
        size = [float(v) / 2 for v in child.get('size').split()]
        return Geom('box', tuple(size), pose)
    if child.tag == 'cylinder':
        return Geom(
            'cylinder',
            (float(child.get('radius')), float(child.get('length')) / 2),
            pose,
        )
    if child.tag == 'sphere':
        return Geom('sphere', (float(child.get('radius')),), pose)
    skipped.append(f'{link}: {child.tag} collision')
    return None


def accumulate_inertia(
    body: Body, mass: float, com: np.ndarray, inertia: np.ndarray
) -> None:
    """Add a rigid part (mass, COM and inertia about its COM, in body axes) to a body."""
    if mass <= 0.0:
        return
    total = body.mass + mass
    new_com = (body.com * body.mass + com * mass) / total

    def shifted(m, c, i):
        d = c - new_com
        return i + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))

    body.inertia = shifted(body.mass, body.com, body.inertia) + shifted(
        mass, com, inertia
    )
    body.com = new_com
    body.mass = total


def build_robot(urdf_text: str, base_link: str = 'base_link') -> RobotModel:
    """Lump the URDF into a base body plus one hinge body per continuous joint."""
    root = ET.fromstring(urdf_text)
    links = {link.get('name'): link for link in root.findall('link')}
    children: Dict[str, List[ET.Element]] = {}
    for joint in root.findall('joint'):
        children.setdefault(joint.find('parent').get('link'), []).append(joint)

    skipped: List[str] = []
    base = Body(base_link, np.eye(4))
    wheels: List[Body] = []
    frames: Dict[str, np.ndarray] = {}

    def add_link_contents(body: Body, link_name: str, link_in_body: np.ndarray) -> None:
        link = links[link_name]
        frames[link_name] = (
            link_in_body if body is base else frames.get(link_name, link_in_body)
        )
        inertial = link.find('inertial')
        if inertial is not None:
            t = link_in_body @ parse_origin(inertial.find('origin'))
            m = float(inertial.find('mass').get('value'))
            i = inertial.find('inertia')
            local = np.array(
                [
                    [
                        float(i.get('ixx')),
                        float(i.get('ixy', 0)),
                        float(i.get('ixz', 0)),
                    ],
                    [
                        float(i.get('ixy', 0)),
                        float(i.get('iyy')),
                        float(i.get('iyz', 0)),
                    ],
                    [
                        float(i.get('ixz', 0)),
                        float(i.get('iyz', 0)),
                        float(i.get('izz')),
                    ],
                ]
            )
            r = t[:3, :3]
            accumulate_inertia(body, m, t[:3, 3], r @ local @ r.T)
        for collision in link.findall('collision'):
            geom = urdf_geom(
                collision.find('geometry'),
                link_in_body @ parse_origin(collision.find('origin')),
                link_name,
                skipped,
            )
            if geom is not None:
                body.geoms.append(geom)

    def walk(body: Body, link_name: str, link_in_body: np.ndarray) -> None:
        add_link_contents(body, link_name, link_in_body)
        for joint in children.get(link_name, []):
            child = joint.find('child').get('link')
            child_in_body = link_in_body @ parse_origin(joint.find('origin'))
            kind = joint.get('type')
            if kind == 'fixed':
                walk(body, child, child_in_body)
            elif kind in ('continuous', 'revolute'):
                axis_element = joint.find('axis')
                axis = np.array(
                    [
                        float(v)
                        for v in (
                            axis_element.get('xyz')
                            if axis_element is not None
                            else '1 0 0'
                        ).split()
                    ]
                )
                wheel = Body(
                    child,
                    child_in_body,
                    joint_axis=axis / np.linalg.norm(axis),
                    joint_name=joint.get('name'),
                )
                wheels.append(wheel)
                # The wheel's own subtree (rarely anything) is lumped into the wheel.
                walk(wheel, child, np.eye(4))
                frames[child] = child_in_body
            else:
                skipped.append(f'{joint.get("name")}: {kind} joint')

    walk(base, base_link, np.eye(4))
    return RobotModel(base, wheels, frames, skipped)


# ---------------------------------------------------------------------------------
# World: SDF static models -> geoms
# ---------------------------------------------------------------------------------


@dataclass
class WorldGeom:
    """A static world geom in the world frame."""

    name: str
    kind: str  # box | cylinder | sphere | plane
    size: Tuple[float, ...]
    pose: np.ndarray
    rgba: Tuple[float, float, float, float] = (0.7, 0.7, 0.7, 1.0)
    texture: Optional[str] = None  # PNG path (visual only)
    collide: bool = True


def sdf_material(
    visual: Optional[ET.Element],
) -> Tuple[Tuple[float, ...], Optional[str]]:
    """Diffuse colour and albedo texture (a model:// URI) of an SDF visual."""
    rgba = (0.7, 0.7, 0.7, 1.0)
    texture = None
    if visual is None:
        return rgba, texture
    material = visual.find('material')
    if material is not None:
        diffuse = material.findtext('diffuse')
        if diffuse:
            values = [float(v) for v in diffuse.split()]
            rgba = tuple((values + [1.0])[:4])
        albedo = material.find('.//albedo_map')
        if albedo is not None and albedo.text:
            texture = albedo.text.strip()
    return rgba, texture


def sdf_geom(name: str, geometry: ET.Element, pose: np.ndarray) -> Optional[WorldGeom]:
    """SDF geometry to a WorldGeom (without material)."""
    child = list(geometry)[0]
    if child.tag == 'box':
        return WorldGeom(
            name,
            'box',
            tuple(float(v) / 2 for v in child.findtext('size').split()),
            pose,
        )
    if child.tag == 'cylinder':
        return WorldGeom(
            name,
            'cylinder',
            (float(child.findtext('radius')), float(child.findtext('length')) / 2),
            pose,
        )
    if child.tag == 'sphere':
        return WorldGeom(name, 'sphere', (float(child.findtext('radius')),), pose)
    if child.tag == 'plane':
        size = [float(v) / 2 for v in (child.findtext('size') or '100 100').split()]
        normal = np.array(
            [float(v) for v in (child.findtext('normal') or '0 0 1').split()]
        )
        # MJCF planes face +z: rotate +z onto the SDF normal.
        z = normal / np.linalg.norm(normal)
        helper = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
        x = np.cross(helper, z)
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        r = np.eye(4)
        r[:3, :3] = np.column_stack([x, y, z])
        return WorldGeom(name, 'plane', (size[0], size[1], 0.05), pose @ r)
    return None


def build_world(
    sdf_path: str, resolve_uri: Callable[[str], Optional[str]]
) -> Tuple[List[WorldGeom], List[Tuple[np.ndarray, bool]], List[str]]:
    """
    Read an SDF world's static models.

    Returns the geoms, the lights ((direction, directional)), and notes on anything
    skipped.
    """
    root = ET.parse(sdf_path).getroot()
    world = root.find('world')
    geoms: List[WorldGeom] = []
    lights: List[Tuple[np.ndarray, bool]] = []
    notes: List[str] = []

    for light in world.findall('light'):
        direction = light.findtext('direction')
        lights.append(
            (
                np.array([float(v) for v in (direction or '0 0 -1').split()]),
                light.get('type') == 'directional',
            )
        )

    def add_model(
        model: ET.Element, model_pose: np.ndarray, prefix: str, model_dir: Optional[str]
    ) -> None:
        for link in model.findall('link'):
            link_pose = model_pose @ parse_sdf_pose(link.findtext('pose'))
            visuals = {v.get('name'): v for v in link.findall('visual')}
            for collision in link.findall('collision'):
                pose = link_pose @ parse_sdf_pose(collision.findtext('pose'))
                geom = sdf_geom(
                    f'{prefix}{collision.get("name")}', collision.find('geometry'), pose
                )
                if geom is None:
                    notes.append(
                        f'{prefix}{collision.get("name")}: unsupported collision geometry skipped'
                    )
                    continue
                # Colour from the visual with the same geometry, if any.
                match = next(iter(visuals.values()), None)
                geom.rgba, _ = sdf_material(match)
                geoms.append(geom)
            for visual in link.findall('visual'):
                rgba, texture = sdf_material(visual)
                if texture is None:
                    continue  # plain visuals are already represented by their collision
                pose = link_pose @ parse_sdf_pose(visual.findtext('pose'))
                geom = sdf_geom(
                    f'{prefix}{visual.get("name")}', visual.find('geometry'), pose
                )
                if geom is None:
                    continue
                geom.rgba = rgba
                geom.collide = False
                path = texture
                if texture.startswith('model://'):
                    path = resolve_uri(texture)
                elif model_dir and not os.path.isabs(texture):
                    path = os.path.join(model_dir, texture)
                if path and os.path.exists(path):
                    geom.texture = path
                else:
                    notes.append(
                        f'{prefix}{visual.get("name")}: texture {texture} not found'
                    )
                geoms.append(geom)

    for model in world.findall('model'):
        add_model(
            model, parse_sdf_pose(model.findtext('pose')), f'{model.get("name")}/', None
        )
    for include in world.findall('include'):
        uri = include.findtext('uri', '').strip()
        model_dir = resolve_uri(uri)
        if not model_dir or not os.path.exists(os.path.join(model_dir, 'model.sdf')):
            notes.append(f'include {uri}: not found or not a local model; skipped')
            continue
        model = ET.parse(os.path.join(model_dir, 'model.sdf')).getroot().find('model')
        name = include.findtext('name') or model.get('name')
        add_model(
            model, parse_sdf_pose(include.findtext('pose')), f'{name}/', model_dir
        )
    return geoms, lights, notes


# ---------------------------------------------------------------------------------
# MJCF assembly
# ---------------------------------------------------------------------------------


@dataclass
class SimParams:
    """Physics and sensor-placement settings for the generated model."""

    timestep: float = 0.002
    # Sliding friction 0.6: loose sand. (Torsional/rolling unused with condim 3.)
    wheel_friction: Tuple[float, float, float] = (0.6, 0.005, 0.0001)
    world_friction: Tuple[float, float, float] = (0.6, 0.005, 0.0001)
    wheel_kv: float = 5.0
    wheel_max_torque: float = 3.0  # ST3215: ~2.9 N m stall at 12 V
    wheel_damping: float = 0.01
    # Ratio of frictional to normal constraint impedance; 1 (soft) lets wheels slip
    # the way loose ground does.
    impratio: float = 1.0
    camera_link: str = 'oak_right_camera_optical_frame'
    camera_hfov: float = 1.274
    camera_width: int = 640
    camera_height: int = 480
    lidar_link: str = 'c1_lidar_frame'
    imu_link: str = 'imu_link'


def camera_fovy_deg(hfov: float, width: int, height: int) -> float:
    """Return the camera fovy (vertical, degrees) MuJoCo needs for a horizontal FOV."""
    return math.degrees(2.0 * math.atan(math.tan(hfov / 2.0) * height / width))


def geom_attributes(geom, robot: bool) -> Dict[str, str]:
    """Return the common MJCF attributes of a geom (pose, size, type)."""
    attrs = {
        'type': geom.kind,
        'size': fmt(geom.size),
        'pos': fmt(geom.pose[:3, 3]),
        'quat': fmt(matrix_to_quat(geom.pose[:3, :3])),
    }
    if robot:
        attrs.update(
            contype=str(ROBOT_CONTYPE),
            conaffinity=str(ROBOT_CONAFFINITY),
            group=str(ROBOT_GROUP),
        )
    return attrs


def build_mjcf(
    robot: RobotModel,
    world_geoms: List[WorldGeom],
    lights: List[Tuple[np.ndarray, bool]],
    spawn: Tuple[float, float, float, float],
    params: SimParams,
) -> str:
    """Assemble the MJCF XML for the robot (spawned at x, y, z, yaw) in the world."""
    mujoco = ET.Element('mujoco', model='perseus_lite')
    ET.SubElement(mujoco, 'compiler', angle='radian', autolimits='true')
    # Elliptic friction cones couple the two tangential directions, so a wheel that is
    # slipping along its rolling direction offers little sideways resistance -- what
    # lets a skid-steer turn in place. (Pyramidal cones limit each direction
    # independently, and a skid-steer longer than it is wide then cannot turn.)
    # implicitfast integrates actuator velocity terms implicitly: a velocity servo on a
    # light wheel (kv * dt / inertia >> 1) is unstable with the default Euler.
    ET.SubElement(
        mujoco,
        'option',
        timestep=fmt([params.timestep]),
        cone='elliptic',
        impratio=fmt([params.impratio]),
        integrator='implicitfast',
    )
    visual = ET.SubElement(mujoco, 'visual')
    ET.SubElement(
        visual,
        'global',
        offwidth=str(max(640, params.camera_width)),
        offheight=str(max(480, params.camera_height)),
    )
    ET.SubElement(visual, 'headlight', ambient='0.35 0.35 0.35', diffuse='0.4 0.4 0.4')
    asset = ET.SubElement(mujoco, 'asset')
    worldbody = ET.SubElement(mujoco, 'worldbody')

    for direction, directional in lights or [(np.array([-0.5, 0.1, -0.9]), True)]:
        ET.SubElement(
            worldbody,
            'light',
            directional='true' if directional else 'false',
            dir=fmt(direction / np.linalg.norm(direction)),
            pos='0 0 10',
            diffuse='0.7 0.7 0.7',
            castshadow='false',
        )

    for index, geom in enumerate(world_geoms):
        attrs = geom_attributes(geom, robot=False)
        attrs['name'] = geom.name.replace('/', '_')
        attrs['group'] = str(WORLD_GROUP)
        attrs['friction'] = fmt(params.world_friction)
        if geom.collide:
            attrs.update(contype=str(WORLD_CONTYPE), conaffinity=str(WORLD_CONAFFINITY))
        else:
            # Textured faces are visual only (and must not catch lidar rays twice).
            attrs.update(contype='0', conaffinity='0', group=str(VISUAL_GROUP))
        if geom.texture:
            texture_name = f'tex{index}'
            ET.SubElement(
                asset, 'texture', name=texture_name, type='2d', file=geom.texture
            )
            ET.SubElement(
                asset,
                'material',
                name=f'mat{index}',
                texture=texture_name,
                texuniform='false',
                texrepeat='1 1',
                emission='0.6',
                specular='0',
                rgba='1 1 1 1',
            )
            attrs['material'] = f'mat{index}'
        else:
            attrs['rgba'] = fmt(geom.rgba)
        ET.SubElement(worldbody, 'geom', attrs)

    x, y, z, yaw = spawn
    base = ET.SubElement(
        worldbody,
        'body',
        name=robot.base.name,
        pos=fmt([x, y, z]),
        quat=fmt(matrix_to_quat(rpy_matrix(0, 0, yaw))),
    )
    ET.SubElement(base, 'freejoint', name='root')
    i = robot.base.inertia
    ET.SubElement(
        base,
        'inertial',
        pos=fmt(robot.base.com),
        mass=fmt([robot.base.mass]),
        fullinertia=fmt([i[0, 0], i[1, 1], i[2, 2], i[0, 1], i[0, 2], i[1, 2]]),
    )
    for index, geom in enumerate(robot.base.geoms):
        attrs = geom_attributes(geom, robot=True)
        attrs.update(name=f'base_geom{index}', rgba='0.3 0.3 0.35 1')
        ET.SubElement(base, 'geom', attrs)
    # Sensor frames as sites.
    for frame_name in (params.lidar_link, params.imu_link, params.camera_link):
        if frame_name in robot.frames:
            pose = robot.frames[frame_name]
            ET.SubElement(
                base,
                'site',
                name=frame_name,
                pos=fmt(pose[:3, 3]),
                quat=fmt(matrix_to_quat(pose[:3, :3])),
                size='0.01',
                group='4',
            )
    if params.camera_link in robot.frames:
        # MuJoCo cameras look along -z with +y up; an optical frame looks along +z with
        # +y down: the same frame turned pi about x.
        pose = robot.frames[params.camera_link]
        flip = np.diag([1.0, -1.0, -1.0])
        ET.SubElement(
            base,
            'camera',
            name='oak',
            pos=fmt(pose[:3, 3]),
            quat=fmt(matrix_to_quat(pose[:3, :3] @ flip)),
            fovy=fmt(
                [
                    camera_fovy_deg(
                        params.camera_hfov, params.camera_width, params.camera_height
                    )
                ]
            ),
        )

    actuator = ET.SubElement(mujoco, 'actuator')
    for wheel in robot.wheels:
        body = ET.SubElement(
            base,
            'body',
            name=wheel.name,
            pos=fmt(wheel.pose[:3, 3]),
            quat=fmt(matrix_to_quat(wheel.pose[:3, :3])),
        )
        ET.SubElement(
            body,
            'joint',
            name=wheel.joint_name,
            type='hinge',
            axis=fmt(wheel.joint_axis),
            damping=fmt([params.wheel_damping]),
        )
        i = wheel.inertia
        ET.SubElement(
            body,
            'inertial',
            pos=fmt(wheel.com),
            mass=fmt([max(wheel.mass, 1e-3)]),
            fullinertia=fmt(
                [
                    max(i[0, 0], 1e-7),
                    max(i[1, 1], 1e-7),
                    max(i[2, 2], 1e-7),
                    i[0, 1],
                    i[0, 2],
                    i[1, 2],
                ]
            ),
        )
        for index, geom in enumerate(wheel.geoms):
            attrs = geom_attributes(geom, robot=True)
            attrs.update(
                name=f'{wheel.name}_geom{index}',
                friction=fmt(params.wheel_friction),
                rgba='0.1 0.1 0.1 1',
                condim='3',
            )
            ET.SubElement(body, 'geom', attrs)
        ET.SubElement(
            actuator,
            'velocity',
            name=f'{wheel.joint_name}_motor',
            joint=wheel.joint_name,
            kv=fmt([params.wheel_kv]),
            forcerange=fmt([-params.wheel_max_torque, params.wheel_max_torque]),
        )

    sensor = ET.SubElement(mujoco, 'sensor')
    if params.imu_link in robot.frames:
        ET.SubElement(sensor, 'gyro', name='imu_gyro', site=params.imu_link)
        ET.SubElement(sensor, 'accelerometer', name='imu_accel', site=params.imu_link)
    ET.indent(mujoco)
    return ET.tostring(mujoco, encoding='unicode')
