"""
Calibrate the MuJoCo robot's effective skid-steer wheel separation.

A skid-steer turns slower than its wheel speeds predict, because the wheels scrub
sideways. The robot's diff_drive_controller therefore uses an *effective* separation,
calibrated so a commanded yaw rate is achieved (0.8286 m on the real robot, whose true
track is 0.376 m). The simulator's contact model scrubs differently, so it needs its
own value: this tool finds it by driving the MuJoCo model directly (no ROS) and
bisecting until the achieved yaw rate matches the command.

    ros2 run perseus_lite_mujoco calibrate_turning

Put the result in perseus_lite_mujoco/config/mujoco_controllers.yaml. Re-run it after
changing the robot's mass, wheels or the friction parameters.
"""

import argparse
import math
import os

from perseus_lite_mujoco import model_builder as mb


def turn_ratio(
    model_xml, separation, yaw_rate, linear=0.0, wheel_radius=0.0725, seconds=3.0
):
    """Return achieved / commanded yaw rate for one steady command."""
    import mujoco

    model = mujoco.MjModel.from_xml_string(model_xml)
    data = mujoco.MjData(model)
    names = [model.actuator(i).name for i in range(model.nu)]
    left = [i for i, n in enumerate(names) if 'left' in n]
    right = [i for i, n in enumerate(names) if 'right' in n]
    for _ in range(int(0.5 / model.opt.timestep)):
        mujoco.mj_step(model, data)

    def yaw():
        w, x, y, z = data.qpos[3:7]
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    data.ctrl[left] = (linear - yaw_rate * separation / 2) / wheel_radius
    data.ctrl[right] = (linear + yaw_rate * separation / 2) / wheel_radius
    for _ in range(int(1.0 / model.opt.timestep)):  # settle into the turn
        mujoco.mj_step(model, data)
    # Accumulate step by step: a fast turn passes +/- pi within the window.
    turned = 0.0
    previous = yaw()
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)
        current = yaw()
        turned += (current - previous + math.pi) % (2 * math.pi) - math.pi
        previous = current
    return turned / seconds / yaw_rate


def calibrate(model_xml_for, yaw_rates=(0.3, 0.6), low=0.4, high=6.0, iterations=12):
    """Bisect the separation so the mean achieved/commanded ratio is 1."""
    for _ in range(iterations):
        mid = 0.5 * (low + high)
        xml = model_xml_for()
        ratio = sum(turn_ratio(xml, mid, w) for w in yaw_rates) / len(yaw_rates)
        if ratio < 1.0:
            low = mid
        else:
            high = mid
    return 0.5 * (low + high)


def main():
    """Command-line entry point."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument(
        '--friction',
        type=float,
        default=0.6,
        help='sliding friction (wheels and ground)',
    )
    parser.add_argument('--impratio', type=float, default=1.0)
    args = parser.parse_args()

    from ament_index_python.packages import get_package_share_directory

    from perseus_lite_mujoco.sim_node import xacro_robot_description

    xacro = os.path.join(
        get_package_share_directory('perseus_lite'), 'urdf', 'perseus_lite.urdf.xacro'
    )
    robot = mb.build_robot(xacro_robot_description(xacro))
    params = mb.SimParams(
        wheel_friction=(args.friction, 0.005, 0.0001),
        world_friction=(args.friction, 0.005, 0.0001),
        impratio=args.impratio,
    )
    ground = [
        mb.WorldGeom('ground', 'box', (20, 20, 0.05), mb.transform((0, 0, -0.05)))
    ]

    def model_xml():
        return mb.build_mjcf(robot, ground, [], (0, 0, 0.1, 0), params)

    mass = robot.base.mass + sum(w.mass for w in robot.wheels)
    print(
        f'robot {mass:.2f} kg, friction {args.friction}, impratio {args.impratio}: calibrating...'
    )
    separation = calibrate(model_xml)
    xml = model_xml()
    print(f'effective wheel_separation: {separation:.3f} m')
    for rate in (0.3, 0.6, 1.0):
        print(
            f'  check: yaw rate {rate:.1f} rad/s -> '
            f'achieved x{turn_ratio(xml, separation, rate):.2f}'
        )
    print(
        'Set diff_drive_base_controller.wheel_separation in '
        'perseus_lite_mujoco/config/mujoco_controllers.yaml'
    )


if __name__ == '__main__':
    main()
