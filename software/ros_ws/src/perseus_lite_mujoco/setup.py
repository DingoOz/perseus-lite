from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'perseus_lite_mujoco'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    # extras_require, not the deprecated tests_require= (ERRORS.md 2026-07-03).
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='DingoOz',
    maintainer_email='1388693+DingoOz@users.noreply.github.com',
    description='MuJoCo simulation of Perseus Lite, bridged to ROS 2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mujoco_sim = perseus_lite_mujoco.sim_node:main',
            'calibrate_turning = perseus_lite_mujoco.calibration:main',
        ],
    },
)
