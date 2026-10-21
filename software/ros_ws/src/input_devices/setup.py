from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'input_devices'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*')),
        ),
    ],
    install_requires=['setuptools'],
    # Declared via extras_require (not the deprecated `tests_require=`, see
    # ERRORS.md 2026-07-03/07-07) so colcon selects its pytest runner instead of
    # the setup.py/unittest fallback, which finds no tests and exits 5.
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='james',
    maintainer_email='59348282+RandomSpaceship@users.noreply.github.com',
    description='Nodes which publish input device data to topics',
    license='MIT',
    entry_points={
        'console_scripts': [
            'xbox_controller = input_devices.xbox_controller:main',
        ],
    },
)
