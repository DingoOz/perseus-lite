# Copyright (c) 2026 Nigel Hungerford-Symes
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Package setup for perseus_lite_missions."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'perseus_lite_missions'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (
            os.path.join('share', package_name, 'missions', 'm00-cold-open'),
            glob('missions/m00-cold-open/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'missions', 'm00-cold-open', 'configs'),
            glob('missions/m00-cold-open/configs/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    # Declared via extras_require (not the deprecated `tests_require=`, see
    # ERRORS.md 2026-07-03) so colcon selects its pytest runner instead of the
    # setup.py/unittest fallback, which finds no tests (ERRORS.md 2026-07-07).
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Nigel Hungerford-Symes',
    maintainer_email='nigel.hungerfordsymes@gmail.com',
    description=(
        'Mission Zero (Selene Base Rescue) and follow-on scripted missions '
        'for the lite robot tutorial campaign.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'orchestrator_node = perseus_lite_missions.orchestrator_node:main',
            'map_quality_checker_node = '
            'perseus_lite_missions.checkers.map_quality_node:main',
            'nav2_params_checker_node = '
            'perseus_lite_missions.checkers.nav2_params_node:main',
            'goal_reached_checker_node = '
            'perseus_lite_missions.checkers.goal_reached_node:main',
        ],
    },
)
