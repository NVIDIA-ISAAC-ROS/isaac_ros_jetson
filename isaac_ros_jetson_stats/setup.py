# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

from glob import glob
import importlib.util
from os import path
from pathlib import Path
import sys

from ament_index_python.packages import get_resource
from setuptools import setup

ISAAC_ROS_COMMON_PATH = get_resource(
    'isaac_ros_common_scripts_path',
    'isaac_ros_common'
)[0]

ISAAC_ROS_COMMON_VERSION_INFO = Path(ISAAC_ROS_COMMON_PATH) / 'isaac_ros_common-version-info.py'

spec = importlib.util.spec_from_file_location(
    'isaac_ros_common_version_info',
    ISAAC_ROS_COMMON_VERSION_INFO
)

isaac_ros_common_version_info = importlib.util.module_from_spec(spec)
sys.modules['isaac_ros_common_version_info'] = isaac_ros_common_version_info
spec.loader.exec_module(isaac_ros_common_version_info)

from isaac_ros_common_version_info import GenerateVersionInfoCommand  # noqa: E402, I100

package_name = 'isaac_ros_jetson_stats'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (path.join('share', package_name,  'config'), glob('config/*.yaml')),
        (path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Isaac ROS Maintainers',
    maintainer_email='isaac-ros-maintainers@nvidia.com',
    description='Isaac ROS Jetson. Set of packages to control your jetson from your board',
    license='MIT',
    extras_require={
        'test': [
            'pytest'
        ]
    },
    entry_points={
        'console_scripts': [
            'jtop = isaac_ros_jetson_stats.ros2_jtop_node:main',
        ],
    },
    cmdclass={
        'build_py': GenerateVersionInfoCommand,
    },
)
# EOF
