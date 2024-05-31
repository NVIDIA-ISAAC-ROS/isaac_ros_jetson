# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#
# SPDX-License-Identifier: MIT

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_isaac_ros_jetson_stats = get_package_share_directory('isaac_ros_jetson_stats')

    config_path = LaunchConfiguration('config_path')

    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(pkg_isaac_ros_jetson_stats, 'config', 'jtop.yaml'),
        description='Diagnostic aggregator path')

    jtop_node = Node(
        package='isaac_ros_jetson_stats',
        name='jtop',
        executable='jtop',
        output='screen'
    )

    diagnostic_aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        parameters=[config_path],
        output='screen'
    )

    return launch.LaunchDescription([declare_config_path_cmd, jtop_node, diagnostic_aggregator])
# EOF
