# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    robot_xacro = os.path.join(get_package_share_directory('vehicle_simulator'),
                              'urdf',
                              'robot.urdf.xacro')
                    
    lidar_xacro = os.path.join(get_package_share_directory('vehicle_simulator'),
                            'urdf',
                            'lidar.urdf.xacro')

    camera_xacro = os.path.join(get_package_share_directory('vehicle_simulator'),
                            'urdf',
                            'camera.urdf.xacro')

    spawn_robot = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-file', robot_xacro,
                                   '-entity', 'robot'],
                        output='screen')

    spawn_lidar = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-file', lidar_xacro,
                                '-entity', 'lidar'],
                    output='screen')
            
    spawn_camera = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-file', camera_xacro,
                                '-entity', 'camera'],
                    output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(get_package_share_directory('vehicle_simulator'), 'world', 'garage.world')],
            description='SDF world file',
        ),
        gazebo,
        spawn_robot,
        spawn_lidar,
        spawn_camera
    ])