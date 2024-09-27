# Copyright 2019 Intelligent Robotics Lab
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
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    package_dir = get_package_share_directory('experiments_plansys_actions')

    declare_human_scene_config_cmd = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(
            package_dir,
            'config',
            'humans_scene_manager.yaml'
        ),
        description='Main path of the folder containing subfolders with the exported data')
    print(os.path.join(
            package_dir,
            'config',
            'human_scene_manager.yaml'
        ))
    human_scene_manager_cmd = Node(
        package='experiments_plansys_actions',
        executable='humans_scene_manager.py',
        name='humans_scene_manager_node',
        output='screen',
        parameters=[LaunchConfiguration('config_path')],
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_human_scene_config_cmd)
    ld.add_action(human_scene_manager_cmd)

    return ld