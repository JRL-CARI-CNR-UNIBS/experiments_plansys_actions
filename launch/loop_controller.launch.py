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

from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    package_dir = get_package_share_directory('experiments_plansys_actions')

    
    experiments_controller_param_path = os.path.join(
        package_dir,
        'config', 
        'experiments_controller_param.yaml'
        )
        
    with open(experiments_controller_param_path, 'r') as file:
        config_file = yaml.safe_load(file)
        experiments_controller_param = config_file['experiments_loop_controller_node']['ros__parameters']

    problem_path = os.path.join(
        package_dir,
        'pddl',
        experiments_controller_param['problem_name']
    )

    with open(problem_path, 'r') as file:
        problem = yaml.safe_load(file)

    experiments_controller_param["problem"] = problem

    controller_cmd = Node(
        package='experiments_plansys_actions',
        executable='experiments_loop_controller_node',
        name='experiments_loop_controller_node',
        output='screen',
        parameters=[experiments_controller_param],
    )
    
    ld = LaunchDescription()
    ld.add_action(controller_cmd)

    return ld
