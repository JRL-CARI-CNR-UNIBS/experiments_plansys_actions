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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the launch directory
    package_dir = get_package_share_directory('experiments_plansys_actions')

    # Declare the launch argument
    declare_use_auction_mechanism_arg = DeclareLaunchArgument(
        'use_auction_mechanism',
        default_value='false',
        description='Flag to enable or disable auction mechanism'
    )

    use_auction_mechanism = LaunchConfiguration('use_auction_mechanism')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={'model_file': package_dir + '/pddl/hospital_domain.pddl',
                          'use_auction_mechanism': use_auction_mechanism}.items()
        )
    
    actions_param_path = os.path.join(
        package_dir,
        'config',
        'actions_params.yaml'
        )
    
    waypoints_path = os.path.join(
        package_dir,
        'config',
        'waypoints.yaml'
        )
    
    with open(waypoints_path, 'r') as file:
        config_file = yaml.safe_load(file)
        waypoints_config_params = config_file['waypoints_broadcaster_node']['ros__parameters']
        clean_waypoints_config_params = config_file['clean_waypoints']['ros__parameters']  
        # controller_node_param = config_file['controller_node']['ros__parameters']

    with open(actions_param_path, 'r') as file:
        actions_config_params = yaml.safe_load(file)
    
    robot1_move_action_params = actions_config_params['move_action_node_robot1']['ros__parameters']
    robot2_move_action_params = actions_config_params['move_action_node_robot2']['ros__parameters']
    robot1_patrol_action_paramas = actions_config_params['patrol_action_node_robot1']['ros__parameters']
    robot2_patrol_action_paramas = actions_config_params['patrol_action_node_robot2']['ros__parameters']

    robot1_clean_action_paramas = actions_config_params['clean_action_node_robot1']['ros__parameters']
    robot2_clean_action_paramas = actions_config_params['clean_action_node_robot2']['ros__parameters']

    robot1_move_cmd = Node(
        package='experiments_plansys_actions',
        executable='move_action_node',
        name='move_action_node_robot1',
        output='screen',
        parameters=[waypoints_config_params, 
                    robot1_move_action_params],
        # remappings=[('/tf_static', 'robot1/tf_static')]
    )
    robot2_move_cmd = Node(
        package='experiments_plansys_actions',
        executable='move_action_node',
        name='move_action_node_robot2',
        output='screen',
        parameters=[waypoints_config_params, 
                    robot2_move_action_params],
        # remappings=[('/tf_static', 'robot2/tf_static')]
    )

    # robot2_move_cmd = Node(
    #     package='experiments_plansys_actions',
    #     executable='move_action_node',
    #     name='robot2_move_action_node',
    #     output='screen',
    #     parameters=[waypoints_config_params, 
    #                 move_action_paramas],
    #     remappings=[('/tf_static', 'robot1/tf_static')]
    # )

    robot1_patrol_cmd = Node(
        package='experiments_plansys_actions',
        executable='patrol_action_node',
        name='patrol_action_node_robot1',
        output='screen',
        parameters=[robot1_patrol_action_paramas])
    robot2_patrol_cmd = Node(
        package='experiments_plansys_actions',
        executable='patrol_action_node',
        name='patrol_action_node_robot2',
        output='screen',
        parameters=[robot2_patrol_action_paramas])

    robot1_clean_cmd = Node(
        package='experiments_plansys_actions',
        executable='clean_action_node',
        name='clean_action_node_robot1',
        output='screen',
        parameters=[clean_waypoints_config_params, 
                    robot1_clean_action_paramas],
        # namespace='robot1',
        # remappings=[('/tf', 'tf'),
        #             ('/tf_static', 'tf_static')]
    )
    robot2_clean_cmd = Node(
        package='experiments_plansys_actions',
        executable='clean_action_node',
        name='clean_action_node_robot2',
        output='screen',
        parameters=[clean_waypoints_config_params, 
                    robot2_clean_action_paramas],
        # namespace='robot1',
        # remappings=[('/tf', 'tf'),
        #             ('/tf_static', 'tf_static')]
    )


    # patrol_cmd = Node(
    #     package='plansys2_learning_patrolling',
    #     executable='patrol_action_node',
    #     name='patrol_action_node',
    #     output='screen',
    #     parameters=[waypoints_config_params])

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_use_auction_mechanism_arg)

    # Declare the launch options
    ld.add_action(plansys2_cmd)
    ld.add_action(robot1_move_cmd)
    ld.add_action(robot2_move_cmd)
    ld.add_action(robot1_patrol_cmd)
    ld.add_action(robot2_patrol_cmd)
    ld.add_action(robot1_clean_cmd)
    ld.add_action(robot2_clean_cmd)

    # ld.add_action(patrol_cmd)
    # ld.add_action(clean_cmd)
    # ld.add_action(nav2_cmd)

    # ld.add_action(move_cmd)
    # # ld.add_action(move_cmd1)
    # ld.add_action(patrol_cmd)

    return ld
