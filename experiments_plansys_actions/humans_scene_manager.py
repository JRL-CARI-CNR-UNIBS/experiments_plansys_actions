#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
import os

from std_srvs.srv import Trigger

class HumanSceneManager(Node):

    def __init__(self):
        super().__init__('humans_scene_manager_node')

        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')

        if not self.spawn_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/spawn_entity service not available, exiting...')
            return
        if not self.delete_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/delete_cli service not available, exiting...')
            return

        self.declare_parameter('people_groups', [''])

        # sdf_file_path = self.get_parameter('sdf_file_path').get_parameter_value().string_value
        people_groups = self.get_parameter('people_groups').get_parameter_value().string_array_value
        self.get_logger().info(f'People groups: {people_groups}')
        if not people_groups:
            self.get_logger().error('No people groups provided')
            return
        
        self.people_groups_request = dict()
        for group in people_groups:
            self.declare_parameter(group, [''])
            people = self.get_parameter(group).get_parameter_value().string_array_value
            
            self.people_groups_request[group] = []
            for person in people:
                self.declare_parameter(f'{person}.sdf_file_path', '')
                self.declare_parameter(f'{person}.position', [0.0, 0.0, 0.0])
                self.declare_parameter(f'{person}.orientation', [0.0, 0.0, 0.0, 1.0])
                self.declare_parameter(f'{person}.reference_france', 'world')

                sdf_file_path = self.get_parameter(f'{person}.sdf_file_path').get_parameter_value().string_value
                if not sdf_file_path:
                    self.get_logger().warning(f'No SDF file path provided for {person}, is not added')
                    continue
                position = self.get_parameter(f'{person}.position').get_parameter_value().double_array_value
                orientation = self.get_parameter(f'{person}.orientation').get_parameter_value().double_array_value
                reference_france = self.get_parameter(f'{person}.reference_france').get_parameter_value().string_value

                # Check if the SDF file exists
                if not os.path.isfile(sdf_file_path):
                    self.get_logger().error(f"SDF file not found: {sdf_file_path}")
                    return
                try:
                    # Read the content of the SDF file
                    with open(sdf_file_path, 'r') as sdf_file:
                        sdf_content = sdf_file.read()
                except Exception as e:
                    self.get_logger().error(f"An error occurred while reading the SDF file: {str(e)}")
                    return
                # Check orientation
                if len(orientation) != 4:
                    self.get_logger().error(f"Invalid orientation: {orientation}")
                    return
                norm = sum([x**2 for x in orientation])
                if abs(norm - 1) > 1e-6:
                    self.get_logger().error(f"Invalid orientation: {orientation}")
                    return
                # check position
                if len(position) != 3:
                    self.get_logger().error(f"Invalid position: {position}")
                    return
                
                # Prepare the request to spawn the entity
                spawn_request = SpawnEntity.Request()
                spawn_request.name = f'{person}_{group}'  # Name of the model in Gazebo
                spawn_request.xml = sdf_content  # The entire content of the SDF file
                spawn_request.robot_namespace = ''  # You can specify a namespace if needed
                spawn_request.reference_frame = reference_france  # Use 'world' as the reference frame
                spawn_request.initial_pose.position.x = position[0]  # Initial position of the model
                spawn_request.initial_pose.position.y = position[1]
                spawn_request.initial_pose.position.z = position[2]
                spawn_request.initial_pose.orientation.x = orientation[0]  # Initial orientation of the model
                spawn_request.initial_pose.orientation.y = orientation[1]
                spawn_request.initial_pose.orientation.z = orientation[2]
                spawn_request.initial_pose.orientation.w = orientation[3]
                self.people_groups_request[group].append(spawn_request)
        
        # for request in self.people_groups_request.values():
        #     for req in request:
        #         self.spawn_future = self.spawn_cli.call_async(req)
        #         self.spawn_future.add_done_callback(self.callback_spawn)

        self.create_service(Trigger, '/delete_all_entities', self.delete_model)
        self.create_service(Trigger, '/spawn_all_entities', self.spawn_model)

    def spawn_model(self, request, response):
        # Create a client to the /spawn_entity service
        
        if not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/spawn_entity service not available, waiting...')
            return
        for group in self.people_groups_request.keys():
            for req in self.people_groups_request[group]:
                self.spawn_future = self.spawn_cli.call_async(req)
                self.spawn_future.add_done_callback(self.callback_spawn)
        response.success = True
        response.message = 'All models successfully spawned!'
        return response

    def callback_spawn(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Model successfully spawned!')
            else:
                self.get_logger().error(f'Failed to spawn model: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'An error occurred while spawning: {str(e)}')

    def delete_model(self,request, response):
        # Create a client to the /delete_entity service
        
        if not self.delete_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/delete_entity service not available, waiting...')
            return
        for group in self.people_groups_request.keys():
            for req in self.people_groups_request[group]:
                self.get_logger().info(f'Deleting model: {req.name}')
                model_name = req.name

                delete_request = DeleteEntity.Request()
                delete_request.name = model_name  # Name of the model to be deleted

                # Asynchronous service call to delete the entity
                self.delete_future = self.delete_cli.call_async(delete_request)
                self.delete_future.add_done_callback(self.callback_delete)
        response.success = True
        response.message = 'All models successfully deleted!'
        return response

    def callback_delete(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Model successfully deleted!')
            else:
                self.get_logger().error(f'Failed to delete model: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'An error occurred while deleting: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = HumanSceneManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
