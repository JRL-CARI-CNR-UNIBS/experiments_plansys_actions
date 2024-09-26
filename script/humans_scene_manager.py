import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
import os

class HumanSceneManager(Node):

    def __init__(self):
        super().__init__('human_scene_manager')

        # Declare parameter for the SDF file path
        self.declare_parameter('sdf_file_path', '/home/holden/projects/plansys_ws/src/multi_robot/models/human/human.sdf')
        sdf_file_path = self.get_parameter('sdf_file_path').get_parameter_value().string_value

        # Check if the SDF file exists
        if not os.path.isfile(sdf_file_path):
            self.get_logger().error(f"SDF file not found: {sdf_file_path}")
            return

        # Read the content of the SDF file
        with open(sdf_file_path, 'r') as sdf_file:
            sdf_content = sdf_file.read()

        # Name of the model to spawn and delete
        self.model_name = 'my_entity'

        # Create a client to the /spawn_entity service
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('/spawn_entity service not available, waiting...')

        # Configure the request to spawn the entity
        spawn_request = SpawnEntity.Request()
        spawn_request.name = self.model_name  # Name of the model in Gazebo
        spawn_request.xml = sdf_content  # The entire content of the SDF file
        spawn_request.robot_namespace = ''  # You can specify a namespace if needed
        spawn_request.reference_frame = 'world'  # Use 'world' as the reference frame

        # Asynchronous service call to spawn the entity
        self.spawn_future = self.spawn_cli.call_async(spawn_request)
        self.spawn_future.add_done_callback(self.callback_spawn)

    def callback_spawn(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Model successfully spawned!')
                # Now call the service to delete the model
                # self.delete_model()
            else:
                self.get_logger().error(f'Failed to spawn model: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'An error occurred while spawning: {str(e)}')

    def delete_model(self):
        # Create a client to the /delete_entity service
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')
        while not self.delete_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('/delete_entity service not available, waiting...')

        # Configure the request to delete the entity
        delete_request = DeleteEntity.Request()
        delete_request.name = self.model_name  # Name of the model to be deleted

        # Asynchronous service call to delete the entity
        self.delete_future = self.delete_cli.call_async(delete_request)
        self.delete_future.add_done_callback(self.callback_delete)

    def callback_delete(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Model "{self.model_name}" successfully deleted!')
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
