
#- CELL_WIDTH : SCREEN_WIDTH // int(FRAME_WIDTH / FRAME_DISCRETIZATION)
#  - CELL_HEIGHT : SCREEN_HEIGHT // int(FRAME_HEIGHT / FRAME_DISCRETIZATION)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  # Replace with the appropriate message types for your topics
import pygame
import yaml
import os

class PlantDataSubscriber(Node):
    def __init__(self):
        super().__init__('dual_subscriber')
        
        package_share_directory = os.path.join(
            os.getenv('AMENT_PREFIX_PATH').split(':')[0], 'share/data_collection_pkg/config')
        config_file_path = os.path.join(package_share_directory, 'config.yaml')
    
        with open(config_file_path, 'r') as file:
                config = yaml.safe_load(file)
        constants = config['constants']
        
        self.plant_velocity = float()
        self.plant_rotation = float()
        self._CELL_WIDTH = constants['SCREEN_WIDTH'] // int(constants['FRAME_WIDTH'] / constants['FRAME_DISCRETIZATION'])
        self._CELL_HEIGHT = constants['SCREEN_HEIGHT'] // int(constants['FRAME_WIDTH'] / constants['FRAME_DISCRETIZATION'])
        # Subscription to the first topic
        self.subscription_1 = self.create_subscription(
            Float32MultiArray,             # Replace 'String' with your topic's message type
            'plant_position_publisher',          # Replace 'topic_1' with the name of your first topic
            self.callback_plant_position_publisher,    # Callback for the first topic
            10                  # QoS depth
        )
        self.subscription_1  # Prevent unused variable warning
        
        # Subscription to the second topic
        self.subscription_2 = self.create_subscription(
            Float32MultiArray,             # Replace 'String' with your topic's message type
            'plant_velocity_publisher',          # Replace 'topic_2' with the name of your second topic
            self.callback_plant_velocity_publisher,    # Callback for the second topic
            10                  # QoS depth
        )
        self.subscription_2  # Prevent unused variable warning

    # Callback for the first topic
    def callback_plant_position_publisher(self, msg):
        
        self.get_logger().info(f'Received from topic_1: {msg.data}')

    # Callback for the second topic
    def callback_plant_velocity_publisher(self, msg):
        self.plant_velocity = msg.data[0]
        self.plant_rotation = msg.data[1]
        self.get_logger().info(f'Received from topic_2: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = PlantDataSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
