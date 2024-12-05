import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Replace with the appropriate message types for your topics
import json
import pygame

class PlantDataVisualizer(Node):
    def __init__(self):
        super().__init__('plant_data_visualizer')
        
        # Subscription to the first topic
        self.plant_data_subscription = self.create_subscription(
            String,             # Replace 'String' with your topic's message type
            'plant_data_publisher',          # Replace 'topic_1' with the name of your first topic
            self.callback_plant_data_subscriber,    # Callback for the first topic
            10                  # QoS depth
        )
        self.plant_data_subscription  # Prevent unused variable warning
        


    # Callback for the first topic
    def callback_plant_data_subscriber(self, msg):
        self.get_logger().info(f'Received from {self.plant_data_subscription.topic_name}: {msg.data}')
        data = json.loads(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = PlantDataVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
