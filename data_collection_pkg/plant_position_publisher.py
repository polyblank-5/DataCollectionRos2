import rclpy
from rclpy.node import Node
import random
from std_msgs.msg import Float32MultiArray
import os 
import yaml


class PlantPositionPublisher(Node):

    def __init__(self):
        super().__init__('plant_position_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'topic', 10)
        package_share_directory = os.path.join(
            os.getenv('AMENT_PREFIX_PATH').split(':')[0], 'share/data_collection_pkg/config')
        config_file_path = os.path.join(package_share_directory, 'config.yaml')
    
        with open(config_file_path, 'r') as file:
                config = yaml.safe_load(file)

        # Extract constants
        constants = config['constants']
        self._FRAME_HEIGHT = constants['FRAME_HEIGHT']
        self._FRAME_WIDTH = constants['FRAME_WIDTH']
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float32MultiArray()
        y = round(random.uniform(0, self._FRAME_HEIGHT), 1)
        msg.data = [0.0, y]
        if random.choice([0, 1]) > 0.7:
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = PlantPositionPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()