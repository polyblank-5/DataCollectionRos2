import rclpy
from rclpy.node import Node
import random
from std_msgs.msg import Float32MultiArray
import os 
import yaml


class PlantVelocityPublisher(Node):

    def __init__(self):
        super().__init__('plant_velocity_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'plant_velocity_publisher', 10)
        os.chdir('src/data_collection_pkg')
        #package_share_directory = os.path.join(    os.getenv('AMENT_PREFIX_PATH').split(':')[0], 'share/data_collection_pkg/config')
        config_file_path = os.path.join('config/', 'config.yaml')
    
        with open(config_file_path, 'r') as file:
                config = yaml.safe_load(file)

        # Extract constants
        constants = config['constants']
        self._SPEED = constants['SPEED']
        self._ROTATION_ANGLE = constants['ROTATION_ANGLE']
        timer_period = constants['TIMER_PERIOD']['VELOCITY_TIMEOUT']
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [self._SPEED, self._ROTATION_ANGLE]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = PlantVelocityPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()