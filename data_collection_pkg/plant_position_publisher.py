import rclpy
from rclpy.node import Node
import random
from std_msgs.msg import String
import os 
import yaml
import math
from typing import List,Tuple
import json


class PlantPositionPublisher(Node):

    def __init__(self):
        super().__init__('plant_position_publisher')
        self.publisher_ = self.create_publisher(String, 'plant_position_publisher', 10)
        os.chdir('src/data_collection_pkg')
        #package_share_directory = os.path.join(    os.getenv('AMENT_PREFIX_PATH').split(':')[0], 'share/data_collection_pkg/config')
        config_file_path = os.path.join('config/', 'config.yaml')
        with open(config_file_path, 'r') as file:
                config = yaml.safe_load(file)

        # Extract constants
        constants = config['constants']
        self._FRAME_HEIGHT = constants['FRAME_HEIGHT']
        self._FRAME_WIDTH = constants['FRAME_WIDTH']
        self._SPEED = float(constants['SPEED'])
        self._ROTATION_ANGLE = constants["ROTATION_ANGLE"]
        timer_period = constants['TIMER_PERIOD']['POSITION_TIMEOUT']
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.plant_positions:List[List[float,float]] = []

    def update_position(self,x:float,y:float) -> Tuple[float,float]:
        x = x + self._SPEED
        vertical_adjustment = self._SPEED * math.sin(math.radians(self._ROTATION_ANGLE))
        y += vertical_adjustment
        return (x,y)
    """
    callback is being called 
    all of the members in the list are beeing moved foreward
    delete all members that exceed the limits  
    if new data is being generated it is appended to the list
    """
    def timer_callback(self):
        msg = String()
        if len(self.plant_positions) > 1:
            for i,positions in enumerate(self.plant_positions):
                self.plant_positions[i] = self.update_position(positions[0],positions[1])

            for i in range(len(self.plant_positions)):
                if self.plant_positions[i][0] > self._FRAME_WIDTH or self.plant_positions[i][1] > self._FRAME_HEIGHT:
                    self.plant_positions.pop(i)
                else:
                    break

        if random.choice([0, 1]) > 0.7:
            y = round(random.uniform(0, self._FRAME_HEIGHT), 1)
            new_data = [0.0,y]
            self.plant_positions.append(new_data)
            #msg.data = self.plant_positions

        msg.data = json.dumps(self.plant_positions)
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