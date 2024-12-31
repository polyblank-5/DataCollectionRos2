import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String  # Replace with the appropriate message types for your topics
import yaml
import os
from typing import List, Tuple
import math
from copy import deepcopy
import json

class RobotArea():
    def __init__(self,width:float, height:float, y:float):
        self.width: float = float(width)
        self.height: float = float(height)
        self.Y:float = float(y)

class PlantDataSubscriber(Node):
    def __init__(self):
        super().__init__('plant_data_subscriber')
        
        os.chdir('src/data_collection_pkg')
        #package_share_directory = os.path.join(    os.getenv('AMENT_PREFIX_PATH').split(':')[0], 'share/data_collection_pkg/config')
        config_file_path = os.path.join('config/', 'config.yaml')
    
        with open(config_file_path, 'r') as file:
                config = yaml.safe_load(file)

        constants = config['constants']
        
        self.plant_velocity:float = 0.0
        self.plant_rotation:float = 0.0
        self.plant_positions:List[List[float]] = []

        self._FRAME_HEIGHT = constants['FRAME_HEIGHT']
        self._FRAME_WIDTH = constants['FRAME_WIDTH']
        self._CELL_WIDTH = constants['SCREEN_WIDTH'] // int(constants['FRAME_WIDTH'] / constants['FRAME_DISCRETIZATION'])
        self._CELL_HEIGHT = constants['SCREEN_HEIGHT'] // int(constants['FRAME_WIDTH'] / constants['FRAME_DISCRETIZATION'])

        self._DETECTION_AREA:RobotArea = RobotArea(constants['DETECTION_AREA']['WIDTH'],constants['DETECTION_AREA']['HEIGHT'],constants['DETECTION_AREA']['Y'])
        self._LASER_AREA:RobotArea = RobotArea(constants['LASER_AREA']['WIDTH'],constants['LASER_AREA']['HEIGHT'],constants['LASER_AREA']['Y'])
        
        self.previous_time:float = 0.0
        self.publisher_ = self.create_publisher(String, 'plant_data_publisher', 10) # TODO increase publish frequency to 1000Hz maybe safe values and do a timer 
        # Subscription to the first topic
        self.plant_position_subscription = self.create_subscription(
            String,             # Replace 'String' with your topic's message type
            'plant_position_publisher',          # Replace 'topic_1' with the name of your first topic
            self.callback_plant_position_subscriber,    # Callback for the first topic
            10                  # QoS depth
        )
        self.plant_position_subscription  # Prevent unused variable warning
        
        # Subscription to the second topic
        self.plant_velocity_subscription = self.create_subscription(
            Float32MultiArray,             # Replace 'String' with your topic's message type
            'plant_velocity_publisher',          # Replace 'topic_2' with the name of your second topic
            self.callback_plant_velocity_subscriber,    # Callback for the second topic
            10                  # QoS depth
        )
        self.plant_velocity_subscription  # Prevent unused variable warning

    # Callback for the first topic
    def callback_plant_position_subscriber(self, msg):
        self.get_logger().info(f'Received from {self.plant_position_subscription.topic_name}: {msg.data}')
        # New positions are being loaded 
        new_postions = json.loads(msg.data)
        # IF the list is empty make them equal
        current_time:float = self.get_clock().now().nanoseconds / 1e9
        if self.previous_time == 0.0:
            self.previous_time = current_time
        t_d = current_time - self.previous_time
        if self.plant_positions == []:
            self.plant_positions = new_postions
        # If there are already plant positions existing 
        else:
            for i,positions in enumerate(self.plant_positions):
                self.plant_positions[i][0], self.plant_positions[i][1] = self.update_position(positions[0], positions[1],t_d)
            self.plant_positions = self.find_closest_points(self.plant_positions,new_postions) # TODO only do that when the point is in the border of the detection box
        self.previous_time = current_time
        publisher_msg = String()
        publisher_msg.data = json.dumps(self.plant_positions) # TODO delete postions which are out of bounds
        self.publisher_.publish(publisher_msg)
        self.get_logger().info(f'Publishing Data to {self.publisher_.topic_name}: {publisher_msg.data}')

    # Callback for the second topic
    def callback_plant_velocity_subscriber(self, msg):
        self.plant_velocity = float(msg.data[0])
        self.plant_rotation = float(msg.data[1])
        self.get_logger().info(f'Received from {self.plant_velocity_subscription.topic_name}: {msg.data}')

    def update_position(self,x:float,y:float, t_d :float) -> Tuple[float,float]:
        y -= self.plant_velocity * t_d
        vertical_adjustment = self.plant_velocity * math.sin(math.radians(self.plant_rotation))
        x += vertical_adjustment * t_d
        return (x,y)
    
    def find_closest_points(self,plant_positions:List[List[float]], new_plant_positions:List[List[float]])-> List[List[float]]:

        def euclidean_distance(p1, p2):
            """Calculate Euclidean distance between two points."""
            return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        
        for positions in plant_positions:
            if not new_plant_positions:
                break  # Stop if list2 is empty
            if positions[1] <= self._DETECTION_AREA.Y:
                if positions
                continue
            elif positions[0] >= self._FRAME_WIDTH or positions[1] <= 0:
                continue
            
            # Find the closest point in list2 to point1
            closest_point = min(new_plant_positions, key=lambda p2: euclidean_distance(positions, p2))
            
            # Remove the closest point from list2
            new_plant_positions.remove(closest_point)

            # Modify the value of the current point in list1 (e.g., set to [0.0, 0.0])
            positions[0] = positions[0]*1/3+closest_point[0]*2/3
            positions[1] = positions[1]*1/3+closest_point[1]*2/3
        
        for new_positions in new_plant_positions:
            plant_positions.append(new_positions)

        return plant_positions

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
