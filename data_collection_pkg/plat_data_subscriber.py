
#- CELL_WIDTH : SCREEN_WIDTH // int(FRAME_WIDTH / FRAME_DISCRETIZATION)
#  - CELL_HEIGHT : SCREEN_HEIGHT // int(FRAME_HEIGHT / FRAME_DISCRETIZATION)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  # Replace with the appropriate message types for your topics

class DualSubscriber(Node):
    def __init__(self):
        super().__init__('dual_subscriber')
        
        # Subscription to the first topic
        self.subscription_1 = self.create_subscription(
            Float32MultiArray,             # Replace 'String' with your topic's message type
            'topic_1',          # Replace 'topic_1' with the name of your first topic
            self.callback_1,    # Callback for the first topic
            10                  # QoS depth
        )
        self.subscription_1  # Prevent unused variable warning
        
        # Subscription to the second topic
        self.subscription_2 = self.create_subscription(
            Float32MultiArray,             # Replace 'String' with your topic's message type
            'topic_2',          # Replace 'topic_2' with the name of your second topic
            self.callback_2,    # Callback for the second topic
            10                  # QoS depth
        )
        self.subscription_2  # Prevent unused variable warning

    # Callback for the first topic
    def callback_1(self, msg):
        self.get_logger().info(f'Received from topic_1: {msg.data}')

    # Callback for the second topic
    def callback_2(self, msg):
        self.get_logger().info(f'Received from topic_2: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = DualSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
