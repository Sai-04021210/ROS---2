import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        """
        Initialize the SimpleSubscriber node.

        The node will subscribe to the 'topic_name' topic and print
        received messages to the console.
        """
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10  # QoS profile depth
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        """
        Callback function to handle received messages.
        
        :param msg: The received message
        """
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()