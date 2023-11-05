import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MoveShelfNode(Node):

    def __init__(self):
        super().__init__('elevator_node')
        self.publisher_down_ = self.create_publisher(String, '/elevator_down', 10)
        self.publisher_up_ = self.create_publisher(String, '/elevator_up', 10)
        self.elevator_msg = String()
        self.elevator_msg.data = ""   # this will always be blank string!
        return None

    def publish_message_down(self):
        self.publisher_down_.publish(self.elevator_msg)
        self.get_logger().info(f'[Down] Publish: "{self.elevator_msg.data}"')
        time.sleep(5.0)
        return None

    def publish_message_up(self):
        self.publisher_up_.publish(self.elevator_msg)
        self.get_logger().info(f'[Up] Publish: "{self.elevator_msg.data}"')
        time.sleep(5.0)
        return None


def main(args=None):
    rclpy.init(args=args)
    elevator_node = MoveShelfNode()
    elevator_node.publish_message_down()
    elevator_node.publish_message_up()
    elevator_node.destroy_node()
    rclpy.shutdown()
    return None


if __name__ == '__main__':
    main()