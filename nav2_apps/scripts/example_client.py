import sys
import rclpy
from rclpy.node import Node
from services_quiz_srv.srv import Turn


class ClientAsync(Node):

    def __init__(self):
        super().__init__('services_quiz_client')
        self.client = self.create_client(Turn, 'turn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service Currently Unavailable, Waiting...')
        self.request = Turn.Request()
        return None

    def send_request(self):
        self.request.direction = "right"  # sys.argv[1]
        self.request.angular_velocity = 0.2  # sys.argv[2]
        self.request.time = 10  # sys.argv[3]
        self.get_logger().info("Calling /turn Service: d:%s w:%f t:%d" %
                               (self.request.direction, self.request.angular_velocity, self.request.time))
        self.future = self.client.call_async(self.request)
        return None


def main(args=None):
    rclpy.init(args=args)
    client = ClientAsync()
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info(
                    'Service call failed %r' % (e))
            else:
                client.get_logger().info(
                    'Response state %r' % (response.success,))
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# End of Code