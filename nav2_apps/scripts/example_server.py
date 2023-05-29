import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from services_quiz_srv.srv import Turn

class Service(Node):

    def __init__(self):
        super().__init__('services_quiz_server')
        self.service = self.create_service(
            Turn, 'turn', self.turn_service_callback)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist_cmd = Twist()
        return None

    def turn_service_callback(self, request, response):
        direction = request.direction.lower()
        ang_vel = float(request.angular_velocity)
        turn_time = int(request.time)
        self.get_logger().info("Executing /turn Service: d:%s w:%f t:%d" %
                               (direction, ang_vel, turn_time))
        if (direction == "left"):
            self.twist_cmd.linear.x = 0.0
            self.twist_cmd.angular.z = ang_vel
            self.cmd_vel_pub.publish(self.twist_cmd)
            self.get_logger().info("Robot Spinning Left...")
            time.sleep(turn_time)
            self.twist_cmd.linear.x = 0.0
            self.twist_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist_cmd)
            self.get_logger().info("Done")
            response.success = True
        elif (direction == "right"):
            self.twist_cmd.linear.x = 0.0
            self.twist_cmd.angular.z = -ang_vel
            self.cmd_vel_pub.publish(self.twist_cmd)
            self.get_logger().info("Robot Spinning Right...")
            time.sleep(turn_time)
            self.twist_cmd.linear.x = 0.0
            self.twist_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist_cmd)
            self.get_logger().info("Done")
            response.success = True
        else:
            self.twist_cmd.angular.z = 0.0
            self.get_logger().info("Robot Stopped")
            response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    service = Service()
    rclpy.spin(service)
    rclpy.shutdown()
    return None

if __name__ == '__main__':
    main()

# End of Code
