import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
from tf2_ros.buffer_interface import BufferInterface
import math
import tf2_py as tf2
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class FollowCartFrame(Node):

    def __init__(self):
        super().__init__('robot_chase')

        # Crear el publisher
        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(Twist, 'robot/cmd_vel', qos_profile)

        # Crear el buffer y el listener de transformaciones
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        # Crear el timer
        self.timer_ = self.create_timer(0.2, self.send_command)

        # Inicializar la velocidad
        self.cmd_vel = Twist()

    def send_command(self):
        try:
            # Obtener la transformación entre "rick/base_link" y "morty/base_link"
            transform = self.tf_buffer_.lookup_transform('robot_base_footprint', 'robot_cart_laser', rclpy.time.Time())

            # Calcular la distancia y el error angular entre los frames de referencia
            error_distance = math.sqrt(pow(transform.transform.translation.x, 2) +
                                       pow(transform.transform.translation.y, 2))
            error_yaw = math.atan2(transform.transform.translation.y,
                                   transform.transform.translation.x)

            # Definir la velocidad angular y lineal
            kp_yaw = 0.5
            kp_distance = 0.2
            angular_vel = kp_yaw * error_yaw
            linear_vel = kp_distance * error_distance

            # Verificar si Rick ya alcanzó a Morty
            if error_distance <= 0.20:
                self.get_logger().info('Reached!')
                self.cmd_vel.angular.z = 0
                self.cmd_vel.linear.x = 0
                self.timer_.cancel()
            else:
                # Sigue a Morty
                self.cmd_vel.angular.z = angular_vel
                self.cmd_vel.linear.x = linear_vel

            # Publicar la velocidad
            self.publisher_.publish(self.cmd_vel)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn('TF2 error: {}'.format(str(e)))

def main(args=None):
    rclpy.init(args=args)

    robot_chase = FollowCartFrame()
    rclpy.spin(robot_chase)
    robot_chase.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
