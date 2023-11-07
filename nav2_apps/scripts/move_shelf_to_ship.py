#! /usr/bin/env python3

import time
from copy import deepcopy
import threading
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from attach_shelf.srv import GoToLoading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from geometry_msgs.msg import Polygon,Point32, Twist
from std_msgs.msg import Empty
from math import cos, sin, pi
from std_msgs.msg import String
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros import TransformListener, Buffer
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
import math

####################
request_loading_position = 'loading_position'
request_destination = 'shipping_position'
request_init_position = 'init_position'
####################

class FollowCartFrame(Node):

    def __init__(self):
        super().__init__('FollowFrame node')

        # Crear el publisher
        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(Twist, 'robot/cmd_vel', qos_profile)

        # Crear el buffer y el listener de transformaciones
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.timer_ = self.create_timer(0.2, self.send_command)
        self.timer_on = False
        self.cmd_vel = Twist()

    def send_command(self):

        if(self.timer_on):

            try:
                # Obtener la transformación entre "rick/base_link" y "morty/base_link"
                transform = self.tf_buffer_.lookup_transform('robot_base_footprint', 'robot_cart_frame', rclpy.time.Time())

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
                    self.timer_on = False
                    self.timer_.cancel()
                else:
                    # Sigue a Morty
                    self.cmd_vel.angular.z = angular_vel
                    self.cmd_vel.linear.x = linear_vel

                # Publicar la velocidad
                self.publisher_.publish(self.cmd_vel)

            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn('TF2 error: {}'.format(str(e)))

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

class FootPrintPublisher(Node):

    def __init__(self):
        super().__init__('FootPrintPublisher node')

        self.pub_local = self.navigator.create_publisher(Polygon, '/local_costmap/footprint', 10)
        self.pub_global = self.navigator.create_publisher(Polygon, '/global_costmap/footprint', 10)

        self.publisher_down_ = self.create_publisher(String, '/elevator_down', 10)
        self.publisher_up_ = self.create_publisher(String, '/elevator_up', 10)

        self.elevator_msg = String()
        self.elevator_msg.data = ""   # this will always be blank string!
        return None

    def publish_footprint_shelf(self):

        points = [Point32(0.5, 0.4, 0.0), Point32(0.5, -0.4, 0.0),
                Point32(-0.5, -0.4, 0.0), Point32(-0.5, 0.4, 0.0)]

        footprint = Polygon(points=points)   

        self.pub_local.publish(footprint)
        self.pub_global.publish(footprint)

    def publish_init_footprint(self):

        points = [Point32(0.25, 0.25, 0.0), Point32(0.25, -0.25, 0.0),
                Point32(-0.25, -0.25, 0.0), Point32(-0.25, 0.25, 0.0)]

        footprint = Polygon(points=points)

        self.pub_local.publish(footprint)
        self.pub_global.publish(footprint)

class Navigation(Node):

    def __init__(self):
        super().__init__('Navigator node')

        self.state_nav= 0
        self.navigator = BasicNavigator()
        self.client_cb_group = MutuallyExclusiveCallbackGroup()

        self.RB1_position = {

            "init_position" :   [0.031, -0.023, -0.000,
                                -0.000, -0.000, -0.000, 1.000],
            "loading_position": [5.600, -0.272, 0.000,
                                -0.000, -0.000, -0.679, 0.734],

            "shipping_position":[0.281, -3.0, 0.000,
                                0.000, 0.000, -0.689, 0.725]
        }

        #Service Approach Client for SIMULATION Purposes
        # self.cli = self.navigator.create_client(GoToLoading, '/approach_shelf',callback_group=client_cb_group)
        # while not cli.wait_for_service(timeout_sec=1.0):
        #     self.navigator.get_logger().info('service not available, waiting again...')

    def set_pos_init(self):

        # Set your demo's initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.RB1_position[request_init_position][0]
        initial_pose.pose.position.y = self.RB1_position[request_init_position][1]
        initial_pose.pose.position.z = self.RB1_position[request_init_position][2]
        initial_pose.pose.orientation.x = self.RB1_position[request_init_position][3]
        initial_pose.pose.orientation.y = self.RB1_position[request_init_position][4]
        initial_pose.pose.orientation.z = self.RB1_position[request_init_position][5]
        initial_pose.pose.orientation.w = self.RB1_position[request_init_position][6]

        self.navigator.setInitialPose(initial_pose)

        # Wait for navigation to activate fully
        self.navigator.waitUntilNav2Active()

        return None

    def call_service(self):

        # once arrived at loading position step to execute the approach service
        self.navigator.req = GoToLoading.Request()
        self.navigator.req.attach_to_shelf = True
        self.navigator.future = self.cli.call_async(self.navigator.req)
        rclpy.spin_until_future_complete(self.navigator, self.navigator.future)
        status = self.navigator.future.result()
        if status != True:
            #navigator.error('failed service')
            self.state_nav=3
        else:
            self.navigator.info('successful service!')
            self.state_nav=3

    def go_pose(self, key_dic, state_num):

        shelf_item_pose = PoseStamped()
        shelf_item_pose.header.frame_id = 'map'
        shelf_item_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        shelf_item_pose.pose.position.x = self.RB1_position[key_dic][0]
        shelf_item_pose.pose.position.y = self.RB1_position[key_dic][1]
        shelf_item_pose.pose.position.z = self.RB1_position[key_dic][2]
        shelf_item_pose.pose.orientation.x = self.RB1_position[key_dic][3]
        shelf_item_pose.pose.orientation.y = self.RB1_position[key_dic][4]
        shelf_item_pose.pose.orientation.z = self.RB1_position[key_dic][5]
        shelf_item_pose.pose.orientation.w = self.RB1_position[key_dic][6]
        print('Received request for item picking at ' + request_item_location + '.')
        self.navigator.goToPose(shelf_item_pose)

        # Do something during your route
        # (e.x. queue up future tasks or detect person for fine-tuned positioning)
        # Print information for workers on the robot's ETA for the demonstration
        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival at ' + key_dic +
                    ' for worker: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.state_nav = self.state_num
            print('! Bringing product to shipping destination (' + key_dic + ')...')

        elif result == TaskResult.CANCELED:
            print('Task at ' + key_dic  +
                ' was canceled. Returning to starting point...')
            exit(-1)

        elif result == TaskResult.FAILED:
            print('Task at ' + key_dic + ' failed!')
            exit(-1)

        while not self.navigator.isTaskComplete():
            pass

def main():

    rclpy.init()
    cart_frame_node_ = FollowCartFrame()
    up_down_node_ = MoveShelfNode()
    footprint_pub_node_ = FootPrintPublisher()
    navigation_node_ = Navigation()

    executor = MultiThreadedExecutor()
    executor.add_node(cart_frame_node_)
    executor.add_node(up_down_node_)
    executor.add_node(footprint_pub_node_)
    executor.add_node(navigation_node_)

    try:
        #Set Initial Pose && Go to Loading Pose
        if(navigation_node_.state_nav ==1):
            navigation_node_.get_logger().info(f"[1] state_nav = {navigation_node_.state_nav}")
            navigation_node_.get_logger().info("[Beginning script, shut down with CTRL-C]")
            navigation_node_.get_logger().info(" Setting Initial Pose")
            navigation_node_.set_pos_init()
            navigation_node_.go_pose(request_loading_position,2) #which means go to "loading_position"
            navigation_node_.get_logger().info(f"[1] state_nav = {navigation_node_.state_nav}")

        #Follow robot_cart_frame && Load shelf
        elif(navigation_node_.state_nav ==2):
            cart_frame_node_.get_logger().info(f"[2] state_nav = {navigation_node_.state_nav}")
            cart_frame_node_.timer_on = True #Start following cart_frame [real robot]
            navigation_node_.state_nav =3
            cart_frame_node_.get_logger().info(f"[2] state_nav = {navigation_node_.state_nav}")
            #After RB-1 reach object, I use elevator_up and change the footprint here.
            #In process...

        elif(navigation_node_.state_nav ==3):
            navigation_node_.get_logger().info(f"[3] state_nav = {navigation_node_.state_nav}") 
            #Once RB-1 have the object, use Navigation Class for rotating and backup in order to
            #Align robot for shipping_position
            navigation_node_.state_nav = 4
            navigation_node_.get_logger().info(f"[3] state_nav = {navigation_node_.state_nav}")

        #Send shipping position && down object && send initial position
        elif(navigation_node_.state_nav ==4):

            navigation_node_.get_logger().info(f"[4] state_nav = {navigation_node_.state_nav}") 

        else:

            cart_frame_node_.get_logger().info(f"[ERORR: Something happens]")

            pass

        executor.spin()#Where I put the spin? o I just use while instead?

    except KeyboardInterrupt:
        #Is it possible to have a general get_logger? how?
        navigation_node_.get_logger().info('Keyboard interrupt, shutting down.\n')

    #There are a lot of nodes!
    cart_frame_node_.destroy_node()
    up_down_node_.destroy_node()
    footprint_pub_node_.destroy_node()
    navigation_node_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    