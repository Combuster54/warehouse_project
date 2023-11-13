#! /usr/bin/env python3

import time
import math
import threading
from math import cos, sin, pi
from copy import deepcopy
# ----- #
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# ----- #
from std_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Polygon, Point32, Twist
# from attach_shelf.srv import GoToLoading
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# ----- #

####################
request_init_position = 'init_position'
request_loading_position = 'loading_position'
request_shipping_position = 'shipping_position'
####################

class FollowCartFrame(Node):

    def __init__(self):
        super().__init__('FollowFrame_Node')

        # Crear el publisher
        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(Twist, 'robot/cmd_vel', qos_profile)

        # Crear el buffer y el listener de transformaciones
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.ready_for_footprint = False
        self.cmd_vel = Twist()

        #timer 
        self.timer_ = None

        self.timer_on = False

        return None

    def call_timer(self):
        print("Starting_timer!!!")
        self.timer_ = self.create_timer(0.2, self.send_command)

    def send_command(self):

        # print("send_command")
        if(self.timer_on):

            try:
                # print("timer is on!!!")
                # Obtener la transformación entre "rick/base_link" y "morty/base_link"
                transform = self.tf_buffer_.lookup_transform('robot_base_footprint', 'robot_cart_laser', rclpy.time.Time())

                # Calcular la distancia y el error angular entre los frames de referencia
                error_distance = math.sqrt(pow(transform.transform.translation.x, 2) +
                                        pow(transform.transform.translation.y, 2))
                error_yaw = math.atan2(transform.transform.translation.y,
                                    transform.transform.translation.x)

                # Definir la velocidad angular y lineal
                kp_yaw = 0.25
                kp_distance = 0.05
                angular_vel = kp_yaw * error_yaw
                linear_vel = kp_distance * error_distance

                # Verificar si Rick ya alcanzó a Morty
                if error_distance <= 0.20:
                    self.get_logger().info('[FollowCartFrame] robot_cart_laser has been reached!')
                    self.cmd_vel.angular.z = 0.0
                    self.cmd_vel.linear.x = 0.0
                    self.timer_on = False
                    self.ready_for_footprint = True

                    self.timer_.cancel()
                else:
                    # Sigue a Morty
                    self.cmd_vel.angular.z = angular_vel
                    self.cmd_vel.linear.x = linear_vel

                # Publicar la velocidad
                self.publisher_.publish(self.cmd_vel)

            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn('TF2 error: {}'.format(str(e)))
                self.get_logger().warn("[FollowCartFrame send_command EXCEPT]")
#                self.ready_for_footprint = True

                self.timer_.cancel()

                self.ready_for_footprint = True


        return None

class MoveShelfNode(Node):

    def __init__(self):
        super().__init__('MoveShelf_Node')
        self.publisher_down_ = self.create_publisher(String, '/elevator_down', 10)
        self.publisher_up_ = self.create_publisher(String, '/elevator_up', 10)
        self.elevator_msg = String()
        self.elevator_msg.data = ""   # this will always be blank string!
        return None

    def publish_message_down(self):
        self.publisher_down_.publish(self.elevator_msg)
        self.get_logger().info(f'[Down-Elevator] Publishing...')
        time.sleep(2.0)
        return None

    def publish_message_up(self):
        self.publisher_up_.publish(self.elevator_msg)
        self.get_logger().info(f'[Up-Elevator] Publishing...')
        time.sleep(2.0)
        return None

class FootPrintPublisher(Node):

    def __init__(self):
        super().__init__('FootPrintPublisher_Node')

        self.pub_local = self.create_publisher(Polygon, '/local_costmap/footprint', 10)
        self.pub_global = self.create_publisher(Polygon, '/global_costmap/footprint', 10)

        self.publisher_down_ = self.create_publisher(String, '/elevator_down', 10)
        self.publisher_up_ = self.create_publisher(String, '/elevator_up', 10)

        self.elevator_msg = String()
        self.elevator_msg.data = ""   # this will always be blank string!
        return None
        
    def publish_footprint_shelf(self):

        footprint = Polygon()
        point1 = Point32()
        point1.x = 0.5
        point1.y = 0.4

        point2 = Point32()
        point2.x = 0.5
        point2.y = -0.4

        point3 = Point32()
        point3.x = -0.5
        point3.y = -0.4

        point4 = Point32()
        point4.x = -0.5
        point4.y = 0.4

        footprint.points = [point1, point2, point3, point4]

        self.pub_local.publish(footprint)
        self.pub_global.publish(footprint)
        self.get_logger().info(f"Publish Shelf FootPrint") 

        return None

    def publish_init_footprint(self):


        footprint = Polygon()
        point1 = Point32()
        point1.x = 0.25
        point1.y = 0.25

        point2 = Point32()
        point2.x = 0.25
        point2.y = -0.25

        point3 = Point32()
        point3.x = -0.25
        point3.y = -0.25

        point4 = Point32()
        point4.x = -0.25
        point4.y = 0.25

        footprint.points = [point1, point2, point3, point4]

        self.pub_local.publish(footprint)
        self.pub_global.publish(footprint)
        self.get_logger().info(f"Publish Init FootPrint") 

        return None

class Navigation(Node):

    def __init__(self):
        super().__init__('Navigator_Node')

        self.state_nav = 1
        self.navigator = BasicNavigator()
        self.client_cb_group = MutuallyExclusiveCallbackGroup()

        #Nodes
        self.follow_cart_frame = FollowCartFrame()
        self.move_shelf_node = MoveShelfNode()
        self.footprint_publisher = FootPrintPublisher()

        self.rate1 = self.create_rate(10,self.get_clock())  # 10 Hz, cada 0.1 segundos
        self.publisher_ = self.create_publisher(Twist, 'robot/cmd_vel', 10)
        self.duration_backward = Duration(seconds=16)
        self.duration_forward = Duration(seconds=10)
        self.duration_spin = Duration(seconds=10) 
        self.counter = 0
        self.timer_called = False

        self.RB1_position = {

            "init_position" :   [0.031, -0.023, -0.000,
                                -0.000, -0.000, -0.000, 1.000],

            "loading_position": [4.39, -0.199, 0.000,
                                -0.000, -0.000,  -0.782, 0.667],

            "shipping_position":[0.281, -3.0, 0.000,
                                0.000, 0.000, -0.689, 0.725]
        }

        # call the navigation process once here

        self.nav_timer = self.create_timer(10, self.navigation_process)

        return None

    def publish_velocity_spin(self):
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) < self.duration_spin:
            msg = Twist()
            msg.linear.x = 0.00
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = -0.27
            self.publisher_.publish(msg)
            self.get_logger().info(f"[Publish Velocity] Backward") 
            self.rate1.sleep
            
        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        
    def publish_velocity_backward(self):
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) < self.duration_backward:
            msg = Twist()
            msg.linear.x = -0.04
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info(f"[Publish Velocity] Backward") 
            self.rate1.sleep
            
        msg.linear.x = 0.0
        self.publisher_.publish(msg)

    def publish_velocity_forward(self):
        start_time = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - start_time) < self.duration_forward:
            msg = Twist()
            msg.linear.x = 0.04
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info(f"[Publish Velocity] Forward") 
            self.rate1.sleep
            
        msg.linear.x = 0.0
        self.publisher_.publish(msg)

    def set_pos_init(self):

        # Setting inialpose
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

    def go_pose(self, key_dic):

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
        print('Received request for item picking at ' + key_dic + '.')
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
            print('(' + key_dic + ') position has been reached..!')
            if(self.state_nav == 4):
                print("[state_nav] SUCCEEDED")
                self.state_nav = 5
                self.nav_timer.cancel()


        elif result == TaskResult.CANCELED:
            print('Task at ' + key_dic  +
                ' was canceled. Returning to starting point...')
            if(self.state_nav == 4):
                print("[state_nav] CANCELED")
                self.state_nav = 5
            exit(-1)

        elif result == TaskResult.FAILED:
            print('Task at ' + key_dic + ' failed!')
            if(self.state_nav == 4):
                print("[state_nav] FAILED")
                self.state_nav = 5
            exit(-1)

        while not self.navigator.isTaskComplete():
            pass

        return None

    def navigation_process(self):
        print("[Nav process]")
        # if(self.state_nav ==0):
        #     self.self.nav_timer.cancel()

        #Set Initial Pose && Go to Loading Pose
        if(self.state_nav == 1):
            self.state_nav = 2
            self.get_logger().info(f"[START-1] state_nav = {self.state_nav}")
            self.get_logger().info("[Beginning script, shut down with CTRL-C]")
            self.set_pos_init()
            time.sleep(4)
            self.go_pose(request_loading_position) #which means go to "loading_position"
            self.get_logger().info(f"[FINISHIM-1] state_nav = {self.state_nav}")
            self.state_nav = 2
        #Follow robot_cart_frame && publish_message_up && publish_footprint_shelf
        if(self.state_nav == 2):
            #self.state_nav = 0 #To not repeat them
            self.state_nav == -1
            self.get_logger().info(f"call timer")
            if (not self.timer_called):
                self.follow_cart_frame.timer_on = True #Start following cart_frame [real robot]
                self.follow_cart_frame.call_timer()
                self.timer_called = True
            #self.state_nav == -1
            self.follow_cart_frame.get_logger().info(f"[START-2] state_nav = {self.state_nav}")

        if(self.state_nav == -1): 
            self.follow_cart_frame.get_logger().info(f"Following robot_cart_laser frame...")

        if(self.follow_cart_frame.ready_for_footprint):
            self.state_nav = 3
            self.follow_cart_frame.get_logger().info(f"[FINISHIM-2] state_nav = {self.state_nav}")
            self.publish_velocity_forward()
            self.get_logger().info(f"[FootPrint READY] PublishFootprint && Elevator UP!")
            self.move_shelf_node.publish_message_up()
            self.footprint_publisher.publish_footprint_shelf()

        # Backup && rotate in order to move for the new goal
        if(self.state_nav == 3):
            self.state_nav = 4
            self.follow_cart_frame.ready_for_footprint = False
            self.publish_velocity_backward()
            self.publish_velocity_spin()
            self.get_logger().info(f"[START-3] state_nav = {self.state_nav}")
            self.publish_velocity_forward()

        #Send shipping position && down object && send initial position
        if(self.state_nav == 4):
            self.state_nav = 0
            self.get_logger().info(f"[FINISHIM-3] state_nav = {self.state_nav}")
            self.get_logger().info(f"[START-4] state_nav = {self.state_nav}")
            self.go_pose(request_shipping_position)
            self.get_logger().info(f"[RESTARTFOOTPRINT] PublishFootprint && Elevator Down!")
            self.footprint_publisher.publish_init_footprint()
            self.move_shelf_node.publish_message_down()
            self.publish_velocity_backward()
            self.go_pose(request_init_position)
            self.get_logger().info(f"[FINISH-4] state_nav = {self.state_nav}") 
        if(self.state_nav == 5):
            self.go_pose(request_init_position)
            self.get_logger().info(f"CheckPoint6 Done!")
            self.nav_timer.cancel()

        return None

def main():

    rclpy.init()

    navigation_node_ = Navigation()

    cart_frame_node_ = navigation_node_.follow_cart_frame
    move_shelf_node_ = navigation_node_.move_shelf_node
    footprint_pub_node_ = navigation_node_.footprint_publisher

    executor = MultiThreadedExecutor()

    executor.add_node(cart_frame_node_)
    executor.add_node(move_shelf_node_)
    executor.add_node(footprint_pub_node_)
    executor.add_node(navigation_node_)

    try:
        print("hello")
        executor.spin()#Where I put the spin? o I just use while instead?
    except KeyboardInterrupt :
        #Is it possible to have a general get_logger? how?
        print("[ERROR]")
        navigation_node_.get_logger().info('Keyboard interrupt, shutting down.\n')

    #There are a lot of nodes!
    cart_frame_node_.destroy_node()
    move_shelf_node_.destroy_node()
    footprint_pub_node_.destroy_node()
    navigation_node_.destroy_node()
    rclpy.shutdown()

    return None

if __name__ == '__main__':
    main()