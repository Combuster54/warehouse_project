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
from geometry_msgs.msg import Polygon,Point32, Twist
# from attach_shelf.srv import GoToLoading
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# ----- #

####################
request_loading_position = 'loading_position'
request_destination = 'shipping_position'
request_init_position = 'init_position'
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

        self.timer_on = False
        self.ready_for_footprint = False
        self.cmd_vel = Twist()

        return None

    # def create_timer(self):

    #     self.timer_ = self.create_timer(0.2, self.send_command)

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
        super().__init__('FootPrintPublisher_Node')

        self.pub_local = self.create_publisher(Polygon, '/local_costmap/footprint', 10)
        self.pub_global = self.create_publisher(Polygon, '/global_costmap/footprint', 10)

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

        return None

    def publish_init_footprint(self):

        points = [Point32(0.25, 0.25, 0.0), Point32(0.25, -0.25, 0.0),
                Point32(-0.25, -0.25, 0.0), Point32(-0.25, 0.25, 0.0)]

        footprint = Polygon(points=points)

        self.pub_local.publish(footprint)
        self.pub_global.publish(footprint)

        return None

class Navigation(Node):

    def __init__(self):
        super().__init__('Navigator_Node')

        self.state_nav = 0
        self.navigator = BasicNavigator()
        self.client_cb_group = MutuallyExclusiveCallbackGroup()

        
        #Nodes
        self.follow_cart_frame = FollowCartFrame()
        self.move_shelf_node = MoveShelfNode()
        self.footprint_publisher = FootPrintPublisher()

        self.counter = 0
        self.RB1_position = {

            "init_position" :   [0.031, -0.023, -0.000,
                                -0.000, -0.000, -0.000, 1.000],

            "loading_position": [4.255, -0.199, 0.000,
                                -0.000, -0.000,  -0.782, 0.623],

            "shipping_position":[0.281, -3.0, 0.000,
                                0.000, 0.000, -0.689, 0.725]
        }

        #Service Approach Client for SIMULATION Purposes
        # self.cli = self.navigator.create_client(GoToLoading, '/approach_shelf',callback_group=client_cb_group)
        # while not cli.wait_for_service(timeout_sec=1.0):
        #     self.navigator.get_logger().info('service not available, waiting again...')

        # call the navigation process once here
        self.navigation_process()

        return None

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

        return None

    def navigation_process(self):
        self.state_nav =1
        #Set Initial Pose && Go to Loading Pose
        if(self.state_nav == 1):
            self.get_logger().info(f"[START-1] state_nav = {self.state_nav}")
            self.get_logger().info("[Beginning script, shut down with CTRL-C]")
            #self.set_pos_init()
            #time.sleep(3)
            self.go_pose(request_init_position, 2) #which means go to "loading_position"
            self.get_logger().info(f"[FINISHIM-1] state_nav = {self.state_nav}")

        # #Follow robot_cart_frame && publish_message_up && publish_footprint_shelf
        # elif(self.state_nav == 2):
        #     self.follow_cart_frame.get_logger().info(f"[START-2] state_nav = {self.state_nav}")
        #     self.follow_cart_frame.timer_on = True #Start following cart_frame [real robot]
        #     while(self.follow_cart_frame.ready_for_footprint == False):
        #         pass
        #     #Start with publisherFootPrint && MoveShelfUp
        #     while(self.counter<= 10):
        #         self.footprint_publisher.publish_footprint_shelf()
        #         #self.move_shelf_node.publish_message_up()
        #         self.follow_cart_frame.get_logger().info(f"Elevator UP!")
        #         self.counter += 1
        #     #Once It suppossed that everything is okay... I hope so e.e
        #     self.state_nav = 3
        #     self.follow_cart_frame.get_logger().info(f"[FINISHIM-2] state_nav = {self.state_nav}")
        
        # # Backup && rotate in order to move for the new goal
        # elif(self.state_nav == 3):
        #     self.get_logger().info(f"[START-3] state_nav = {self.state_nav}")
        #     self.navigator.backup(backup_dist=0.3, backup_speed=0.95, time_allowance=0.3)
        #     # self.navigator.spin(spin_dist=1.57, time_allowance=10)
        #     #Once RB-1 have the object, use Navigation Class for rotating and backup in order to
        #     #Align robot for shipping_position
        #     self.state_nav = 4
        #     self.get_logger().info(f"[FINISHIM-3] state_nav = {self.state_nav}")
        #     self.get_logger().info(f"END")
        #     exit(0)
        # #Send shipping position && down object && send initial position
        # elif(self.state_nav == 4):

        #     self.get_logger().info(f"[START-4] state_nav = {self.state_nav}") 

        else:

            self.navigator.get_logger().info(f"[ERORR: Something happens]")

            pass

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
    except KeyboardInterrupt:
        #Is it possible to have a general get_logger? how?
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