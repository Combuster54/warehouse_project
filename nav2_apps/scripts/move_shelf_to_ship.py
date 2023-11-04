#! /usr/bin/env python3

import time
from copy import deepcopy
import threading
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# from attach_shelf.srv import GoToLoading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from geometry_msgs.msg import Polygon,Point32, Twist
from std_msgs.msg import Empty
from std_msgs.msg import String

#Positions to RB-1 Robot based on map 
#frame_id: map

RB1_positions = {
    "init_position" :   [0.0, -0.00, -0.000,
                        -0.000, -0.000, -0.000, 1.000],

    "loading_position": [4.377, -0.684, 0.000,
                        -0.000, -0.000, -0.709,  0.705],

    "shipping_position":[0.281, -3.0, 0.000,
                        0.000, 0.000, -0.689, 0.725]
    }

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

rclpy.init()

#variables
state_nav= 0

navigator = BasicNavigator()
pub_shelf_down = navigator.create_publisher(Empty,'/elevator_down',1)
pub_local = navigator.create_publisher(Polygon, '/local_costmap/footprint', 10)
pub_global = navigator.create_publisher(Polygon, '/global_costmap/footprint', 10)
pub_cmd_vel = navigator.create_publisher(Twist,'robot/cmd_vel',10)

# coordinates of important areas

####################
request_item_location = 'loading_position'
request_destination = 'shipping_position'
request_init_position = 'init_position'
####################

def publish_footprint_shelf():

    #Add publisher footprint_ 2.5-5hz or 1hz
    # publish footprint
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
        
    pub_local.publish(footprint)
    pub_global.publish(footprint)

def pub_initial_footprint():
    # publish footprint
    # Crear los puntos que forman el polígono
    footprint = Polygon()
    points = []

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
    point4.y =  0.25

    footprint.points = [point1, point2, point3, point4]

    pub_local.publish(footprint)
    pub_global.publish(footprint)

def set_pos_init():

    # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = RB1_positions[request_init_position][0]
    initial_pose.pose.position.y = RB1_positions[request_init_position][1]
    initial_pose.pose.position.z = RB1_positions[request_init_position][2]
    initial_pose.pose.orientation.x = RB1_positions[request_init_position][3]
    initial_pose.pose.orientation.y = RB1_positions[request_init_position][4]
    initial_pose.pose.orientation.z = RB1_positions[request_init_position][5]
    initial_pose.pose.orientation.w = RB1_positions[request_init_position][6]

    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()


def go_pose(go_pose_dic, key_dic, state_num):
    global state_nav
    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = go_pose_dic[key_dic][0]
    shelf_item_pose.pose.position.y = go_pose_dic[key_dic][1]
    shelf_item_pose.pose.position.z = go_pose_dic[key_dic][2]
    shelf_item_pose.pose.orientation.x = go_pose_dic[key_dic][3]
    shelf_item_pose.pose.orientation.y = go_pose_dic[key_dic][4]
    shelf_item_pose.pose.orientation.z = go_pose_dic[key_dic][5]
    shelf_item_pose.pose.orientation.w = go_pose_dic[key_dic][6]
    print('Received request for item picking at ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)

    # Do something during your route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Print information for workers on the robot's ETA for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + key_dic +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')
    
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        state_nav = state_num
        print('! Bringing product to shipping destination (' + key_dic + ')...')

    elif result == TaskResult.CANCELED:
        print('Task at ' + key_dic  +
              ' was canceled. Returning to staging point...')
        exit(-1)

    elif result == TaskResult.FAILED:
        print('Task at ' + key_dic + ' failed!')
        exit(-1)

    while not navigator.isTaskComplete():
        pass

# def self_localize():
#     #rotate 2 sec for autolocalization
#     pass

def main():
    global state_nav
    print(state_nav)
    state_nav = 1
    while(1):
        if state_nav==1:
            print(state_nav)
            #self_localize()
            set_pos_init()
            go_pose(RB1_positions,'loading_position',0)
            print(RB1_positions['loading_position'])
            exit(0)
        if state_nav==2:
            print(state_nav)
            #call follow robot_cart_frame  && state_nav=3
        if state_nav==3:
            print(state_nav)
            publish_footprint_shelf()
            #Do some movements with navigator.backup nad navigator.rotate
            go_pose(RB1_positions,'shipping_position',4)
        if state_nav==4:
            print(state_nav)
            #down Shelf && backup && spin 180 degrees
            pub_initial_footprint()
            go_pose(RB1_positions,'init_position',0)
            exit(0)
        if state_nav==1:
            exit(0)
    exit(0)

if __name__ == '__main__':
    main()
    