#! /usr/bin/env python3

import time
from copy import deepcopy
import threading
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from geometry_msgs.msg import Polygon,Point32, Twist
from std_msgs.msg import Empty
from math import cos, sin, pi
# Shelf positions for picking


#Init Position

# - Translation: [-0.067, -0.141, 0.000]
# - Rotation: in Quaternion [0.000, -0.000, -0.002, 1.000]



#Loading Position
# - Translation: [5.557, -0.301, -0.000]
# - Rotation: in Quaternion [-0.000, -0.000, -0.648, 0.762]

#Shipping Position

# - Translation: [0.661, -3.097, 0.000]
# - Rotation: in Quaternion [0.000, 0.000, -0.710, 0.705]
    # "init_position" :   [0.031, -0.023, -0.000,
    #                     -0.000, -0.000, -0.000, 1.000],
shelf_positions = {
    #"loading_position": [5.845, -0.3,-0.7071067,0.7073883 ],
    #"init_position" : [0.0, 0.0, 0.0, 1.0]
    "init_position" :   [0.0, -0.0, -0.000,
                        -0.000, -0.000, -0.000, 1.000],
                        
    "loading_position": [5.600, -0.272, 0.000,
                        -0.000, -0.000, -0.679, 0.734],

    "shipping_position":[0.281, -3.0, 0.000,
                        0.000, 0.000, -0.689, 0.725]

    }

# Shipping destination for picked products
shipping_destinations = {
    "shipping_position":[0.281, -3.0, 0.000,
                        0.000, 0.000, -0.689, 0.725]
    }



rclpy.init()

#variables
state_nav= 0

navigator = BasicNavigator()

#create client for service approach
client_cb_group = MutuallyExclusiveCallbackGroup()

#create publish
pub_shelf_down = navigator.create_publisher(Empty,'/elevator_down',10)
pub_local = navigator.create_publisher(Polygon, '/local_costmap/footprint', 10)
pub_global = navigator.create_publisher(Polygon, '/global_costmap/footprint', 10)
pub_cmd_vel = navigator.create_publisher(Twist,'robot/cmd_vel',10)

# coordinates of important areas

####################
request_item_location = 'loading_position'
request_destination = 'shipping_position'
request_init_position = 'init_position'
####################


def set_pos_init():

    # Set your demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = shelf_positions[request_init_position][0]
    initial_pose.pose.position.y = shelf_positions[request_init_position][1]
    initial_pose.pose.position.z = shelf_positions[request_init_position][2]
    initial_pose.pose.orientation.x = shelf_positions[request_init_position][3]
    initial_pose.pose.orientation.y = shelf_positions[request_init_position][4]
    initial_pose.pose.orientation.z = shelf_positions[request_init_position][5]
    initial_pose.pose.orientation.w = shelf_positions[request_init_position][6]

    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

def call_service():
    global state_nav
    global cli
    # once arrived at loading position step to execute the approach service
    navigator.req = GoToLoading.Request()
    navigator.req.attach_to_shelf = True
    navigator.future = cli.call_async(navigator.req)
    rclpy.spin_until_future_complete(navigator, navigator.future)
    status = navigator.future.result()
    if status != True:
        #navigator.error('failed service')
        state_nav=3
    else:
        navigator.info('successful service!')
        state_nav=3

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

def main():
    global state_nav
    print(state_nav)
    state_nav = 1
    while(1):
        if state_nav==1:
            print(state_nav)
            set_pos_init()
            exit(0)

    exit(0)


if __name__ == '__main__':
    main()
    