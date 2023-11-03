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
from math import cos, sin, pi

#Positions to RB-1 Robot based on map 
#frame_id: map

RB1_positions = {
    #"loading_position": [5.845, -0.3,-0.7071067,0.7073883 ],
    #"init_position" : [0.0, 0.0, 0.0, 1.0]

    #Init positiion
#     - Translation: [0.021, -0.008, 0.000]
# - Rotation: in Quaternion [0.000, 0.000, -0.010, 1.000]

# loading real position
# - Translation: [0.239, -2.746, 0.000]
# - Rotation: in Quaternion [0.000, 0.000, -0.684, 0.730]

#shipping_ position

    "init_position" :   [0.0, -0.00, -0.000,
                        -0.000, -0.000, -0.000, 1.000],

    "loading_position": [0.239, -2.746, 0.000,
                        -0.000, -0.000, -0.684, 0.730],

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
# cli = navigator.create_client(GoToLoading, '/approach_shelf',callback_group=client_cb_group)
# while not cli.wait_for_service(timeout_sec=1.0):
#     navigator.get_logger().info('service not available, waiting again...')
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

def publish_footprint_robot():
    # publish footprint
    # Crear los puntos que forman el polígono
    footprint = Polygon()
    points = []
    for angle in range(0, 360, 10):
        point = Point32()
        point.x = 0.25 * cos(angle * pi / 180)  # Radio de 0.25
        point.y = 0.25 * sin(angle * pi / 180)  # Radio de 0.25
        points.append(point)
        # Asignar los puntos al polígono
        footprint.points = points
        
        pub_local.publish(footprint)
        pub_global.publish(footprint)

def go_to_point2():

    print('Got product from ' + request_item_location +
              '! Bringing product to shipping destination (' + request_destination + ')...')
        
    shipping_destination = PoseStamped()
    shipping_destination.header.frame_id = 'map'
    shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
    shipping_destination.pose.position.x = shipping_destinations[request_destination][0]
    shipping_destination.pose.position.y = shipping_destinations[request_destination][1]
    shipping_destination.pose.position.z = shipping_destinations[request_destination][2]
    shipping_destination.pose.orientation.x = shipping_destinations[request_destination][3]
    shipping_destination.pose.orientation.y = shipping_destinations[request_destination][4]
    shipping_destination.pose.orientation.z = shipping_destinations[request_destination][5]
    shipping_destination.pose.orientation.w = shipping_destinations[request_destination][6]
    navigator.goToPose(shipping_destination)
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + request_item_location +
                ' for worker: ' + '{0:.0f}'.format(
                Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                + ' seconds.')

def down_shelf():
    print('Down shelf')
    msg_pub = Empty()
    pub_shelf_down.publish(msg_pub)
    
    duration = Duration(seconds=5)
    rate = navigator.create_rate(10, navigator.get_clock())
    start_time = navigator.get_clock().now()
    while rclpy.ok() and (navigator.get_clock().now() - start_time) < duration:
        rate.sleep
def exit_under_the_shelf():
    # publish for fordward
    msg_vel = Twist()
    msg_vel.linear.x = 0.1
    duration = Duration(seconds=7)
    rate = navigator.create_rate(10, navigator.get_clock())
    start_time = navigator.get_clock().now()
    while rclpy.ok() and (navigator.get_clock().now() - start_time) < duration:
        msg = Twist()
        msg.linear.x = 0.1
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        pub_cmd_vel.publish(msg)
        print('moving forward')
        rate.sleep
    print('stop')
    msg.linear.x = 0.0
    pub_cmd_vel.publish(msg)

def exit_with_shelf():
    # publish for backward
    msg_vel = Twist()
    msg_vel.linear.x = -0.1
    #duration = Duration(seconds=12)
    duration = Duration(seconds=26)
    rate = navigator.create_rate(10, navigator.get_clock())
    start_time = navigator.get_clock().now()
    while rclpy.ok() and (navigator.get_clock().now() - start_time) < duration:
        msg = Twist()
        msg.linear.x = -0.1
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        pub_cmd_vel.publish(msg)
        print('moving backward with shelf')
        rate.sleep
    print('stop')
    msg.linear.x = 0.0
    pub_cmd_vel.publish(msg)
    #turn
    duration = Duration(seconds=3,nanoseconds=500000000)
    start_time = navigator.get_clock().now()
    while rclpy.ok() and (navigator.get_clock().now() - start_time) < duration:
        msg = Twist()
        msg.linear.x = 0.05
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = -0.30
        pub_cmd_vel.publish(msg)
        print('turn with shelf')
        rate.sleep
    print('stop')
    msg.linear.x = 0.1
    pub_cmd_vel.publish(msg)
    #forward
    duration = Duration(seconds=3)
    start_time = navigator.get_clock().now()
    while rclpy.ok() and (navigator.get_clock().now() - start_time) < duration:
        msg = Twist()
        msg.linear.x = 0.05
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = -0.30
        pub_cmd_vel.publish(msg)
        print('moving backward with shelf')
        rate.sleep
    print('stop')
    msg.linear.x = 0.0
    pub_cmd_vel.publish(msg)

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
            #set_pos_init()
            go_pose(RB1_positions,'init_position',0)
            print(RB1_positions['init_position'])
            print(state_nav)
            exit(0)
        # if state_nav==2:
        #     print(state_nav)
        #     call_service()
        # if state_nav==3:
        #     print(state_nav)
        #     publish_footprint_shelf()
        #     exit_with_shelf()
        #     go_pose(shipping_destinations,request_destination,4)
        # if state_nav==4:
        #     print(state_nav)
        #     down_shelf()
        #     exit_under_the_shelf()
        #     publish_footprint_robot()
        #     go_pose(RB1_positions,request_init_position,0)
        #     exit(0)
        # if state_nav==1:
        #     exit(0)


    exit(0)


if __name__ == '__main__':
    main()
    