#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class ItemPicker:
    def __init__(self):
        self.shelf_positions = {
            "shelf_A": [-3.829, -7.604],
            "shelf_B": [-3.791, -3.287],
            "shelf_C": [-3.791, 1.254],
            "shelf_D": [-3.24, 5.861]}

        self.shipping_destinations = {
            "recycling": [-0.205, 7.403],
            "pallet_jack7": [-0.073, -8.497],
            "conveyer_432": [6.217, 2.153],
            "frieght_bay_3": [-6.349, 9.147]}

    def pick(self, request_item_location, request_destination):
        rclpy.init()

        navigator = BasicNavigator()

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 3.45
        initial_pose.pose.position.y = 2.15
        initial_pose.pose.orientation.z = 1.0
        initial_pose.pose.orientation.w = 0.0
        navigator.setInitialPose(initial_pose)

        navigator.waitUntilNav2Active()

        shelf_item_pose = PoseStamped()
        shelf_item_pose.header.frame_id = 'map'
        shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
        shelf_item_pose.pose.position.x = self.shelf_positions[request_item_location][0]
        shelf_item_pose.pose.position.y = self.shelf_positions[request_item_location][1]
        shelf_item_pose.pose.orientation.z = 1.0
        shelf_item_pose.pose.orientation.w = 0.0
        print('Received request for item picking at ' + request_item_location + '.')
        navigator.goToPose(shelf_item_pose)

        i = 0
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival at ' + request_item_location +
                    ' for worker: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Got product from ' + request_item_location +
                '! Bringing product to shipping destination (' + request_destination + ')...')
            shipping_destination = PoseStamped()
            shipping_destination.header.frame_id = 'map'
            shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
            shipping_destination.pose.position.x = self.shipping_destinations[request_destination][0]
            shipping_destination.pose.position.y = self.shipping_destinations[request_destination][1]
            shipping_destination.pose.orientation.z = 1.0
            shipping_destination.pose.orientation.w = 0.0
            navigator.goToPose(shipping_destination)

        elif result == TaskResult.CANCELED:
            print('Task at ' + request_item_location +
                ' was canceled. Returning to staging point...')
            initial_pose.header.stamp = navigator.get_clock().now().to_msg()
            navigator.goToPose(initial_pose)

        elif result == TaskResult.FAILED:
            print('Task at ' + request_item_location + ' failed!')
            exit(-1)

        while not navigator.isTaskComplete():
            pass

        exit(0)

if __name__ == '__main__':
    picker = ItemPicker()
    picker.pick('shelf_C', 'pallet_jack7')
