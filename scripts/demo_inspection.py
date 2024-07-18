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

from copy import deepcopy

from geometry_msgs.msg import PoseStamped 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import math

import numpy as np

#Added a quaternion to euler formula as to be able to use the correct angle values, takes rool pitch and yaw and outputs quaterniono values in a matrix
def get_quaternion_from_euler(roll, pitch, yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2) #X component
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) #Y component
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) #Z component
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2) #W component
    print('test 1 \n \n \n \n \n \n \n' )
 
    return [qx, qy, qz, qw]

"""
Basic stock inspection demo. In this demonstration, the expectation
is that there are cameras or RFID sensors mounted on the robots
collecting information about stock quantity and location.
"""

#Add entire function into a swutch case function to be able to change inspection route from analog controller on robot.

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Inspection route, probably read in from a file for a real application
    # from either a map or drive and repeat. The main route that will be taken for DT3, needs to determine the waypoint values before starting. [x, y, angle]. Angle needs to be in rads and NOT DEG
    '''
    inspection_route = [
    	[2.870, 0.000, -1.571], #exit of extrance
        [2.870, -0.513, 3.1414], #start last line
        [1.191, -0.513, 3.1415], #midpoint of last line
        [0.487, -0.513, -1.571], #endpoint of final line
        [0.487, -0.818, 0.000], #start of middle line
        [1.191, -0.818, 0.000], #midpoint of middle line
        [2.316, -0.818, -1.571], #end of middle line
        [2.316, -1.000, 3.1414], #start of first line (might need to tweek values)
        [0.487, -1.000, -0.7855], #endpoint of first line with 45deg to thumper 
        [1.537, -2.146, -0.7855], #thumper impact
        [2.870, -0.900, 1.571], #exit of middle
        [2.870, 0.000, 3.1414], #entrance to tunnel
        [0.000, 0.000, 0.000], return to startpoint
        
    
    '''
    inspection_route = [
        [2.870, 0.000, -1.571], #exit of extrance
        [2.870, -3.153, 3.1414], #start enemy line
        [0.208, -3.000, 1.571], #end enemy line
        [0.208, -1.000, 0.000], #start first line
        [2.466, -1.000, 1.571], #end first line
        [2.316, -0.818, 3.1414], #start middle line
        [1.191, -0.818, 3.1414], #midpoint middle line
        [0.487, -0.818, 1.571], #end middle line
        [0.487, -0.513, 0.000], #start final line
        [1.191, -0.513, 0.000], #midpoint of final line
        [2.870, -0.513, 1.571], #end of spice collection
        [2.870, 0.000, 3.1414], #entrance to exiti
        [1.435, 0.000, 3.1414], #mid point return
        [0.000, 0.000, 3.1414]] #end point
   
    '''
    #More risky route through the final course
    inspection_route = [
        [2.870, 0.000, -1.571],
        [2.870, -2.337, 3.1414],
        [1.041, -2.337, -0.785],
        [1.134, -2.680, 0.000],
        [1.903, -2.553, 0.785],
        [2.032, -2.032, 2.356],
        [1.689, -1.689, 3.1415],
        [1.168, -1.816, -2.356],
        [1.041, -2.032, 0.000],
        [1.727, -1.994, 3.1415],
        [2.388, 1.841, -1.571],    #Waypoint that makes contact with the thumper (Need x point in a way that only a small part touches (depends on extended attachment)
        [0.358, -2.088, 1.571], #first waypoint outside of middle need to add middle waypoints later.
        [0.208, -1.000, 0.000],
        [2.466, -1.000, 1.571], 
        [2.316, -0.818, 3.1415],
        [0.487, -0.818, 1.571],
        [0.487, -0.513, 0.000],
        [2.870, -0.513, 1.571],
        [2.870, 0.000, 3.1415],
        [0.000, 0.000, 3.1415]]
    
    '''
  
    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.000 #Initial position set to 0.30 since 12 inch from walls (.3, -.0)
    initial_pose.pose.position.y = 0.000
    initial_pose.pose.orientation.z = 0.000
    initial_pose.pose.orientation.w = 1.000
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Send our route
    inspection_points = []
    inspection_pose = PoseStamped()
    inspection_pose.header.frame_id = 'map'
    inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
    inspection_pose.pose.orientation.z = 1.0
    inspection_pose.pose.orientation.w = 0.0
    
    for pt in inspection_route:
        q = 0
        inspection_pose.pose.position.x = pt[0]
        inspection_pose.pose.position.y = pt[1]
        q = get_quaternion_from_euler(0.00,0.00,pt[2]) #Added the euler to quaternion conversion formula as to be able to set the orientation. In effect for the following 4 lines for the x, y, z and w values
        inspection_pose.pose.orientation.x = q[0]
        inspection_pose.pose.orientation.y = q[1]
        inspection_pose.pose.orientation.z = q[2]
        inspection_pose.pose.orientation.w = q[3]
        inspection_points.append(deepcopy(inspection_pose))
        
        
    navigator.followWaypoints(inspection_points)

    # Do something during our route (e.x. AI to analyze stock information or upload to the cloud)
    # Simply the current waypoint ID for the demonstation
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Inspection of shelves complete! Returning to start...')
    elif result == TaskResult.CANCELED:
        print('Inspection of shelving was canceled. Returning to start...')
    elif result == TaskResult.FAILED:
        print('Inspection of shelving failed! Returning to start...')

    # go back to start
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.goToPose(initial_pose)
    while not navigator.isTaskComplete():
        pass

    exit(0)


if __name__ == '__main__':
    main()
    

