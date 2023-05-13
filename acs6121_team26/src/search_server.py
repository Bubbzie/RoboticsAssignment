#! /usr/bin/env python3
# search_server_test.py

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from tuos_ros_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal

# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import sqrt, pow

import os
import roslaunch
import time

class SearchActionServer():
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        # Initialize the action server:
        self.actionserver = actionlib.SimpleActionServer(
            "/Pranav_search", 
            SearchAction, 
            self.action_server_launcher, 
            auto_start=False)
        self.actionserver.start()

        # Initialize the necessary modules:
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        rospy.loginfo("The 'Search Action Server' is active...")

        # Create maps folder
        os.system("mkdir ~/catkin_ws/src/acs6121_team26/maps/")

    def map_node(self):

        # Launch the map saving node using roslaunch:
        try:
            # Generate a UUID and configure ROS logging:

            node = roslaunch.core.Node(
                package = 'map_server',
                node_type = 'map_saver',
                
                args = '-f /home/student/catkin_ws/src/acs6121_team26/maps/explore_map',
                name = 'map_saver_test',
                output='screen',
            )


            # Create a ROSLaunch() object and launch the map saver node:
            launch = roslaunch.scriptapi.ROSLaunch()
            launch.start()

            process = launch.launch(node)
            print (f"Process alive state: {process.is_alive()}")

        except Exception as exception_object:
            print("(!) There was an error during the 'launch_another_node()' function.")
            print(f"Error: {exception_object}")

        # Sleep for a short amount of time to allow the map saving node to finish saving the map:
        time.sleep(2)

    # The action's "callback function":
    def action_server_launcher(self, goal: SearchGoal):
        rate = rospy.Rate(10)

        # Get the robot's current odometry from the Tb3Odometry() class:
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy
        self.posyaw0 = self.tb3_odom.yaw

        # Get information about objects up ahead from the Tb3LaserScan() class:
        self.closest_object = self.tb3_lidar.min_distance
        self.closest_object_location = self.tb3_lidar.closest_object_position

        # Initialize success to True:
        success = True

        # Check if the goal parameters are valid:
        vel = goal.fwd_velocity
        dist = goal.approach_distance
        if vel > 0.26 or vel < 0:
            rospy.logwarn("Invalid velocity")
            success = False
        if dist < 0.2:
            rospy.logwarn("Invalid distance")
            success = False

        # If the goal parameters are not valid, set the result to -1 and abort the action:
        if not success:
            self.result.total_distance_travelled = -1.0
            self.result.closest_object_angle = -1.0
            self.result.closest_object_distance = -1.0
            self.actionserver.set_aborted(self.result)
            return

        # Print a message to indicate that the requested goal was valid:
        rospy.loginfo(f"Search Goal received: fwd_vel = {vel} m/s, approach distance = {dist}m.")

        left_fight_count=0

        # Outer loop to keep the robot searching indefinitely:
        while not self.actionserver.is_preempt_requested() and not rospy.is_shutdown():

            # Set the robot's forward velocity as specified in the "goal":
            self.vel_controller.set_move_cmd(linear=vel, angular=0)
            
                        # Inner loop to move forward and search for objects:
            while self.tb3_lidar.min_distance > dist:
                # Update LaserScan data:
                self.closest_object = self.tb3_lidar.min_distance
                self.closest_object_location = self.tb3_lidar.closest_object_position

                # Publish the velocity command to make the robot move forward:
                self.vel_controller.publish()

                # Determine how far the robot has traveled so far:
                self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))

                # Update result parameters:
                self.result.total_distance_travelled = self.distance
                self.result.closest_object_angle = self.closest_object_location
                self.result.closest_object_distance = self.closest_object

                # Update feedback message values and publish a feedback message:
                self.feedback.current_distance_travelled = self.distance
                self.actionserver.publish_feedback(self.feedback)

                # Check if there has been a request to cancel the action mid-way through:
                if self.actionserver.is_preempt_requested():
                    # Take appropriate action if the action is canceled (preempted):
                    rospy.loginfo("Action preempt requested")
                    self.actionserver.set_preempted(self.result)
                    self.vel_controller.stop()
                    success = False
                    break
                    
                rate.sleep()

                # Launch map node to save the map
                self.map_node()

            # Check if the action was preempted:
            if not success:
                break

            # Stop the robot after detecting an object:
            self.vel_controller.set_move_cmd(linear=0, angular=0)
            self.vel_controller.publish()

            ##if(left_fight_count==0):

                 # Turn the robot 90 degrees to the right:
            self.vel_controller.set_move_cmd(linear=0, angular=1.6)
            self.vel_controller.publish()
                ##left_fight_count+=1
            #rospy.sleep() 
            ##else:
             ##   self.vel_controller.set_move_cmd(linear=0, angular=-1.6)
              ##  self.vel_controller.publish()
              ##  left_fight_count=0
              ##  rospy.sleep(1.5) 
                # Wait for the robot to complete the turn:
                # rospy.sleep(0.009)  
            

        # Check if the action was completed successfully:
        if success:
            rospy.loginfo("Approach completed successfully.")
            # Set the action server to "succeeded" and stop the robot:
            self.actionserver.set_succeeded(self.result)
            self.vel_controller.stop()

if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()