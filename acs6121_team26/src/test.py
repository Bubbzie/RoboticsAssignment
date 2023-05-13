#! /usr/bin/env python3

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

class explore():

    def __init__(self):
        node_name = "explore"
        # a flag if this node has just been launched
        self.startup = True

        # Initialize the necessary modules:
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # hz

        # Create maps folder
        os.system("mkdir ~/catkin_ws/src/acs6121_team26/maps/")

        # This might be useful in the main_loop() (to switch between 
        # turning and moving forwards)
        self.turn = False

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")

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

    def shutdownhook(self):
        self.vel_controller.stop()
        self.ctrl_c = True

    # The action's "callback function":
    def main_loop(self):
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
        vel = 0.26
        dist = 0.6
        if vel > 0.26 or vel < 0:
            rospy.logwarn("Invalid velocity")
            success = False
        if dist < 0.2:
            rospy.logwarn("Invalid distance")
            success = False

        # If the goal parameters are not valid, set the result to -1 and abort the action:
        #if not success:
         #   self.result.total_distance_travelled = -1.0
          #  self.result.closest_object_angle = -1.0
           # self.result.closest_object_distance = -1.0
            #self.actionserver.set_aborted(self.result)
            #return

        # Print a message to indicate that the requested goal was valid:
        rospy.loginfo(f"Search Goal received: fwd_vel = {vel} m/s, approach distance = {dist}m.")

        # Outer loop to keep the robot searching indefinitely:
        while not self.ctrl_c() and not rospy.is_shutdown():

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
                    
                rate.sleep()

                # Launch map node to save the map
                self.map_node()

            # Check if the action was preempted:
            if not success:
                break

            # Stop the robot after detecting an object:
            self.vel_controller.set_move_cmd(linear=0, angular=0)
            self.vel_controller.publish()

            self.vel_controller.set_move_cmd(linear=0, angular=1.6)
            self.vel_controller.publish()

            

        # Check if the action was completed successfully:
        if success:
            rospy.loginfo("Approach completed successfully.")
            self.vel_controller.stop()

if __name__ == "__main__":
    node = explore()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass