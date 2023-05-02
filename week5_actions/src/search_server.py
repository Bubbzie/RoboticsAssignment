#! /usr/bin/env python3
# search_server.py

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from tuos_ros_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal

# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import sqrt, pow

class SearchActionServer():
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
       
        self.actionserver = actionlib.SimpleActionServer(
            "/Pranav_search", 
            SearchAction, 
            self.action_server_launcher, 
            auto_start=False)

        self.actionserver.start()

        # pull in some useful publisher/subscriber functions from the tb3.py module:
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        rospy.loginfo("The 'Search Action Server' is active...")

    # The action's "callback function":
    def action_server_launcher(self, goal: SearchGoal):
        rate = rospy.Rate(10)

        success = True
        vel = goal.fwd_velocity
        dist = goal.approach_distance
        if vel > 0.26 or vel < 0:
            print("Invalid velocity")
            success = False
        if dist < 0.2:
            success = False
            print("Invalid distance")


        if not success:
            
            self.result.total_distance_travelled = -1.0
            self.result.closest_object_angle = -1.0
            self.result.closest_object_distance = -1.0
            self.actionserver.set_aborted(self.result)
            return


            return

        ## TODO: Print a message to indicate that the requested goal was valid
        print(f"Search Goal received: fwd_vel = {vel} m/s, approach distance = {dist}m.")

        # Get the robot's current odometry from the Tb3Odometry() class:
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy
        # Get information about objects up ahead from the Tb3LaserScan() class:
        self.closest_object = self.tb3_lidar.min_distance
        self.closest_object_location = self.tb3_lidar.closest_object_position

        ## TODO: set the robot's forward velocity (as specified in the "goal")...
        self.vel_controller.set_move_cmd(linear = vel, angular=0)

        ## TODO: establish a conditional statement so that the  
        ## while loop continues as long as the distance to the closest object
        ## ahead of the robot is always greater than the "approach distance"
        ## (as specified in the "goal")...
        while self.tb3_lidar.min_distance > dist:
            # update LaserScan data:
            self.closest_object = self.tb3_lidar.min_distance
            self.closest_object_location = self.tb3_lidar.closest_object_position

            ## TODO: publish a velocity command to make the robot start moving 
            self.vel_controller.publish()

            # determine how far the robot has travelled so far:
            self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))


            ## TODO: update all result parameters:
            self.result.total_distance_travelled = self.distance
            self.result.closest_object_angle = self.closest_object_location
            self.result.closest_object_distance = self.closest_object


            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                ## TODO: take appropriate action if the action is cancelled (peempted)...
                print("Pre-empt requested")
                self.actionserver.set_preempted(self.result)
                self.vel_controller.stop()

                success = False
                # exit the loop:
                break

            
            ## TODO: update all feedback message values and publish a feedback message:
            self.feedback.current_distance_travelled = self.distance
            self.actionserver.publish_feedback(self.feedback)


            rate.sleep()

        if success:
            rospy.loginfo("approach completed successfully.")
            ## TODO: Set the action server to "succeeded" and stop the robot...
            self.vel_controller.stop()
            self.actionserver.set_succeeded(self.result)




if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()
