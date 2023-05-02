#! /usr/bin/env python3
# search_client.py

from unittest import result
import rospy
import actionlib

from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback

class SearchActionClient():
    goal = SearchGoal()

    def feedback_callback(self, feedback_data: SearchFeedback):
        ## TODO: get the current distance travelled, from the feedback message
        ## and assign this to a class variable...
        self.distance = feedback_data.current_distance_travelled
        
        if self.msg_counter >5:
            print(f"FEEDBACK: distance travelled = {self.distance:.3f} meters")
        else:
            self.msg_counter += 1



    def __init__(self):
        self.distance = 0.0
        self.msg_counter = 0

        self.action_complete = False
        rospy.init_node("search_action_client")
        self.rate = rospy.Rate(1)

        ## TODO: setup a "simple action client" with a callback function
        ## and wait for the server to be available...
        self.client = actionlib.SimpleActionClient(
            "/Pranav_search",
            SearchAction
        )

        self.client.wait_for_server()



        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            ## TODO: cancel the goal request, if this node is shutdown before the action has completed...
            self.client.cancel_goal()

            rospy.logwarn("Goal Cancelled...")

        ## TODO: Print the result here...
        rospy.sleep(1) # wait for the result to come in
        result = self.client.get_result()
        print("RESULT:")
        print(f"  * Action State = {self.client.get_state()}")
        print(f"  * total_distance_travelled = {result.total_distance_travelled:.3f} meters")
        print(f"  * closest_object_distance = {result.closest_object_distance:.3f} meters")
        print(f"  * closest_object_angle = {result.closest_object_angle:.1} degrees")


    def main_loop(self):
        ## TODO: assign values to all goal parameters
        ## and send the goal to the action server...
        self.goal.approach_distance = 0.4 #meters
        self.goal.fwd_velocity = 0.1 #m/s

        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)


        while self.client.get_state() < 2:
            ## TODO: Construct an if statement and cancel the goal if the 
            ## distance travelled exceeds 2 meters...
            if self.distance > 2:
                print("STOP: Distance exceeded  meters!!")
                # break out of the while loop to stop the node:
                break

            self.rate.sleep()

        self.action_complete = True if self.client.get_state() == 3 else False

if __name__ == '__main__':
    ## TODO: Instantiate the node and call the main_loop() method from it...
    node = SearchActionClient()
    node.main_loop()
