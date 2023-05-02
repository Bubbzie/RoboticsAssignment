#!/usr/bin/env python3


from concurrent.futures.process import _ThreadWakeup
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from tuos_ros_msgs.srv import Approach, ApproachResponse, ApproachRequest
from geometry_msgs.msg import Twist


class ApproachService(): 

    def approach_callback(self, request: ApproachRequest):
        distance = request.approach_distance
        speed = request.approach_velocity

        response = ApproachResponse()
        vel = Twist()

        invalid_input = False
        if speed > 0.26 or speed <=0:
            invalid_input = True
        
        if distance <= 0.2:
            invalid_input = True
        
        if invalid_input:
            response.response_message = "Invalid Input!"
        else:
            vel.linear.x = speed
            self.pub.publish(vel)
            rospy.loginfo('Published the velocity command to /cmd_vel')
            while self.min_distance > distance:
                continue
            self.pub.publish(Twist())
            response.response_message = f"The robot has now stopped {self.min_distance} meters away from an object!"

        return response

    def scan_callback(self, scan_data): 
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.min_distance = front_arc.min()


    def __init__(self): 
        self.node_name = "Pranavs_approach_server"
        

        rospy.init_node(self.node_name, anonymous=True)

        sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        service = rospy.Service(self.node_name, Approach, self.approach_callback) 

        rospy.loginfo(f"The '{self.node_name}' node is active...")

    def main_loop(self):
        rospy.spin() 

if __name__ == '__main__': 
    node = ApproachService()
    node.main_loop()
