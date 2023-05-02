#!/usr/bin/env python3
# A simple ROS publisher node in Python

import rospy 
from std_msgs.msg import String 
from geometry_msgs.msg import Twist


class Publisher(): 

    def __init__(self): 
        self.node_name = "move_circle_publisher" 
        topic_name = "/cmd_vel" 

        pub = rospy.Publisher(topic_name, Twist, queue_size=10) # a queue size of 10 usually works!
        

        rospy.init_node(self.node_name, anonymous=True) 
        self.rate = rospy.Rate(10) 

        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

        

    def shutdownhook(self): 
        topic_name = "/cmd_vel" 
        pub = rospy.Publisher(topic_name, Twist, queue_size=10)

        vel_cmd = Twist()
        #Command tp stop the robot when shutdown function is called
        vel_cmd.linear.x = 0.0 # m/s
        vel_cmd.angular.z = 0.0 # rad/s
        pub.publish(vel_cmd)

        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def main_loop(self):
        topic_name = "/cmd_vel" 
        pub = rospy.Publisher(topic_name, Twist, queue_size=10) # a queue size of 10 usually works!
         #Twist message commands
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.4 # m/s
        vel_cmd.angular.z = 0.4 # rad/s

        #sending the Twist message to make the robot move in a circle
        pub.publish(vel_cmd)


        while not self.ctrl_c: 
            print(f"The robot is moving with x linear speed {vel_cmd.linear.x:.3f} (m/s), angular theta speed {vel_cmd.angular.z:.3f} (rad/s)")
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.4 # m/s
            vel_cmd.angular.z = 0.4 # rad/s
            pub.publish(vel_cmd)
            self.rate.sleep()
        


if __name__ == '__main__': 
    publisher_instance = Publisher() 
    try:
        publisher_instance.main_loop() 
    except rospy.ROSInterruptException:
        pass