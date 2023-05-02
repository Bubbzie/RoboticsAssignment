#!/usr/bin/env python3 

from shutil import move

from numpy import False_
import rospy
from geometry_msgs.msg import Twist 
from tuos_ros_msgs.srv import TimedMovement, TimedMovementResponse, TimedMovementRequest

service_name = "timed_move_service"

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 

def callback_function(service_request: TimedMovementRequest): 

    vel = Twist()

    service_response = TimedMovementResponse() 

    movement = service_request.movement_request
    duration = service_request.duration

    invalid_input = False

    if duration <= 0:
        invalid_input = True
    
    if movement =="fwd":
        vel.linear.x = 0.1  #m/s
    elif movement == "back":
        vel.linear.x = -0.1 #m/s
    elif movement == "left":
        vel.angular.z = 0.2 #rad/s
    elif movement == "right":
        vel.angular.z = -0.2
    else:
        invalid_input = True
    
    if invalid_input:
        rospy.loginfo("Invalid Input")
        service_response.success = False
    else:
        service_response.success = True
        StartTime = rospy.get_time() #Get time in seconds
        pub.publish(vel)

        rospy.loginfo('Published the velocity command to /cmd_vel')
        while (rospy.get_time() - StartTime) < duration: 
            continue

        rospy.loginfo(f'{duration} second(s) have elapsed, stopping the robot...')
        pub.publish(Twist())        #Sending empty twist message with all values at 0
    


    return service_response

rospy.init_node(f"{service_name}_server") 
my_service = rospy.Service(service_name, TimedMovement, callback_function) 
rospy.loginfo(f"the '{service_name}' Server is ready to be called...") 
rospy.spin() 
