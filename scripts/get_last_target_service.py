#!/usr/bin/env python

import rospy
import assignment_2_2024.msg
from assignment_2_2024.msg import PositionVelocity
from assignment_2_2024.srv import GetLastTarget, GetLastTargetResponse


def get_last_target(msg):
    global last_des_x, last_des_y

    # get last target from ros parameters. 
    # they have been updated when the last target was entered by the user
    last_des_x = rospy.get_param('/des_pos_x')
    last_des_y = rospy.get_param('/des_pos_y')
    
    
def result_callback(s):
    global last_des_x, last_des_y 
    
    # store last target
    response = GetLastTargetResponse()
    response.last_target_x = last_des_x
    response.last_target_y = last_des_y
    
    return response
    	    

def last_target_service():
    rospy.init_node('get_last_target_service')
    rospy.loginfo("Last target node initialized")

    # SUBSCRIBER: Subscribe to the correct action goal topic
    rospy.Subscriber("/pos_vel", PositionVelocity, get_last_target)
    
    # SERVICE: Service to get the last target. It uses Last_target service type
    service = rospy.Service('get_last_target', GetLastTarget, result_callback)
    
    rospy.spin()

if __name__ == "__main__":
    last_target_service()
    
