#!/usr/bin/env python
"""
.. module:: get_last_target_service
   :platform: Unix
   :synopsis: Python module for assignment_2_2024
.. moduleauthor:: Paolo Laishram

Description:
    This node provides a service that returns the last target position set by the user.
    It performs the following functions:

    - Retrieving the Last Target

      - The node listens to the `/pos_vel` topic, which provides the robot's position and velocity.
      - It extracts the last target position (x, y) from the ROS parameters `/des_pos_x` and `/des_pos_y`, which are updated when the user sets a new target.

    - Providing the Last Target via Service

      - The node provides a service (`/get_last_target`) that returns the last target position (x, y) when called.
      - The service uses the `GetLastTarget` service, created specifically for this purpose, which includes the fields `last_target_x` and `last_target_y`.

    - Node Execution

      - The node continuously runs and listens for service calls.
      - When a service call is made, it responds with the last target's position.

Nodes:
    - `/get_last_target_service`
    
Services:
    - `/get_last_target` (assignment_2_2024/GetLastTarget): Returns last target position (x, y)

Subscribed Topics:
    - `/pos_vel` (assignment_2_2024/PositionVelocity): Robot position and velocity

"""

import rospy
import assignment_2_2024.msg
from assignment_2_2024.msg import PositionVelocity
from assignment_2_2024.srv import GetLastTarget, GetLastTargetResponse


def get_last_target(msg):
    """
    Callback function to update the last known target position.

    Args:
        msg (assignment_2_2024.msg.PositionVelocity): Message containing robot's position and velocity.
    """
    global last_des_x, last_des_y

    # get last target from ros parameters. 
    # they have been updated when the last target was entered by the user
    last_des_x = rospy.get_param('/des_pos_x')
    last_des_y = rospy.get_param('/des_pos_y')
    
    
def result_callback(s):
    """
    Service callback function to return the last known target position.

    Args:
        req (assignment_2_2024.srv.GetLastTargetRequest): Service request.

    Returns:
        GetLastTargetResponse: The last known target position (x, y).
    """
    global last_des_x, last_des_y 
    
    # store last target
    response = GetLastTargetResponse()
    response.last_target_x = last_des_x
    response.last_target_y = last_des_y
    
    return response
    	    

def last_target_service():
    """ Initializes the service node and starts the service. """
    rospy.init_node('get_last_target_service')
    rospy.loginfo("Last target node initialized")

    # SUBSCRIBER: Subscribe to the correct action goal topic
    rospy.Subscriber("/pos_vel", PositionVelocity, get_last_target)
    
    # SERVICE: Service to get the last target. It uses Last_target service type
    service = rospy.Service('get_last_target', GetLastTarget, result_callback)
    
    rospy.spin()

if __name__ == "__main__":
    last_target_service()
    
