#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
import assignment_2_2024.msg
from assignment_2_2024.msg import PositionVelocity
from assignment_2_2024.msg import PlanningAction, PlanningGoal, PlanningResult, PlanningActionResult
from std_srvs.srv import SetBool
from actionlib_msgs.msg import GoalStatus


pub = None
first_start = 0

def publisher_node(msg):
    # Create a publisher
    global pub

    # Get the actual position and velocity
    actual_pos = msg.pose.pose.position
    actual_vel_linear = msg.twist.twist.linear
    actual_vel_angular = msg.twist.twist.angular

    # Create a Vel message
    my_pos_and_vel = PositionVelocity()
    my_pos_and_vel.x = actual_pos.x
    my_pos_and_vel.y = actual_pos.y
    my_pos_and_vel.vel_x = actual_vel_linear.x
    my_pos_and_vel.vel_z = actual_vel_angular.z

    pub.publish(my_pos_and_vel)
    
def client():
    #global reached
    global first_start

    # Create an action client
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2024.msg.PlanningAction)
    client.wait_for_server()

    while not rospy.is_shutdown():
        rospy.loginfo("\nPRESS: \n y - to set a new goal \n c - to cancel last goal \n")

        command = input("Choice: ")

        # Get the actual goal
        goal = assignment_2_2024.msg.PlanningGoal()
        goal.target_pose.pose.position.x = rospy.get_param('/des_pos_x')
        goal.target_pose.pose.position.y = rospy.get_param('/des_pos_y')
        
        # Handling inputs
        
        # the user wants to set a new goal
        if command == 'y':
            # get the input and check validity
            while True:
                input_x = input("Enter desired position x: ")
                try:
                    input_x = float(input_x)
                    break

                except:
                    print("Invalid input, enter a number")

            while True:
                input_y = input("Enter desired position y: ")
                try:
                    input_y = float(input_y)
                    break

                except:
                    print("Invalid input, enter a number")

            # Update ros parameters
            rospy.set_param('/des_pos_x', input_x)
            rospy.set_param('/des_pos_y', input_y)

            # Set goal parameters
            goal.target_pose.pose.position.x = input_x
            goal.target_pose.pose.position.y = input_y

            # send goal to the service
            client.send_goal(goal)
            rospy.loginfo("Inserted goal: des_x = %f, des_y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
            
            # update variable to handle cancel operation as soon as simulation starts
            first_start = 1 


        # The user wants to cancel the goal
        elif command == 'c':
            
            # check variable to handle cancel operation as soon as simulation starts
            if first_start == 0:
                rospy.loginfo("Goal has not been already set, cannot cancel it")
                
            elif client.get_state() == actionlib.GoalStatus.ACTIVE:

                    # if goal not reached and goal in active state cancel the goal
                    client.cancel_goal()
                    rospy.loginfo("Goal cancelled")    
            
            else:
                rospy.loginfo("Goal already reached, cannot cancel it")

        # The user digits neither y nor c
        else:
            rospy.loginfo("Invalid input")


def main():
    rospy.init_node('user_input')

    global pub

    # PUBLISHER: send a message which contains two parameters (velocity and position)
    pub = rospy.Publisher("/pos_vel", PositionVelocity, queue_size=1)

    # SUBSCRIBER: get from "Odom" two parameters (velocity and position)
    sub_from_Odom = rospy.Subscriber("/odom", Odometry, publisher_node)

    # Calling the action client
    client()

if __name__ == '__main__':
    main()
