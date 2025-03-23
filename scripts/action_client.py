#! /usr/bin/env python

## @package assignment1_rt2
# \file action_client.py
# \brief The user can set or remove a specific goal to make the robot moves.
# \author Alessandro Mangili
# \version 1.0
# \date 9/03/2025
#
# \details
#
# Subscribes to: <BR>
#   * /odom topic for receives information on the robot's position and velocity
#
# Publishes to: <BR>
#   * /robot_information topic for publishing the robot's position and velocity
#
# Service: <BR>
#   * /reaching_goal to move the robot by sending actions to the action server
#
# Description: <BR>
# The code creates an action client that interacts with the user, allowing them to:
# 1. Set a new target goal
# 2. Delete the active goal
# 3. Exit the program
#
# The action client sends the user's request to the action server. If the request is to add a new target goal, the server adds it and initiates the robot's movement. 
# If the request is to cancel the active goal, the server checks for an active or pending goal, if one exists, it cancels it and notifies the action client of the deletion.
#

import rospy
import actionlib
from actionlib import GoalStatus
import assignment1_rt2.msg
from assignment1_rt2.msg import Robot_info
from nav_msgs.msg import Odometry

pub_position_vel = None # Global variable that contains the publisher information.

##
# \brief The callback function of the subscriber 
# \param msg The message contains the position and velocity information
#
# \return None
#
# The function retrieves the robot's position and velocity from the /odom topic and publishes this information to the /robot_information topic
#
def odom_callback(msg):    
    global pub_position_vel
    
    robot_info = Robot_info()
    robot_info.x = msg.pose.pose.position.x
    robot_info.y = msg.pose.pose.position.y
    robot_info.vel_x = msg.twist.twist.linear.x
    robot_info.vel_z = msg.twist.twist.angular.z

    pub_position_vel.publish(robot_info)

##
# \brief The callback function of the action client
# \param feedback The message contains the status of the active goal provided by the action server
#
# \return None
#
# The function checks whether the robot has reached the target by evaluating the feedback status, if it has, it alerts the user by printing a message on the console
#
def feedback_callback(feedback):     
    if (feedback.stat == "Target reached!"):
        print("")
        rospy.logwarn("Target reached\n{}\nStatus: {}\n".format(feedback.actual_pose, feedback.stat))
        print("Command (s=set goal, c=cancel goal, q=quit): ")

##
# \brief The function sets a new goal
# \param client The instance of the action client
# \param x The x coordinate of the target goal
# \param y The y coordinate of the target goal
#
# \return None
#
# The function sets the position of the target goal (x and y coordinates) using the provided parameters and send it to the action server
#
def send_goal(client, x, y):        
    goal = assignment1_rt2.msg.PlanningGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    
    goal.target_pose.pose.position.z = 0.0      
    goal.target_pose.pose.orientation.x = 0.0  
    goal.target_pose.pose.orientation.y = 0.0  
    goal.target_pose.pose.orientation.z = 0.0   
    goal.target_pose.pose.orientation.w = 0.0   

    client.send_goal(goal, feedback_cb=feedback_callback)
    rospy.loginfo("Goal sent")
    
##
# \brief The function cancels an active goal
# \param client The instance of the action client
#
# \return None
#
# The function cancels an active or pending goal by sending a request to the action server. If no active or pending goal exists, it prompts a warning message.
# If the cancellation request fails, the function prompt a warning message
#
def cancel_goal(client):    
    if client.get_state() in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
        rospy.loginfo("Cancelling current goal")
        client.cancel_goal()
        rospy.sleep(0.5)  # Timeout for forward the cancel
        state = client.get_state()
        if state in [GoalStatus.PREEMPTED, GoalStatus.RECALLED]:
            rospy.loginfo("Goal successfully cancelled")
        else:
            rospy.logwarn("Failed to cancel the goal")
    else:
        rospy.logwarn("No active goal to cancel.")

##
# \brief The input function
#
# \return The x and y coordinates of the target goal, converted to float
#
# The function prompts the user to enter the x and y coordinates of the target goal. If the user enters a non-numeric value, it notifies the error
# by displaying a warning message
#
def get_input():    
    while True:
        try:
            x = float(input("Enter the value for setting the x coordinate: "))
            y = float(input("Enter the value for setting the y coordinate: "))
            return x, y
        except ValueError:
            rospy.logwarn("Input not valid, enter only numbers!")

##
# \brief The main fuction
#
# \return None
#
# The function manages both a publisher and subscriber, and creates a client action to communicate the user's choise to the action server. The possible choise are:
#   - 's': send a new target goal
#   - 'c': cancel an active or pending goal
#   - 'q': exit the program
#
def main():    
    global pub_position_vel
    rospy.init_node('action_client')

    pub_position_vel = rospy.Publisher('/robot_information', Robot_info, queue_size=10)
    rospy.wait_for_message('/odom', Odometry)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    client = actionlib.SimpleActionClient('/reaching_goal', assignment1_rt2.msg.PlanningAction)
    rospy.loginfo("Waiting that the action server is avaible")
    client.wait_for_server()
    
    while not rospy.is_shutdown():
        rospy.loginfo_once("Enter 's' to set a goal, 'c' to cancel the current goal, 'q' to quit the action client or 'CTRL+C' for close all the simulation")
        answer = input("Command (s=set goal, c=cancel goal, q=quit, CTRL+C=exit all): ").strip().lower()
        if answer == 's':
            x, y = get_input()
            send_goal(client, x, y)
        elif answer == 'c':
            cancel_goal(client)
        elif answer == 'q':
            rospy.loginfo("Exiting from the action client")
            break
        else:
            rospy.logwarn("Invalid command. Please try again")
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Program interrupted")
