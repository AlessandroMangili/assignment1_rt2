#! /usr/bin/env python

import rospy
import actionlib
from actionlib import GoalStatus
import assignment2_rt.msg
from assignment2_rt.msg import Robot_info
from nav_msgs.msg import Odometry

def odom_callback(msg):
    ##
    # \brief The function is a callback that retrieve the position and velocity from the /odom topic 
    # \param msg Is the callback message containing the position and velocity informations
    #
    
    global pub_position_vel
    
    robot_info = Robot_info()
    robot_info.x = msg.pose.pose.position.x
    robot_info.y = msg.pose.pose.position.y
    robot_info.vel_x = msg.twist.twist.linear.x
    robot_info.vel_z = msg.twist.twist.angular.z

    pub_position_vel.publish(robot_info)

def feedback_callback(feedback):
    ##
    # \brief The function is a callback that checks whether the robot has reached the target
    # \param feedback Is the message containing the information sent by the action server
    #
      
    if (feedback.stat == "Target reached!"):
        print("")
        rospy.logwarn("Target reached\n{}\nStatus: {}\n".format(feedback.actual_pose, feedback.stat))
        print("Command (s=set goal, c=cancel goal, q=quit): ")

def send_goal(client, x, y):   
    ##
    # \brief The function set the position of the target goal and send it to the action server
    # \param client The instance of the action client
    # \param x The x coordinate of the target goal
    # \param y The y coordinate of the target goal
    #
     
    # Creates a goal to send to the action server.
    goal = assignment2_rt.msg.PlanningGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    
    goal.target_pose.pose.position.z = 0.0      # Not supported by two wheels's robot
    goal.target_pose.pose.orientation.x = 0.0   # Not supported by the action server
    goal.target_pose.pose.orientation.y = 0.0   # Not supported by the action server
    goal.target_pose.pose.orientation.z = 0.0   # Not supported by the action server
    goal.target_pose.pose.orientation.w = 0.0   # Not supported by the action server

    # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb=feedback_callback)
    rospy.loginfo("Goal sent")
    
    # Waits for the server to finish performing the action.
    #client.wait_for_result()

    # Prints out the result of executing the action
    #return client.get_result()
    
def cancel_goal(client):
    ##
    # \brief The function is used to cancel a active goal if there it is, otherwise it will prompt a message error
    # \param client The instance of the action client
    #
    
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

def get_input():
    ##
    # \brief The function asks to the user to enter the x and y coordinates of the target goal
    # \return The x and y coordinates converted as float
    
    while True:
        try:
            x = float(input("Enter the value for setting the x coordinate: "))
            y = float(input("Enter the value for setting the y coordinate: "))
            return x, y
        except ValueError:
            rospy.logwarn("Input not valid, enter only numbers!")

def main():
    ##
    # \brief The main fuction handles the publisher and subscriber and creates a client action to communicate the user's choise to the action server, that could be send goal, cancel goal or exit the program.
    #
    
    global pub_position_vel
    rospy.init_node('action_client')

    # Create the publisher for postion and velocity
    pub_position_vel = rospy.Publisher('/robot_information', Robot_info, queue_size=10)
    # Wait for the arrival of the first message on that topic
    rospy.wait_for_message('/odom', Odometry)
    # Subscribe to /odom topic for retrieve the data from the robot
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    # Creates the SimpleActionClient, passing the type of the action (PlanningAction) to the constructor
    client = actionlib.SimpleActionClient('/reaching_goal', assignment2_rt.msg.PlanningAction)
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