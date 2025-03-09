#! /usr/bin/env python

## @package assignment1_rt2
# \file retrieve_last_goal_service.py
# \brief When called, returns the last goal set, if present.
# \author Alessandro Mangili
# \version 1.0
# \date 9/03/2025
#
# \details
#
# Subscribe to: <BR>
#   /reaching_goal/goal
#
# Service to: <BR>
#   /get_last_goal
#
# Description: <BR>
# The code implements a service that, when called, returns the last goal set by the user, if present, otherwise it returns an empty response.
# The function subscribes to the reaching_goal topic to get the last goal set.
# 


import rospy
from assignment1_rt2.msg import PlanningActionGoal
from assignment1_rt2.srv import Target, TargetResponse

last_target = None  # Global variable that contains the last goal set.

##
# \brief The callback function of the subscriber
# \param msg The message contains all the goal information
# 
# \return None
# 
# The function saves the last goal set, contained in the msg parameter, into the global variable
#
def goal_callback(msg):
    global last_target
    last_target = msg

##
# \brief The service function
# \param req The request contains all the information about the caller
#
# \return The information about the last goal set if present, otherwise, it returns an empty message
#
# The function handles the caller's request by sending them the last goal information, retrieving it from the global variable
# 
def handle_last_goal_request(req):
    global last_target
    if last_target is None:
        rospy.logwarn("[SERVICE NODE] No target available")
        return TargetResponse()
    target_info = last_target.goal.target_pose
    return TargetResponse(target_info)
    
##
# \brief The main function
#
# \return None
#
# The function handles the subscriber and the service to make the information about the last set goal available
#
def service_last_goal():
    rospy.init_node('retrieve_last_target')
    rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, goal_callback)
    rospy.Service('/get_last_goal', Target, handle_last_goal_request)
    rospy.loginfo("Service node started. Waiting for requests...")
    rospy.spin()

if __name__ == "__main__":
    try:
        service_last_goal()
    except rospy.ROSInterruptException:
        print("Program interrupted")
