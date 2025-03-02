#! /usr/bin/env python

import rospy
from assignment2_rt.msg import PlanningActionGoal
from assignment2_rt.srv import Target, TargetResponse

last_target = None

def goal_callback(msg):
    ##
    # \brief The function is a callback that save the last set goal
    # \param msg The message contains all the goal information
    #
    
    global last_target
    last_target = msg
    #rospy.loginfo(f"Received a new goal: {last_target.goal.target_pose}")

def handle_last_goal_request(req):
    ##
    # \brief The function handle the request of a caller sending to him the last goal information 
    # \param req The request contains all the information about the caller
    # \return The information about the last set goal, if there was 
    # 
    
    global last_target
    if last_target is None:
        rospy.logwarn("[SERVICE NODE] No target available")
        return TargetResponse()
    target_info = last_target.goal.target_pose
    return TargetResponse(target_info)
    
def service_last_goal():
    ##
    # \brief The main function that handle the subscriber and the service to make the information of the last set goal available
    #
    
    rospy.init_node('retrieve_last_target')
    # Subscriber to receive the set goals
    rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, goal_callback)
    # Service that returns the last set goal
    rospy.Service('/get_last_goal', Target, handle_last_goal_request)
    rospy.loginfo("Service node started. Waiting for requests...")
    rospy.spin()

if __name__ == "__main__":
    try:
        service_last_goal()
    except rospy.ROSInterruptException:
        print("Program interrupted")
