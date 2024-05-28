#! /usr/bin/env python

## @package assignment_2_2023
# \file last_target.py 
# \brief Service server for getting the last target position in turtlesim 
# \author Kohei Tateyama
# \version 0.1 
# \date 27/04/2024 
# \details
# This code is designed to provide the last target position set for the turtlesim by subscribing to the goal topic and offering a service to get the last target.

# Subscribes to: <BR>
# /goal_topic

# Services: <BR>
# /LastTarget

import rospy
from assignment_2_2023.msg import Goal
from assignment_2_2023.srv import LastTarget, LastTargetResponse

##
# \class GetLastTargetServer
# \brief Service server class for providing the last target position.
# \details
# This class initializes the ROS node, subscribes to the goal topic, and provides a service to return the last target position.
class GetLastTargetServer:
    def __init__(self) -> None:
        # Initialize the ROS node.
        rospy.init_node('LastTarget', anonymous=True)

        # Initialize variables.
        self.last_target = Goal()
        self.target_is_set = False

        # Subscribe to /goal_topic to get the goal position.
        rospy.Subscriber('/goal_topic', Goal, self.callback)
        
        # Define the service 'LastTarget' with a callback to handle requests.
        self.service = rospy.Service('LastTarget', LastTarget, self.callback_service)

    ##
    # \brief Callback for the goal topic subscription.
    # \param data Goal data from /goal_topic.
    # \return None
    def callback(self, data):
        # Update the last target with new goal data.
        self.last_target.x = data.x
        self.last_target.y = data.y

        # If it's the first target set, update the flag.
        if not self.target_is_set:
            self.target_is_set = True

    ##
    # \brief Callback for the LastTarget service.
    # \param req Service request (not used).
    # \return LastTargetResponse with the last target position.
    def callback_service(self, req):
        response = LastTargetResponse()
        if self.target_is_set:
            # If a target is set, return the last target position.
            response.x = self.last_target.x
            response.y = self.last_target.y
            print(response)
        else:
            # If no target is set, log an info message.
            rospy.loginfo("No targets yet")
        return response

if __name__ == '__main__':
    try:
        # Create an instance of the server and spin to keep it running.
        GetLastTargetServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
