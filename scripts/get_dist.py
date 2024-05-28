#! /usr/bin/env python

## @package assignment_2_2023
# \file get_dist.py 
# \brief Service server for calculating the distance and average speed of the robot.
# \author Kohei Tateyama
# \version 0.1 
# \date 27/04/2024 
# \details
# This code subscribes to the robot's position and goal topics, and provides a service to calculate the distance between the robot and the target as well as the average velocity of the robot.

# Subscribes to: <BR>
# /goal_topic 
# /position 

# Services: <BR>
# /GetDist

import rospy
from assignment_2_2023.msg import Position, Goal
import math
from assignment_2_2023.srv import GetDist, GetDistResponse

"""
Subscribes to: 
    '/goal_topic' (Position of the goal published by action_client)
    '/position' (Position and the velocity of the robot, published by action_client)

Services: 
    '/GetDist' (Service for distance of the robot and the target, 
                average velocity of the robot)
"""

##
# \class GetDistance
# \brief Service server class for calculating the distance and average speed.
# \details
# This class initializes the ROS node, subscribes to the goal and position topics, and provides a service to return the distance to the goal and the average speed of the robot.

class GetDistance():
    def __init__(self) -> None:
        # Initialize the ROS node.
        rospy.init_node('get_dist', anonymous=True)

        # Initialize variables.
        self.robot = Position()
        self.goal = Goal()
        self.speedx = []
        self.speedy = []
        self.target_is_set = False
        
        # Subscribe to /goal_topic to get the goal position.
        rospy.Subscriber("/goal_topic", Goal, self.callback_goal)

        # Subscribe to /position to get the robot position and velocity.
        rospy.Subscriber("/position", Position, self.callback_robot)

        # Define the service /GetDist with a callback to handle requests.
        rospy.Service("GetDist", GetDist, self.callback_service)

    ##
    # \brief Callback for the robot's position and velocity.
    # \param data Position message containing the robot's position and velocity.
    # \return None
    def callback_robot(self, data):
        # Store the position and velocity in the variables.
        self.robot.x = data.x
        self.robot.y = data.y
        self.robot.vel_x = data.vel_x
        self.robot.vel_y = data.vel_y
        self.speedx.append(data.vel_x)
        self.speedy.append(data.vel_y)

    ##
    # \brief Callback for the goal position.
    # \param data Goal message containing the goal's position.
    # \return None
    def callback_goal(self, data):
        # Store the new goal position in the variables.
        self.goal.x = data.x
        self.goal.y = data.y
        if not self.target_is_set:
            self.target_is_set = True

    ##
    # \brief Callback for the /GetDist service.
    # \param req Service request (not used).
    # \return GetDistResponse with the distance and average speed.
    def callback_service(self, req):
        # Calculate the distance and average speed, and return the response when the service is called.
        response = GetDistResponse()
        if self.target_is_set:
            response.dist = math.sqrt((self.goal.x - self.robot.x)**2 + (self.goal.y - self.robot.y)**2)
            print("Target is set")
        else:
            print("None")
            response.dist = 100.0
        if len(self.speedx) != 0:
            response.av_speed_x = sum(self.speedx) / len(self.speedx)
            response.av_speed_y = sum(self.speedy) / len(self.speedy)
        return response

if __name__ == '__main__':
    try:
        # Create an instance of the server and spin to keep it running.
        GetDistance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
