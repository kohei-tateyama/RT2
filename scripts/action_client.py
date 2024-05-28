#! /usr/bin/env python
## @package assignment_2_2023
# \file action_client.py 
# \brief Controller for the turtlesim 
# \author Kohei Tateyama
# \version 0.1 
# \date 27/04/2024 
# \details
# This code is designed to control the turtlesim by setting goals and monitoring its position and status.

# Subscribes to: <BR>
# /odom
# /reaching_goal/feedback
# /reaching_goal/status
# /reaching_goal/result

# Publishes to: <BR>
# /goal_topic
# /position
# /reaching_goal/goal
# /reaching_goal/cancel

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from assignment_2_2023.msg import Position, Goal
import assignment_2_2023
import actionlib
import actionlib.msg

##
# \class Client
# \brief Action client for controlling the robot.
# \details
# This class initializes the action client, sets up subscribers and publishers, and handles user input for setting or cancelling goals.

class Client():
    def __init__(self) -> None:
        #  Initialize the ROS node.
        rospy.init_node("action_client_node", anonymous=True)

        #  Subscribe to /odom topic to receive robot position.
        rospy.Subscriber('/odom', Odometry, self.callback_odom)

        #  Set up action client to check if the targets are reached.
        action_client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)

        #  Wait for the action server to be available.
        action_client.wait_for_server()

        #  Initialize goal and publisher.
        self.goal = assignment_2_2023.msg.PlanningGoal()
        self.goal_pub = rospy.Publisher('/goal_topic', Goal, queue_size=10)
        self.goal_msg = Goal()

        #  Subscribe to /reaching_goal/status to check if the target is reached.
        rospy.Subscriber('/reaching_goal/status', String, self.callback_goal)

        #  Main loop for user input to set or cancel goals.
        while not rospy.is_shutdown():
            user_input = input("Enter new goal position x,y or 'c' for cancel:")
            print(user_input)

            if user_input == 'c':
                #  Cancel the goal if user inputs 'c'.
                action_client.cancel_goal()
                print("Goal cancelled")
            else:
                try:
                    coordinates = user_input.split(',')
                    if len(coordinates) != 2:
                        print("ERROR! Enter the new goal position x,y")
                        continue

                    x, y = map(float, coordinates)

                    #  Set new goal position.
                    self.goal.target_pose.pose.position.x = x
                    self.goal.target_pose.pose.position.y = y

                    #  Set message to publish.
                    self.goal_msg.x = x
                    self.goal_msg.y = y

                    #  Publish newly set targets.
                    self.goal_pub.publish(self.goal_msg)

                    #  Send goal to the action server.
                    action_client.send_goal(self.goal)
                except ValueError:
                    print("ERROR! Enter the new goal position x,y")

    ##
    # \brief Callback for odometry.
    # \param data Odometry data from /odom topic.
    # \return None

    def callback_odom(self, data):
        #  Set the publisher for robot position.
        pub = rospy.Publisher('/position', Position, queue_size=10)
        rate = rospy.Rate(1)

        #  Get position and velocity from odometry data.
        position_ = data.pose.pose.position
        linear_velocity = data.twist.twist.linear

        #  Set the message to publish.
        msg = Position()
        msg.x = position_.x
        msg.y = position_.y
        msg.vel_x = linear_velocity.x
        msg.vel_y = linear_velocity.y

        #  Publish the position and velocity.
        pub.publish(msg)

    ##
    # \brief Callback for goal status.
    # \param data Status data from /reaching_goal/status topic.
    # \return None

    def callback_goal(self, data):
        #  Get the status of the goal.
        if len(data.status_list) != 0:
            temp = data.status_list[0]
            temp2 = str(temp)
            temp3 = temp2.split("\"")
            temp4 = temp3[2].split()
            if temp4[1] == '3':
                print("Reached Target")

if __name__ == '__main__':
    try:
        Client()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
