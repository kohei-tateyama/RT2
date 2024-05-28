#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from assignment_2_2023.msg import Position,Goal
import assignment_2_2023
import actionlib
import actionlib.msg


"""
Publishes: '/goal_topic' (goal position)
            '/position' (position and velocity of the robot)
            '/reaching_goal/goal' (sends the new goal position to the bug_action_service)
            '/reaching_goal/cancel' (tell bug_action_service to cancel the goal)


Subscribes to: '/odom' (odometry data of the robot published by gazebo)
                '/reaching_goal/feedback' (feedback data of the robot published by bug_action_service)
                '/reaching_goal/status' (status data of the robot published by bug_action_service)
                '/reaching_goal/result' (result data of the robot published by bug_action_service)
"""
class Client():
    def __init__(self) -> None:
        # initialize node
        rospy.init_node("action_client_node", anonymous=True)


        # subscribe to rostopic /odom 
        # and receive robot position with custom message
        rospy.Subscriber('/odom', Odometry, self.callback_odom) # everytime odom is updated, callback_odom is called

        # set as action client to check if the targets are reached
        action_client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)

        # wait for the server to be available
        # action_client.wait_for_server()
        self.goal = assignment_2_2023.msg.PlanningGoal()
        self.goal_pub = rospy.Publisher('/goal_topic', Goal, queue_size = 10)
        self.goal_msg = Goal()

        # subsctribe to status to check if the target is reached
        rospy.Subscriber('/reaching_goal/status',String, self.callback_goal)
        

        while not rospy.is_shutdown():
            # get user input
            user_input = input("Enter new goal position x,y or 'c' for cancel:")
            print(user_input)

            # if user input is 'c' 
            if user_input == 'c':
                # cancel the goal
                action_client.cancel_goal()
                print("Goal cancelled")

            
            else:
                try: # else if user_input is 2 floats
                    # split the 2 floats into x,y
                    coordinates = user_input.split(',')
                    if len(coordinates) != 2:
                        print("ERROR! Enter the new goal position x,y")	

                    x, y = map(float, coordinates)

                    # set it as new goal position
                    self.goal.target_pose.pose.position.x = x
                    self.goal.target_pose.pose.position.y = y  
                
                    # set message to publish
                    
                    self.goal_msg.x = x
                    self.goal_msg.y = y

                    # publish newly set targets
                    
                    self.goal_pub.publish(self.goal_msg)                                    
                                
                    action_client.send_goal(self.goal)
                    # Print an error message if the input is not a float value
                except ValueError: # else if user_input is not 2 floats
                    print("ERROR! Enter the new goal position x,y")	
                
                


        

    def callback_odom(self, data):

        # Set the publisher for robot position
        pub = rospy.Publisher('/position', Position,  queue_size = 10)
        rate=rospy.Rate(1)

        # position
        position_ = data.pose.pose.position
        # velocity
        linear_velocity = data.twist.twist.linear

        # set the message to publish
        msg = Position()
        msg.x = position_.x
        msg.y = position_.y
        msg.vel_x = linear_velocity.x
        msg.vel_y = linear_velocity.y
        # publish the position and velocity with position.msg
        pub.publish(msg)


    def callback_goal(self, data):
        # get the status of the goal
        if(len(data.status_list)!=0):
            temp=data.status_list[0]
            temp2=str(temp)
            temp3 = temp2.split("\"")
            temp4 = temp3[2].split()
            if(temp4[1]=='3'):
                print("Reached Target")
            



if __name__ == '__main__':
    try:
        Client()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass