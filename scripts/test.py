import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from assignment_2_2023.msg import Position, Goal
import assignment_2_2023
import actionlib
import actionlib.msg
import ipywidgets as widgets
import numpy as np
import matplotlib.pyplot as plt

# Initialize global variables
goal_x, goal_y = 0, 0
position_x, position_y = [], []


def position_callback(pos_msg):
    global position_x, position_y  # Declare as global to modify the global variables
    position_x.append(pos_msg.x)
    position_y.append(pos_msg.y)
    print(pos_msg.x,pos_msg.y)

def goal_callback(goal_msg):
    global goal_x, goal_y  # Declare as global to modify the global variables
    goal_x = goal_msg.x
    goal_y = goal_msg.y
    print("Received goal message:")
    print(goal_msg)
    print(f"Goal: x={goal_x}, y={goal_y}")

def main():
    rospy.init_node('jupyter_node') 
    print("Node initialized")

    sub = rospy.Subscriber('/position', Position, position_callback, queue_size=10)
    print("Subscribed to /position topic")

    sub2 = rospy.Subscriber('/goal_topic', Goal, goal_callback, queue_size=10)
    print("Subscribed to /goal_topic")

    # Keep the node running
    rospy.spin()
    print("ROS spin started")

if __name__ == '__main__':
    main()
