#! /usr/bin/env python

## @package assignment_2_2023
# \file wall_follow_service.py 
# \brief Implements wall following behavior for a robot.
# \author Kohei Tateyama
# \version 0.1 
# \date 27/04/2024 
# \details
# This code subscribes to laser scan data and publishes velocity commands to follow walls. It uses a service to switch the wall following behavior on and off.

# Subscribes to: <BR>
# /scan

# Publishes to: <BR>
# /cmd_vel

# Services: <BR>
# /wall_follower_switch

# Import necessary ROS packages and messages
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import *

import math

# Initialize global variables
active_ = False
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

##
# \brief Service callback to switch the wall follower on or off.
# \param req Service request containing the activation state.
# \return SetBoolResponse indicating success.
def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

##
# \brief Callback for laser scan data.
# \param msg LaserScan message containing the distance measurements.
# \return None
def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }

    take_action()

##
# \brief Change the state of the state machine.
# \param state New state to be set.
# \return None
def change_state(state):
    global state_, state_dict_
    if state is not state_:
        rospy.loginfo('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state

##
# \brief Take appropriate action based on the regions data.
# \return None
def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''

    d0 = 1
    d = 1.5

    if regions['front'] > d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

##
# \brief Find the wall by moving forward and turning slightly.
# \return Twist message with linear and angular velocities.
def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3
    return msg

##
# \brief Turn left in place.
# \return Twist message with angular velocity.
def turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    return msg

##
# \brief Follow the wall by moving forward.
# \return Twist message with linear velocity.
def follow_the_wall():
    msg = Twist()
    msg.linear.x = 0.5
    return msg

##
# \brief Main function to initialize the node and control loop.
# \return None
def main():
    global pub_, active_

    rospy.init_node('reading_laser')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        else:
            msg = Twist()
            if state_ == 0:
                msg = find_wall()
            elif state_ == 1:
                msg = turn_left()
            elif state_ == 2:
                msg = follow_the_wall()
            else:
                rospy.logerr('Unknown state!')

            pub_.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    main()
