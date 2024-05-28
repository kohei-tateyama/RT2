#! /usr/bin/env python

import rospy
from assignment_2_2023.msg import Goal
from assignment_2_2023.srv import LastTarget, LastTargetResponse


"""
Subscribes : 'goal_topic' (goal position published by action_client)
Services : 'LastTarget' (service for last target)
"""

class GetLastTargetServer:
    def __init__(self) -> None:
        #initialize node
        rospy.init_node('LastTarget', anonymous=True)

        #initialize variables
        self.last_target=Goal()
        self.target_is_set = False

        # subscribe to /goal_topic to get the goal position
        rospy.Subscriber('/goal_topic', Goal, self.callback)
        self.service = rospy.Service('LastTarget',LastTarget, self.callback_service)
        # returns the last target sent by the user

    def callback(self,data): #when the new goal is set
        self.last_target.x = data.x
        self.last_target.y = data.y

        #if its the first target
        if not self.target_is_set:
            self.target_is_set=True

    def callback_service(self,req): #when service is called
        response = LastTargetResponse()
        if self.target_is_set: # if target is set
            response.x = self.last_target.x
            response.y = self.last_target.y
            print(response)
        else:
            rospy.loginfo("No targets yet")
        return response


if __name__ == '__main__':
    try:
        GetLastTargetServer()

        rospy.spin()
    except ROSInterruptException:
        pass