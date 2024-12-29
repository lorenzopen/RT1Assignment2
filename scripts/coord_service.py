#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

class CoordNode:

    def last_g_clbk(self, msg):
        rospy.loginfo("Last goal entered: %s", msg.data)
        self.last_goal = msg.data

    def __init__(self):
        rospy.init_node('coord_server_node')
        
        self.last_goal = None
        self.sub = rospy.Subscriber('/last_goal', Float64MultiArray, self.last_g_clbk)    

    def run(self):
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            user_input = input("Enter 'prev' for previous coordinates: ")
            
            if user_input.lower() == 'prev':
                if self.last_goal:
                    rospy.loginfo("Previous coordinates: %s", self.last_goal)
                else:
                    rospy.loginfo("No previous coordinates available.")
            rate.sleep()

if __name__ == "__main__":
    node = CoordNode()
    node.run()
