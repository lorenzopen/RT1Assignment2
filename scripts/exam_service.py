#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from assignment_2_2024.msg import PlanningActionFeedback
from assignment2.srv import GetDistanceToGoal, GetDistanceToGoalResponse
from std_msgs.msg import Bool

robot_position = None
last_goal = None
warning_pub = None

def odom_callback(msg):
    global robot_position
    robot_position = msg.pose.pose.position

def goal_callback(msg):
    global last_goal
    last_goal = msg.feedback.actual_pose.pose.position

def handle_get_distance_to_goal(req):
    if last_goal is None or robot_position is None:
        return GetDistanceToGoalResponse(0.0)
    
    distance = math.sqrt((last_goal.x - robot_position.x) ** 2 + (last_goal.y - robot_position.y) ** 2)
    return GetDistanceToGoalResponse(distance)

def reached_callback(msg):
    if msg.feedback.state == 2:   #in bug_as.py state 2 means reached 
        rospy.loginfo("reached")

def laser_callback(msg):
    global warning_pub

    min_distance = min(msg.ranges)
    if min_distance < 1.0:
        warning_pub.publish(Bool(data=True))
    else:
        warning_pub.publish(Bool(data=False))

def main():
    global warning_pub

    rospy.init_node('exam')

    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/reaching_goal/feedback', PlanningActionFeedback, reached_callback)
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    
    warning_pub = rospy.Publisher('/warning', Bool, queue_size=10)
    
    rospy.Service('get_distance_to_goal', GetDistanceToGoal, handle_get_distance_to_goal)

    rospy.spin()

if __name__ == '__main__':
    main()
