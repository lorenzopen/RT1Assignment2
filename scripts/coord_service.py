#!/usr/bin/env python

import rospy
from assignment_2_2024.msg import PlanningActionGoal
from assignment2.srv import GetLastGoal, GetLastGoalResponse

class GoalServer:
    def __init__(self):
        rospy.init_node('goal_server')
        self.last_goal = []
        self.sub = rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, self.goal_callback)
        self.srv = rospy.Service('get_last_goal', GetLastGoal, self.handle_get_last_goal)
        rospy.spin()

    def goal_callback(self, msg):
        position = msg.goal.target_pose.pose.position
        rospy.loginfo(f"Received new goal: x={position.x}, y={position.y}")
        self.last_goal = [position.x, position.y]

    def handle_get_last_goal(self, req):
        return GetLastGoalResponse(last_goal=self.last_goal)

if __name__ == '__main__':
    goal_server = GoalServer()