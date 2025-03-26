#!/usr/bin/env python
"""
.. module:: coord_service
   :platform: Unix
   :synopsis: ROS service that returns the last goal received by the goal server.

.. moduleauthor:: Lorenzo Penna

This module contains the ROS service that returns the last goal received by the goal server.
"""

import rospy
from assignment_2_2024.msg import PlanningActionGoal
from assignment2.srv import GetLastGoal, GetLastGoalResponse

class GoalServer:
    """
    The GoalServer class initializes the ROS node, subscribes to the goal topic, and provides a service to get the last goal.

    Attributes:
        last_goal (list): A list containing the x and y coordinates of the last goal received by the goal server.
        sub (Subscriber): A ROS subscriber to the `/reaching_goal/goal` topic, which listens for new goals.
        srv (Service): A ROS service named `get_last_goal` that handles requests for the last goal.
    """
    def __init__(self):
        """
        Initializes the GoalServer node, subscriber, and service.

        This method sets up the ROS node, initializes the `last_goal` attribute, subscribes to the `/reaching_goal/goal` topic, 
        and advertises the `get_last_goal` service. The node will continue running until it is shut down.
        """
        rospy.init_node('goal_server')
        self.last_goal = []
        self.sub = rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, self.goal_callback)
        self.srv = rospy.Service('get_last_goal', GetLastGoal, self.handle_get_last_goal)
        rospy.spin()

    def goal_callback(self, msg):
        """
        Callback function for the goal subscriber. Updates the last goal received.

        Args:
            msg (PlanningActionGoal): The goal message received from the `/reaching_goal/goal` topic.

        This function extracts the x and y coordinates of the goal from the `msg` and updates the `last_goal` attribute.
        """
        position = msg.goal.target_pose.pose.position
        rospy.loginfo(f"Received new goal: x={position.x}, y={position.y}")
        self.last_goal = [position.x, position.y]

    def handle_get_last_goal(self, req):
        """
        Handles requests for the last goal received.

        Args:
            req (GetLastGoal): The service request.

        Returns:
            GetLastGoalResponse: A response containing the last goal as a list of two floats [x, y].
                                 If no goal has been received , the list will be empty.

        This function retrieves the last goal stored in the `last_goal` attribute and returns it as part of the service response.
        """
        return GetLastGoalResponse(last_goal=self.last_goal)

if __name__ == '__main__':
    goal_server = GoalServer()