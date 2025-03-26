#!/usr/bin/env python
"""
.. module:: action_client
   :platform: Unix
   :synopsis: A ROS node action client to send goals to a robot and monitor its state.

.. moduleauthor:: Lorenzo Penna

This module contains the ActionClientNode class which is a ROS node that sends goals to a robot and monitors its state.
"""

import rospy
import actionlib
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from assignment2.msg import PositionVelocity  # Custom message
from assignment2.srv import GetLastGoal, GetLastGoalResponse
from tf import transformations

class ActionClientNode:
    """
    The ActionClientNode class initializes the ROS node, sets up the action client, publishers, and subscribers.

    Attributes:
        client (SimpleActionClient): The action client for sending goals.
        pub_position_velocity (Publisher): Publisher for the custom PositionVelocity message.
        sub_odom (Subscriber): Subscriber to the /odom topic to get the robot's state.
        current_pose (Pose): The current pose of the robot.
        current_velocity (Twist): The current velocity of the robot.
    """
    def __init__(self):
        """
        Initializes the ActionClientNode, sets up the action client, publishers, and subscribers.

        This method initializes the ROS node, sets up the action client to communicate with the action server,
        and creates publishers and subscribers to handle robot state and custom messages.
        """
        # Initialize the ROS node
        rospy.init_node('action_client_node')

        # Set up Action Client
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        self.client.wait_for_server()
        rospy.loginfo("Action server ready")

        # Set up Publisher for custom message
        self.pub_position_velocity = rospy.Publisher('/position_velocity', PositionVelocity, queue_size=10)

        # Subscribe to /odom topic
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Robot state
        self.current_pose = Pose()
        self.current_velocity = Twist()

    def odom_callback(self, msg):
        """
        Callback function for the odometry subscriber. Updates the robot's position and velocity.

        Args:
            msg (Odometry): The odometry message received from the topic.

        This function extracts the robot's current pose and velocity from the odometry message and publishes
        them as a custom PositionVelocity message.
        """
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

        # Publish position and velocity as custom message
        custom_msg = PositionVelocity()
        custom_msg.x = self.current_pose.position.x
        custom_msg.y = self.current_pose.position.y
        custom_msg.vel_x = self.current_velocity.linear.x
        custom_msg.vel_z = self.current_velocity.angular.z
        self.pub_position_velocity.publish(custom_msg)

    def send_goal(self, x, y):
        """
        Sends a goal to the action server.

        Args:
            x (float): The x-coordinate of the goal.
            y (float): The y-coordinate of the goal.

        This function creates a PlanningGoal message with the specified coordinates and sends it to the action server.
        """
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)
        rospy.loginfo(f"Goal sent: x={x}, y={y}")

    def cancel_goal(self):
        """
        Cancels the current goal.

        This function sends a cancel request to the action server to stop the robot from reaching the current goal.
        """
        self.client.cancel_goal()
        rospy.loginfo("Goal cancelled")

    def feedback_callback(self, feedback):
        """
        Callback function for feedback from the action server.

        Args:
            feedback: The feedback message from the action server.

        This function logs feedback from the action server, such as the robot's current state or progress.
        """
        rospy.loginfo(f"Feedback: {feedback.stat}") 

    def run(self):
        """
        Runs the main loop of the ActionClientNode, waiting for user input to send or cancel goals.

        This function continuously waits for user input to:
        - Send a new goal by entering coordinates.
        - Cancel the current goal by entering 'c'.
        - Retrieve the previous goal by entering 'prev'.

        It handles user input and interacts with the action server and the `get_last_goal` service.
        """
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rospy.loginfo("Waiting for input to send a goal...")
            try:
                user_input = input("Enter 'x y' to send a goal, 'c' to cancel, or 'prev' to retrieve the previous goal: ")
                if user_input.lower() == 'c':
                    self.cancel_goal()
                elif user_input.lower() == 'prev':
                    rospy.wait_for_service('get_last_goal')
                    try:
                        get_last_goal = rospy.ServiceProxy('get_last_goal', GetLastGoal)
                        response = get_last_goal()
                        if response.last_goal:
                            rospy.loginfo(f"Previous goal: x={response.last_goal[0]}, y={response.last_goal[1]}")
                        else:
                            rospy.loginfo("No previous goal available.")
                    except rospy.ServiceException as e:
                        rospy.logerr(f"Service call failed: {e}")
                else:
                    x, y = map(float, user_input.split())
                    self.send_goal(x, y)
            except Exception as e:
                rospy.logerr(f"Error: {e}")
            rate.sleep()

if __name__ == "__main__":
    node = ActionClientNode()
    node.run()

