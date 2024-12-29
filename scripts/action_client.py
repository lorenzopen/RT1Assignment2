#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from assignment2.msg import PositionVelocity  # Custom message
from tf import transformations

class ActionClientNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('action_client_node')

        # Set up Action Client
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        self.client.wait_for_server()
        rospy.loginfo("Action server ready")

        # Set up Publisher for custom message
        self.pub_position_velocity = rospy.Publisher('/position_velocity', PositionVelocity, queue_size=10)

        # Sub to /odom topic
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # robot state
        self.current_pose = Pose()
        self.current_velocity = Twist()

    #Callback to update robot's position and velocity.
    def odom_callback(self, msg):        
        
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

        # Publish position and velocity as custom message
        custom_msg = PositionVelocity()
        custom_msg.x = self.current_pose.position.x
        custom_msg.y = self.current_pose.position.y
        custom_msg.vel_x = self.current_velocity.linear.x
        custom_msg.vel_z = self.current_velocity.angular.z
        self.pub_position_velocity.publish(custom_msg)
    #Send a goal
    def send_goal(self, x, y):
        
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)
        #rospy.loginfo(f"Goal sent: x={x}, y={y}")
    
        #publish last goal entered
        self.last_goal = rospy.Publisher('/last_goal', Float64MultiArray, queue_size=10)
        self.last_goal.publish(data=[x, y])
    	
    #cancel goal
    def cancel_goal(self):
       
        self.client.cancel_goal()
        rospy.loginfo("Goal cancelled")
    
    
    def feedback_callback(self, feedback):
        
        rospy.loginfo(f"Feedback: {feedback.stat}, Current Pose: {feedback.actual_pose}")

    def run(self):
        
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rospy.loginfo("Waiting for input to send a goal...")
            try:
                
                user_input = input("Enter 'x y' to send a goal or 'c' to cancel: ")
                if user_input.lower() == 'c':
                    self.cancel_goal()
                else:
                    x, y = map(float, user_input.split())
                    self.send_goal(x, y)
            except Exception as e:
                rospy.logerr(f"Invalid input: {e}")
            rate.sleep()

if __name__ == "__main__":
    node = ActionClientNode()
    node.run()

