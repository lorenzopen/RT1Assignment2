# Assignment 2 Part 1

To start the whole simulation after cloned the pkg:

-->  roslaunch assignment2 assignment2.launch

## Action Client Node

This script implements a ROS client node that interacts with a `/reaching_goal` action server

**Functionality:**

* **Sends Goals:** Allows sending goals (target poses) to the action server. Users can specify x and y coordinates through console input.
* **Goal Feedback:** Receives feedback from the action server about the goal execution status.
* **Goal Cancellation:** Enables canceling the currently active goal by enter 'c'.
* **Custom Message Publishing:** Publishes a custom message (`PositionVelocity`) containing the robot's current position and velocity data.
* **Retrieving Last Goal :** a service named `get_last_goal`, the node can retrieve and display the previously sent goal by enter 'prev'. It also is possible using rqt and call the service  `get_last_goal`.


## Last Goal Server

**Functionality:**

* **Subscribes to `/reaching_goal/goal`:** Listens for incoming goal messages.
* **Stores the Last Goal:** Stores the x and y coordinates of the most recently received goal.
* **Provides a Service:** Offers a service called `get_last_goal` that allows other nodes to retrieve the coordinates of the last sent goal.
