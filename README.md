# RT1 Assignment 2 - Part 1

This package implements the functionality for controlling a robot in a simulation environment. It includes two main nodes: an action client (user_input.py) and a service node (get_last_target_service.py). 

The package allows the user to set a target position for the robot, cancel the target, and retrieve the last set target coordinates. It also publishes the robot's position and velocity as a custom message.

## Package Overview
1. **User Input Node (`user_input.py`)**:
   - This node allows the user to input a target position (x, y) for the robot and send it as a goal to the action server.
   - The user can also cancel the last goal.
   - The node publishes the robot's position and velocity to the `/pos_vel` topic.

2. **Get Last Target Service Node (`get_last_target_service.py`)**:
   - This node provides a service to return the last target position set by the user.
   - The service is called via `rosservice call /get_last_target`, and it returns the x and y coordinates of the last target.

3. **Go to Point Service Node (`go_to_point_service.py`)**:
   - This implements a finite state machine that controls whether the robot behaves correctly and lets it move towards a specified point.
   - It also checks if the robot successfully reaches the goal.

4. **Wall Follower Service Node (`wall_follow_service.py`)**:
   - This node implements a service to make the robot follow a wall in the simulation environment.
   - It processes data to detect obstacles and avoid them: if an obstacle is detected on the front, the node makes the robot rotate until no obstacles are found.

5. **Bug Action Server (`bug_as.py`)**:
   - This node implements an action server that handles the robot's movement towards a target point.
   - Using a function (change_state), it switches between `go_to_point` and `wall_follow` behaviours.

### Custom messages and services:
- **PositionVelocity.msg**: A custom message containing the robot's position (x, y) and velocity (vel_x, vel_z).
- **GetLastTarget.srv**: A custom service that returns the last target coordinates (last_target_x, last_target_y).

### Launch File:
The package includes a launch file (`assignment1.launch`) that launches all the necessary nodes.

```xml
<?xml version="1.0"?>
<launch>
    <include file="$(find assignment_2_2024)/launch/sim_w1.launch" />
    <param name="des_pos_x" value="0.0" />
    <param name="des_pos_y" value="1.0" />
    <node pkg="assignment_2_2024" type="wall_follow_service.py" name="wall_follower" />
    <node pkg="assignment_2_2024" type="go_to_point_service.py" name="go_to_point" />
    <node pkg="assignment_2_2024" type="bug_as.py" name="bug_action_service" output="screen" />
    <node pkg="assignment_2_2024" type="user_input.py" name="user_input" output="screen" launch-prefix="lxterminal -e"/> 
    <node pkg="assignment_2_2024" type="get_last_target_service.py" name="get_last_target_service" />
```
By adding ' launch-prefix="lxterminal -e" ', the node set_target_client.py is launched in a different terminal to allow the user to easily set the target.

## `user_input.py` Node

The `user_input.py` node is a key component of the system, enabling user interaction for setting and canceling goals for the robot, while also publishing the robot's position and velocity. The node performs the following functions:

### 1. Action Client for Goal Setting and Cancellation
- The node interacts with an action server (`/reaching_goal`) to send goals to the robot.
- The user can input a target position (x, y) for the robot.
- The user has the option to cancel the current goal if it's active.
- The node uses the feedback/status of the action server to determine when the robot reaches the target.

### 2. Publishing Robot's Position and Velocity
- The node subscribes to the `/odom` topic, which provides the robot's current position and velocity.
- It then publishes a custom message (`PositionVelocity`) containing:
  - `x`, `y`: Robot's position.
  - `vel_x`, `vel_z`: Robot's linear and angular velocities.
- This information is published on the `/pos_vel` topic.

### 3. User Interaction
- The node prompts the user with two commands:
  - `y`: Set a new target goal (x, y).
  - `c`: Cancel the last goal.
- The user can enter valid numeric coordinates for the target position, which are then sent to the action server.

### 4. Node Execution
- The node continuously runs and listens for user inputs while also publishing position and velocity data.
- The user can interact with the robot by providing commands and monitoring the robot's progress toward the target goal.

## `get_last_target_service.py` Node

The `get_last_target_service.py` node provides a service to retrieve the last target position set by the user. It performs the following functions:

### 1. Retrieving the Last Target
- The node listens to the `/pos_vel` topic, which provides the robot's position and velocity.
- It extracts the last target position (x, y) from the ROS parameters `/des_pos_x` and `/des_pos_y`, which are updated when the user sets a new target.

### 2. Providing the Last Target via Service
- The node provides a service (`/get_last_target`) that returns the last target position (x, y) when called.
- The service uses the `GetLastTarget` service, created specifically for this purpose, which includes the fields `last_target_x` and `last_target_y`.

### 3. Node Execution
- The node continuously runs and listens for service calls.
- When a service call is made, it responds with the last target's position.




## Running the Package
To run the package and launch the simulation environment, follow these steps:

### 1) Build the package: 
If you haven't built the package yet, run the following command in your ROS workspace:
```bash
catkin_make
```

### 2) Launch the simulation:
```
roslaunch assignment_2_2024 assignment1.launch
```

### 3) Set a new target:
This node is run, by the launch file, in a separate terminal, allowing the user to directly access interface.
The user_input.py node will prompt you to enter a target position (x, y).
You can set a new target by entering the desired x and y coordinates.
The robot will attempt to reach the target, and the position and velocity will be published to the /pos_vel topic.

### 4) Cancel the target:

You can cancel the last goal by typing c in the terminal running the user_input.py node.

### 5) Get the last target coordinates:

To retrieve the last set target, run the following command in a new terminal:
```
rosservice call /get_last_target
```
