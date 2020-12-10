# Behavioral Architecture with Real Simualtion
ROS architecture for a robot moving into a simulated arena. The robot has 3 behaviors:
- Play
- Sleep
- Normal

The user can interact with the robot by moving the ball
# Software architecture and states diagrams
## Software architecture 
The architercture is composed by 4 components: 

- Command: simulate a user that move the ball into the arena

- Ball detection: detect the green ball

- Robot controller: control the robot toward a target position

- Command manager: implement robot behaviors through a FSM

<p align="center">
  <img src="./images/Behavioral_Architecture.jpg">
</p>

## State diagram
The finite state machine is composed by 3 state (robot behaviors):

- PLAY: the robot follows the ball and when the ball is stationary, it rotates its head 45 degrees to the left and 45 degrees to the right. Back in position, the robot follows the ball

- NORMAL (initial state): robot moves in a random ways

- SLEEP: robot goes to home position, it stays there for a certain time and then goes to NORMAL state

<p align="center">
  <img src="./images/Behavioral_Architecture_FSM.jpg">
</p>

## ROS messages and parameters
The messages are:

- `PoseStamped`: robot target position
- `Pose`: position reached by the robot
- `String`: radius and center of the ball
- `Int`: radius and center of the ball
- `CompressedImage`: images received from the camera

The parameters are:

- `home_pos_x,home_pos_x`: define the home position in the map (double)
- `map_x,map_y`: define the dimensions of the map (integer)
- `min_delay_command,max_delay_command`: define the min and max delay for sending the random position of the ball (double)
- `min_transition_play_normal,max_transition_play_normal`: define the min and max delay to trasit between PLAY and NORMAL (integer)
- `min_transition_normal_sleep,max_transition_normal_sleep`: define the min and max delay to trasit between NORMAL and SLEEP (integer)
- `min_sleep_delay,max_sleep_delay`: define the min and max delay for the SLEEP state (double)
- `min_dis_ball_delay,max_dis_ball_delay`: define the min and max delay for disappearing the ball (int)


# Packages and files
There are 3 packages:

- `Sensoring`: contains the [command.py](sensoring/src/command.py) and [gesture.py](sensoring/src/gesture.py) files used to simulate the user command and pointed gestures
- `Robot control`: contains the [motion.py](robot_control/src/motion.py) file used to simulate robot motion
- `Command manager`: contains the [command_manager.py](manager/src/command_manager) file that implements the FSM of robot behaviors.

# Installation and running
In order to run this software, the following prerequisities are needed:
- [ROS Noetic](http://wiki.ros.org/noetic)
- [smach](http://wiki.ros.org/smach)

Before running the software, you must have all files as executable otherwise you can make them executable with the following command
```
cd <your_workspace>/src/Behavioral-Architecture
chmod +x sensoring/src/*
chmod +x robot_control/src/*
chmod +x manager/src/*
```
To run the software
```
cd <your_workspace>
catkin_make
source devel/setup.bash
cd src/Behavioral-Architecture
roslaunch launch_file.launch
```

# Working hypothesis and environment
The robot interact with a human via a command and pointed gestures. It moves inside a 2D discrete enviroment. Both robot targets position and pointed gestures belongs to the map. The robot has 3 behaviors: Play,Normal,Sleep. The robot can receive any command while executing PLAY state and it can receive any command or pointed gesture while executing SLEEP state but all of them are ignored while executing one of the two states. The initial state is NORMAL. The only command is "play". There two predifined positions inside the map ("Person" and "Home" position) that cannot be changed during the execution. When the robot moves, it cannot respond to other stimulus. The robot can go into Person or "Home" position during NORMAL behavior and it can go into Home position during PLAY state.

# System's features
- Specify different dimensions of the map
- It is possibile to define different delays for the simulation
- Define different position of the person and the "home" inside the map before start the simulation
- It is possible to visualize the states transition in the shell
- The robot will notify if it will reach the target position and it is possibile to visualize it in the shell (the position that the robot has reached)
- It is not possibile to generate a pointed gesture equal to the person position

# System's limitations
- There is no graphical interface to view the map and the movement of the robot within it
- Commmand, pointed gesture and robot motion are simulated
- There isn't check if the user puts a position for the person or the "home" outside the map
- Since it is used ROS Noetic, the state transition visualization is limited to the shell (it is necessary to fix some files in order to use smach viewer)

# Possible technical improvements
- Add other behaviors to the robot
- Use a graphical interface for viewing the simulation
- Add error handling in order to prevent some user position outside the map
- Implemente a real robot motion control
- Implements a user interface for the command and pointed gestures
- Add more commands
- Prevent the robot from going into "unauthorized" positions during some behaviors

# Author and contact
[Simone Voto](https://github.com/Cavalletta98) - simone.voto98@gmail.com