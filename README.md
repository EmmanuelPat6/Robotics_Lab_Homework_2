# HOMEWORK_2 PATELLARO EMMANUEL P38000239 #
# 🤖 CONTROL A MANIPULATOR TO FOLLOW A TRAJECTORY 🤖 #
This README file will show the instructions on how to build and run the Homework_2 Project 

## Features 🪐 ##
- Trapezoidal Velocity Profile and Cubic Polynomial 📈🚀
- Linear and Circular Trajectories ↔️🔵
- Position Controller 📍📏
- Velocity Controller 🏎️💨
- Effort Controller with Inverse Dynamics both in Joint and Operational Spaces 🦾⚙️
- Torques Plot 🔧🔨

## Available Directory in this Repository 📂 ##
- kdl
- ros2_kdl_package
- ros2_iiwa
- bag

## Getting Started ⏯️
1. Follow the guide to install ROS2 in Docker [here](https://github.com/RoboticsLab2024/ros2_docker_scripts.git)
2. Clone this repo in your `src` folder inside the `ros2_ws`
    ```shell
    cd src
    git clone https://github.com/EmmanuelPat6/Homework_2.git
    ```
3. Build the packages and make them visible to your workspace
    ```shell
    cd ..
    colcon build
    source install/setup.bash
    ```
**NOTE**: To make life easier, a world in Gazebo with *zero gravity* has been used in order to compensate the gravity itself. For this reason, in the controller it is no more necessary to explicitly have the *Gravity Compensation Term* of the *PD+ Controller*.
## Usage 📖
The instructions to make this project work are straightforward and consist of only two steps:  

1. 🤖🤖 An instruction to spawn the robot in Gazebo and Rviz with the appropriate **Controller**.
    ```shell
    ros2 launch iiwa_bringup iiwa.launch.py command_interface:="position/velocity/effort" robot_controller:="position_controller/velocity_controller/effort_controller"
    ```
   by default, launching simply
    ```shell
    ros2 launch iiwa_bringup iiwa.launch.py
    ```
   the position interface and the **position_controller** are launched.

⚠️⚠️⚠️ It is **NECESSARY** to act very quickly by pressing the play button in the bottom left corner to ensure the controllers are activated. If this is not done, you will need to close Gazebo, reissue the same command, and repeat the steps. You can confirm that the controllers have been loaded correctly if, after the opening RViz2, the robot is spawned in the correct way without strange behavior ⚠️⚠️⚠️

2. 🕹️🔧 An istruction to send *commands* to the robot, depending on the interface selected during the launch process.  
    ```shell
    ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=position/velocity/effort
    ```
   Also there, the default command send *position commands*. 
    ```shell
    ros2 run ros2_kdl_package ros2_kdl_node
    ```
**Be careful to give the correct commands depending on which controller was launched initially**. If this not happens, the robot, obviously, will not move.

## Trajectory Selection 🎯
✈️ To change trajectory it is sufficient to change some variables in the file `ros_kdl_package/src/ros2_kdl_node.cpp` in `line 128`. If you want a Linear Trajectory it is necessary to impose `radius = 0` and `!=0` otherwise (possibly a low value like `0.1`). Instead, if you want a Trapezoidal Velocity Profile, it is necessary to have the parameter `acc_duration != 0`. If you want a Cubic Polynomial `acc_duration=0` is needed. So, to decide what type of trajectory the robot should do, the values of the parameters **radius** and **acc_duration** is fundamental. When you change the parameters in the file, you must rebuild all with `colcon build`. To avoid this issue, a file could be implemented to read the necessary parameters for the trajectory (this was not requested and has not been implemented).
 
**Note**: in the file, by default, it has been imposed a simple offset of 0.1 along z-axis. If you want to implement a Linear Trajectory with Cubic Polynomial, it is advisable to change this value to 0.15 because, sometimes, there might be issues, and it may be necessary to give the instruction more than once (only for this type of trajectory; for the others, a value of 0.1 is sufficient).

## Implementation 💻
Let's see all the possible solutions:

### Position Controller 📍📏
1. Launch Gazebo with a position controller
    ```shell
    ros2 launch iiwa_bringup iiwa.launch.py
    ```
2. Send position commands to the robot
    ```shell
    ros2 run ros2_kdl_package ros2_kdl_node
    ```
    After a few seconds the robot should move according to the given trajectory


### Velocity Controller 🏎️💨
1. Launch Gazebo with a velocity controller
    ```shell
    ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
    ```
2. Send velocity commands to the robot
    ```shell
    ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity
    ```
    After a few seconds the robot should move according to the given trajectory


### Effort Controller 🦾⚙️
1. Launch Gazebo with an effort controller :
    ```shell
    ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller"
    ```
2. Send effort commands to the robot
    ```shell
    ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=effort
    ```
    After a few seconds the robot should move according to the given trajectory
3. To view torques sent to the robot run
    ```shell
    rqt
    ```
    and go in `Plugins->Visualization->Plot` and add `/effort_controller/commands/data[0]`, then `/effort_controller/commands/data[1]` up to `/effort_controller/commands/data[6]`
## Inverse Dynamics Control in the Operational Space 🔬
By default, the Inverse Dynamics Controller implemented is the Joint Space one. Instead, to implement an Inverse Dynamics Controller in the Operational Space it is sufficient to comment, in the file `ros_kdl_package/src/ros2_kdl_node.cpp`, from `line 332` to `line 346` (it is important to comment all these line and not only the one of the idCntr in the joint space) and uncomment `line 348`: `joint_efforts_.data = controller_.idCntr(cartpos, des_vel, des_acc, 40, 30, 20, 15);`. It is sufficient to run the same commands as in the previous case.
 
⚠️ To achieve a satisfactory result, take into account the comments provided in the parameter selection section. This final control, in fact, requires lower acceleration (where it is present) then the other one. Instead, ONLY for the Linear Trajectory with Cubic Polynomial, for the Inverse Dynamics Operational Space Control, it is necessary to increase a little bit more the trajectory duration (from 6 to 10)⚠️

## Recap Parameters 🧠
Since the choice of parameters (`lines 129-134`) is crucial for the selection and proper functioning of the system, here is a brief recap of the parameters to be used for each specific trajectory:
### Position and Velocity Control
- Linear Trajectory with Trapezoidal Velocity Profile: `acc_duration=0.7` `radius=0.0` `traj_duration=1.5`
- Linear Trajectory with Cubic Polynomial: `acc_duration=0.0` `radius=0.0` `traj_duration=1.5`
- Circular Trajectory with Trapezoidal Velocity Profile: `acc_duration=0.7` `radius=0.1` `traj_duration=1.5`
- Circular Trajectory with Cubic Polynomial: `acc_duration=0.0` `radius=0.1` `traj_duration=1.5`

### Effort Control
**Inverse Dynamics Joint Space Control**
- Linear Trajectory with Trapezoidal Velocity Profile: `acc_duration=3.5` `radius=0.0` `traj_duration=6.0`
- Linear Trajectory with Cubic Polynomial: `acc_duration=0.0` `radius=0.0` `traj_duration=6.0`
- Circular Trajectory with Trapezoidal Velocity Profile: `acc_duration=3.5` `radius=0.1` `traj_duration=6.0`
- Circular Trajectory with Cubic Polynomial: `acc_duration=0.0` `radius=0.1` `traj_duration=6.0`

**Inverse Dynamics Operational Space Control**
- Linear Trajectory with Trapezoidal Velocity Profile: `acc_duration=1.0` `radius=0.0` `traj_duration=6.0`
- Linear Trajectory with Cubic Polynomial: `acc_duration=0.0` `radius=0.0` `traj_duration=10.0`
- Circular Trajectory with Trapezoidal Velocity Profile: `acc_duration=2.5` `radius=0.1` `traj_duration=6.0`
- Circular Trajectory with Cubic Polynomial: `acc_duration=0.0` `radius=0.1` `traj_duration=6.0`

For further considerations regarding the choice of parameters, refer to the comments within the code and the report.

⛔ **No functions have been implemented to input parameters from the command line in such a way to provide greater control and to ensure safer parameter selection.**
