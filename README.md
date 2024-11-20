# HOMEWORK_2 PATELLARO EMMANUEL P38000239 #
# CONTROL A MANIPULATOR TO FOLLOW A TRAJECTORY #
This README file will show the instructions on how to build and run the Homework_2 Project 

## Features ##
- Trapezoidal Velocity Profile and Cubic Polynomial
- Linear and Circular Trajectories
- Position Controller
- Velocity Controller
- Effort Controller with Inverse Dynamics both in Joint and Operational Spaces
- Torque Plot

## Available Directory in this Repository ##
- kdl
- ros2_kdl_package
- ros2_iiwa
- bag

## Getting Started
1. Follow the guide to install ROS2 in Docker [here](https://github.com/RoboticsLab2024/ros2_docker_scripts.git)
2. Clone this repo in your `src` folder inside the `ros2_ws`
    ```shell
    cd src
    git clone https://github.com/EmmanuelPat6/Homework_1.git
    ```
3. Build the packages and make them visible to your workspace
    ```shell
    cd ..
    colcon build
    source install/setup.bash
    ```
**NOTE**: To make life easier, a world in Gazebo with *zero gravity* has been used in order to compensate the gravity itself. For this reason, in the controller it is no more necessary to explicitly have the *Gravity Compensation Term* of the *PD+ Controller*.
## Usage
The instructions to make this project work are straightforward and consist of only two steps:  

1. An instruction to spawn the robot in Gazebo and Rviz with the appropriate *Controller*.
    ```shell
    ros2 launch iiwa_bringup iiwa.launch.py command_interface:="position/velocity/effort" robot_controller:="position_controller/velocity_controller/effort_controller"
    ```
   by default, launching simply
    ```shell
    ros2 launch iiwa_bringup iiwa.launch.py
    ```
   the position interface and the *position_controller* are launched
:warning::warning::warning: It is *NECESSARY* to act very quickly by pressing the play button in the bottom left corner to ensure the controllers are activated. If this is not done, you will need to close Gazebo, reissue the same command, and repeat the steps. You can confirm that the controllers have been loaded correctly if, after the opening RViz2, the robot is spawned in the correct way without strange behavior.

2. An istruction to send *commands* to the robot, depending on the interface selected during the launch process.  
    ```shell
    ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=position/velocity/effort
    ```
   Also there, the default command send *position commands*. 
    ```shell
    ros2 run ros2_kdl_package ros2_kdl_node
    ```
   **It is necessary to give the correct commands depending on which controller was launched initially**. If this not happens, the robot, obviously, will not move.

### Implementation
1. Launch `arm_gazebo.launch.py`  in a sourced terminal run
    ```shell
    ros2 launch arm_gazebo arm_gazebo.launch.py
    ```
2. Run Publisher and Subscriber contained in `arm_controller_node.cpp` in another terminal
    ```shell
    ros2 run arm_control arm_controller
    ```
    After a few seconds the robot should move and in the terminal, the various joint variables value will be displayed.

3. To view the robot in RViz2, run in another terminal (only if `arm_gazebo.launch.py` is already launched)
    ```shell
    ros2 launch arm_description display.launch.py
    ```
    The image viewed by the camera will also be displayed automatically.

4. To show the image captured by the camera separately, it is possible to see it through `rqt_image_view`. Run
    ```shell
    rqt
    ```
    and go in `Plugins->Visualization->Image View` and select `/videocamera`.

    To appreciate the camera behavior, it is possible to add some objects in the `Gazebo Environment`.

5. To give other commands to the robot, it is necessary to stop the `arm_controller` node (in which there are the Publisher and the Subscriber). It is both possible to use this command
    ```shell
    ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0]" 
    ```
    with the desired joint positions, or run the New Publisher (not required by the project specifications) which allows to give these values directly from the terminal without the entire command but simply by entering the values separated by a space.
   ```shell
    ros2 run arm_control publisher_terminal
    ```
    In this case, to keep track of the joint position values, it is necessary to do
   ```shell
    ros2 topic echo /joint_states
    ```
