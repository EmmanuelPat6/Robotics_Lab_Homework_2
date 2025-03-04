// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;
//int TrajectorySelection(double _accDuration, double _trajRadius);
class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // declare cmd_interface parameter (position, velocity)
            declare_parameter("cmd_interface", "position"); // defaults to "position"
            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort"))
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }

            iteration_ = 0;
            t_ = 0;
            joint_state_available_ = false; 

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj);
            joint_efforts_.resize(nj);

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();
            // std::cout << "The initial EE pose is: " << std::endl;  
            // std::cout << init_cart_pose_ <<std::endl;

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);
            // std::cout << "The inverse kinematics returned: " <<std::endl; 
            // std::cout << q.data <<std::endl;

            // Initialize controller
            KDLController controller_(*robot_);

            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1)); //For Linear Trajectory with Cubic Polynomial
                                                                                                               //it is advisable to set the offset to 0.15 instead of 0.10

            // EE's trajectory end position (just opposite y)
            Eigen::Vector3d end_position; end_position << init_position[0], -init_position[1], init_position[2];


            //You can implement 4 types of trajectories
            // 1. Linear Trajectory with Trapezoidal Velocity Profile (you have to define traj_duration, acc_duration, init_position, end_position)
            // 2. Linear Trajectory with Cubic Polynomial (you have to define traj_duration, init_position, end_position)
            // 3. Circular Trajectory with Trapezoidal Velocity Profile (you have to define traj_duration, acc_duration, init_position, radius)
            // 4. Circular Trajectory with Cubic Polynomial (you have to define traj_duration, init_position, radius)
            //In particular, if you want a Linear Trajectory it is necessary to impose radius = 0 and !=0 otherwise
            //Instead, if you want a Trapezoidal Velocity Profile, it is necessary to have the parameter acc_duration != 0
            //If you want a Cubic Polynomial acc_duration=0 is needed
            //So, to decide what type of trajectory the robot should do, the values of the parameters radius and acc_duration is foundamental
            //index = TrajectorySelection(acc_duration, traj_duration);


            // Plan trajectory
            double acc_duration;
            double t = 0.0;
            double radius = 0.1;
            double traj_duration;

            if(cmd_interface_ == "position" || cmd_interface_ == "velocity"){
                traj_duration = 1.5;
                acc_duration = 0.7; //If you want a Trapezoidal Velocity Profile, please se this quantity to 0.7
            }
            else if(cmd_interface_ == "effort"){
                traj_duration = 6.0; //Longer because of faster implementation. For all the trajectories set this to 6. Only for the Linear Trajectory with Cubic Polynomial in the case of the
                                      //Inverse Dynamics Control in the Operational Space set this to 10
                acc_duration = 3.5;  //If you want a Trapezoidal Velocity Profile with an Inverse Dynamics Control in the JOINT SPACE, please set this quantity to 3.5
                                     //Instead if you it with an Inverse Dynamics Control in the OPERATIONAL SPACE, please set this quantity to 1.0 for Linear Trajectory and 2.5 for Circular Trajectory
            }


            ///////////////////////////////////////////////////// TESTING TRAJECTORIES ////////////////////////////////////////////////////////    

            //Test Circular Trajectory with Cubic Polynomial
            //planner_ = KDLPlanner(traj_duration, init_position, radius);
            
            //Test Linear Trajectory with Cubic Polynomial
            //planner_ = KDLPlanner(traj_duration, init_position, end_position);

            //Test Linear Trajectory with Trapezoidal Velocity Profile
            //planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position);

            //Test Circular Trajectory with Trapezoidal Velocity Profile
            //planner_ = KDLPlanner(traj_duration, acc_duration, init_position, radius);

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




            ///////////////////////////////////////// TRAJECTORY SELECTION ////////////////////////////////////////////    
            if(radius == 0 && acc_duration != 0)
            {
                planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position);
                std::cout<< "The trajectory chosen is: Linear Trajectory with Trapezoidal Velocity Profile \n";
            }
            else if(radius == 0 && acc_duration == 0)
            {
                planner_ = KDLPlanner(traj_duration, init_position, end_position);
                std::cout<< "The trajectory chosen is: Linear Trajectory with Cubic Polynomial \n";
            }
            else if(radius != 0 && acc_duration != 0)
            {
                planner_ = KDLPlanner(traj_duration, acc_duration, init_position, radius);
                std::cout<< "The trajectory chosen is: Circular Trajectory with Trapezoidal Velocity Profile \n";
            }
            else if(radius != 0 && acc_duration == 0)
            {
                planner_ = KDLPlanner(traj_duration, init_position, radius);
                std::cout<< "The trajectory chosen is: Circular Trajectory with Cubic Polynomial \n";
            }
            else
            {
                std::cout<<"Not a valid trajectory chosen for you robot \n";
            }


            // Retrieve the first trajectory point
            trajectory_point p = planner_.compute_trajectory(t);

            // compute errors
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(init_cart_pose_.p.data));
            //std::cout << "The initial error is : " << error << std::endl;
            
            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }
            else if(cmd_interface_ == "effort"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(20), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint effort commands
                for (long int i = 0; i < joint_efforts_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_(i);
                }


            }

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

    private:

        void cmd_publisher(){

            iteration_ = iteration_ + 1;

            double total_time;
            int trajectory_len;
            //Define Trajectory
            if(cmd_interface_ == "position" || cmd_interface_ == "velocity"){
                total_time = 1.5;
                trajectory_len = 150;
            }
            else if(cmd_interface_ == "effort"){
                total_time = 6.0;    //Longer than the previous ones because of faster implementation
                trajectory_len = 600;
            }

            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            t_+=dt;
            KDLController controller_(*robot_);


            if (t_ < total_time){

                // Set endpoint twist
                // double t = iteration_;
                // joint_velocities_.data[2] = 2 * 0.3 * cos(2 * M_PI * t / trajectory_len);
                // joint_velocities_.data[3] = -0.3 * sin(2 * M_PI * t / trajectory_len);

                // Integrate joint velocities
                // joint_positions_.data += joint_velocities_.data * dt;

                // Retrieve the trajectory point
                trajectory_point p = planner_.compute_trajectory(t_);

                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();

                KDL::Twist cartv = robot_->getEEVelocity();

                // Compute desired Frame
                KDL::Frame desFrame;
                desFrame.M = cartpos.M;
                desFrame.p = toKDL(p.pos);
                

                //Desired Velocity
                KDL::Twist des_vel = KDL::Twist::Zero();
                des_vel.rot = cartv.rot;
                des_vel.vel = toKDL(p.vel);            

                //Desired Acceleration
                KDL::Twist des_acc = KDL::Twist::Zero();
                des_acc = KDL::Twist(KDL::Vector(p.acc[0], p.acc[1], p.acc[2]), KDL::Vector::Zero());    


                Eigen::Vector3d omega_d(des_vel.rot.data);
                Eigen::Vector3d omega_e(robot_->getEEVelocity().rot.data);

                // Compute Errors
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d dot_error = computeLinearError(p.vel, Eigen::Vector3d(cartv.vel.data));

                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));  //Orientamento desiderato = quello iniziale, Orientamento effettivo, quello dell'end effector
                Eigen::Vector3d o_dot_error = computeOrientationVelocityError(omega_d, omega_e,toEigen(init_cart_pose_.M), toEigen(cartpos.M));


                std::cout << "The error norm is : " << error.norm() << std::endl;

                if(cmd_interface_ == "position"){
                    // Next Frame
                    KDL::Frame nextFrame;
                    nextFrame.M = cartpos.M;
                    nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1*error))*dt; 

                    // Compute IK
                    robot_->getInverseKinematics(nextFrame, joint_positions_);
                }
                else if(cmd_interface_ == "velocity"){

                    // Compute differential IK
                    Vector6d cartvel; cartvel << p.vel + 5*error, o_error;
                    joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                    joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;
                }
                else if(cmd_interface_ == "effort"){
                    
                    Vector6d cartvel; cartvel << p.vel + 5*error, o_error;
                    //Vector6d cartacc; cartacc << p.acc + 5*error+ 2*dot_error, o_dot_error;
                    KDL::JntArray velocity_temp;

                    velocity_temp = joint_velocities_;
                    
                    //joint_accelerations_.data = pseudoinverse(robot_->getEEJacobian().data)*(cartacc - robot_->getEEJacDotqDot());

                    joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*(cartvel);

                    joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;

                    joint_accelerations_.data = (joint_velocities_.data - velocity_temp.data)/dt;

                    joint_efforts_.data = controller_.idCntr(joint_positions_, joint_velocities_, joint_accelerations_, 40, 20);
                    
                    //joint_efforts_.data = controller_.idCntr(cartpos, des_vel, des_acc, 150, 70, 50, 40);


                }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "position"){
                    // Send joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_(i);
                    }
                }
                else if(cmd_interface_ == "effort"){
                    // Send joint effort commands
                    for (long int i = 0; i < joint_efforts_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_(i);
                    }

                }

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
                // std::cout << "EE pose is: " << robot_->getEEFrame() <<std::endl;  
                // std::cout << "Jacobian: " << robot_->getEEJacobian().data <<std::endl;
                // std::cout << "joint_positions_: " << joint_positions_.data <<std::endl;
                // std::cout << "joint_velocities_: " << joint_velocities_.data <<std::endl;
                // std::cout << "iteration_: " << iteration_ <<std::endl <<std::endl;
                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
            }
            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                if(cmd_interface_ == "velocity"){
                    //Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;
                    }
                }
                if(cmd_interface_ == "effort"){
                    //Send joint efforts commands
                    for (long int i = 0; i < joint_efforts_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;
                    }
                }

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Positions %zu: %f", i, sensor_msg.position[i]);                
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Velocities %zu: %f", i, sensor_msg.velocity[i]);
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Efforts %zu: %f", i, sensor_msg.effort[i]);
            // }

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
                joint_efforts_.data[i] = sensor_msg.effort[i];
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_efforts_;

        KDL::JntArray joint_accelerations_;
        KDL::JntArray zero_array_;

        double Kp, Kd;
        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;
        int iteration_;
        bool joint_state_available_;
        double t_;
        std::string cmd_interface_;
        KDL::Frame init_cart_pose_;
};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}