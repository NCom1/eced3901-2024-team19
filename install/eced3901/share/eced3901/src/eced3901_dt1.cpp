/*
Code for DT1
Based off V. Sieben
Written by Nicholas Comeau - Group 19 ECED3901
Version 1.0
Date: May 14, 2024
License: GNU GPLv3
*/

// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono>		// Timer functions
#include <functional>		// Arithmetic, comparisons, and logical operations
#include <memory>		// Dynamic memory management
#include <string>		// String functions
#include <cmath>		// Math Functions

// ROS Client Library for C++
#include "rclcpp/rclcpp.hpp"
 
// Message types
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;


// Create the node class named SquareRoutine
// It inherits rclcpp::Node class attributes and functions
class SquareRoutine : public rclcpp::Node
{
  public:
	// Constructor creates a node named Square_Routine. 
	SquareRoutine() : Node("Square_Routine")
	{
		// Create the subscription (subscribes to odom)
		// The callback function executes whenever data is published to the 'topic' topic.
		subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&SquareRoutine::topic_callback, this, _1));
          
		// Create the publisher (publishes to cmd_vel)
		// Publisher to a topic named "topic". The size of the queue is 10 messages.
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
      
	  	// Create the timer
	  	timer_ = this->create_wall_timer(100ms, std::bind(&SquareRoutine::timer_callback, this)); 	  
	}

  private:
	void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
	{
		x_now = msg->pose.pose.position.x;
		y_now = msg->pose.pose.position.y;

		//get the quaternion values for the orientation. wont need the x and y values but they are inlcuded so i understand things easier.
		qua_x = msg->pose.pose.orientation.x;
		qua_y = msg->pose.pose.orientation.y;
		qua_z = msg->pose.pose.orientation.z;
		qua_w = msg->pose.pose.orientation.w;
		
		//RCLCPP_INFO(this->get_logger(), "Odom Acquired."); //display the odometry values (i think)
	}
	
	void timer_callback()
	{
		geometry_msgs::msg::Twist msg;
        	
		// Calculate distance travelled from initial
		d_now =	pow( pow(x_now - x_init, 2) + pow(y_now - y_init, 2), 0.5 );

		//normalize the quaternion
		qua_norm = pow((pow(qua_x,2)+pow(qua_y,2)+pow(qua_z,2)+pow(qua_w,2)),0.5);

		qua_x = qua_x/qua_norm; 
		qua_y = qua_y/qua_norm; 
		qua_z = qua_z/qua_norm; 
		qua_w = qua_w/qua_norm; 

		//convert quaternion to euler/degrees Code taken from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

		double siny_cosp = 2 * (qua_w * qua_z + qua_x * qua_y);
    		double cosy_cosp = 1 - 2 * (qua_y * qua_y + qua_z * qua_z);
   		angle_now = ((180/M_PI) * std::atan2(siny_cosp, cosy_cosp));//added 180/pi to get into degrees
		
		// Keep moving if not reached last distance target
		if (d_now < d_aim)
		{
			msg.linear.x = x_vel; 
			msg.angular.z = 0;
			publisher_->publish(msg);		
		}
		//turn to the left when 1 meter has been acheived
		else if (abs(constrainAngle(angle_now-angle_init) < angle_aim)){

			msg.linear.x = 0; //stops the robot
			msg.angular.z = z_ang; //starts to turn on itself for 90degrees
			publisher_->publish(msg);

		}
		// If done step, stop
		else {
	
			msg.linear.x = 0;
			msg.angular.z = 0;
			publisher_->publish(msg);
			last_state_complete = 1;
		}


		sequence_statemachine();		
		
	}
		//RCLCPP_INFO(this->get_logger(), "Published cmd_vel.");
	
	void sequence_statemachine()
	{
		if (last_state_complete == 1)
		{
			switch(count_)
			{
				case 0:
					move_distance(0.87);
					RCLCPP_INFO(this->get_logger(), "case 0");//print current case for debugging
					//current_angle = angle_now;
					break;
				case 1:
					RCLCPP_INFO(this->get_logger(), "%f", angle_now);
					turn_angle(83);
					RCLCPP_INFO(this->get_logger(), "case 1");//print current case for debugging
					break;
				case 2:
					move_distance(0.87);
					//current_angle = angle_now;
					RCLCPP_INFO(this->get_logger(), "case 2");//print current case for debugging
					RCLCPP_INFO(this->get_logger(), "%f", angle_now);
					break;
				case 3:
					turn_angle(83);
					RCLCPP_INFO(this->get_logger(), "case 3");//print current case for debugging
					break;
				case 4:
					move_distance(0.87);
					RCLCPP_INFO(this->get_logger(), "%f", angle_now);
					//current_angle = angle_now;
					RCLCPP_INFO(this->get_logger(), "case 4");//print current case for debugging
					break;
				case 5:
					turn_angle(83);
					RCLCPP_INFO(this->get_logger(), "case 5");//print current case for debugging
					break;
				case 6:
					move_distance(0.87);
					RCLCPP_INFO(this->get_logger(), "%f", angle_now);
					//current_angle = angle_now;
					RCLCPP_INFO(this->get_logger(), "case 6");//print current case for debugging
					break;
				case 7:
					turn_angle(83);
					RCLCPP_INFO(this->get_logger(), "case 7");//print current case for debugging
					RCLCPP_INFO(this->get_logger(), "%f", angle_now);
					break;
				default:
					break;
			}	
		}
	}
	
	// Set the initial position as where robot is now and put new d_aim in place
	void move_distance(double distance)
	{
		d_aim = distance;
		x_init = x_now;		//sets x and y init to be able to calculate distance from setpoint
		y_init = y_now;		
		count_++;		// advance state counter
		last_state_complete = 0;	
	}
	void turn_angle(double angle)//sets the initial angle to where the robot is point to now and adds a new angle_aim in place
	{
		angle_aim = angle; //sets the desired angle we want to turn the robot by
		angle_init = angle_now; //find it current angle from the quaternion values
		count_++; //advance the state count to move to next case
		last_state_complete = 0;
	}
	double constrainAngle(double angle) //contrains angle if it shows up as negative (Angle Wrap)
	{
  		angle = fmod(angle+180,360);
    			if (angle < 0){
        		angle += 360;
			}
  		return angle-180;
	}	
	// Declaration of subscription_ attribute
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
         
	// Declaration of publisher_ attribute      
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	
	// Declaration of the timer_ attribute
	rclcpp::TimerBase::SharedPtr timer_;
	
	// Declaration of Class Variables
	double x_vel = 0.1; //the velocity of the robot experiences when it moves in the x direction
	double z_ang = 0.2; //the angular velocity of the robot when it rotates in the z direction
	double x_now = 0, y_now = 0; //Current x and y position the robot is currently at
	double x_init = 0, y_init = 0; //Initial x and y position the robot at the start of the move distance command
	double d_now = 0; //current distance from its staring point at the start of the move distance command 
	double d_aim = 0; //how far we want the robot to travel
	double distance = 0;
	double angle = 0;
	double angle_aim = 0; //desired angle to turn the robot
	double angle_init = 0; //initial angle seen by the robot at the start of its turn
	double angle_now = 0; //current angle calculated from quaternion0
	double qua_x = 0, qua_y = 0, qua_z = 0, qua_w = 0; //innitialize quaternion values
	double qua_norm = 0;
	double siny_cosp = 0, sinp_cosp = 0; //initialize the quaternion to euler equations
	size_t count_ = 0; //state counter (goes through switch case)
	int last_state_complete = 1;
};
    	


//------------------------------------------------------------------------------------
// Main code execution
int main(int argc, char * argv[])
{
	// Initialize ROS2
	rclcpp::init(argc, argv);
  
	// Start node and callbacks
	rclcpp::spin(std::make_shared<SquareRoutine>());
 
	// Stop node 
	rclcpp::shutdown();
	return 0;
}
