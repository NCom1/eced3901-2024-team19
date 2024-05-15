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
#include <cmath>

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
		
		RCLCPP_INFO(this->get_logger(), "Odom Acquired."); //display the odometry values (i think)
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
		else if (){

			msg.linear.x = 0; //stops the robot
			msg.angular.z = 0.1; //starts to turn on itself for 90degrees
			publisher_-publish(msg);

		}
		// If done step, stop
		else
	
			msg.linear.x = 0;
			msg.angular.z = 0;
			publisher_->publish(msg);
			last_state_complete = 1;
		}


		sequence_statemachine();		
		

		//RCLCPP_INFO(this->get_logger(), "Published cmd_vel.");
	}
	
	void sequence_statemachine()
	{
		if (last_state_complete == 1)
		{
			switch(count_) 
			{
			  case 0:
			    move_distance(1.0);
			    break;
			  case 1:
			    move_distance(1.0);
			    break;
			  case 2:
			    move_distance(1.0);
			    break;
			  case 3:
			    move_distance(1.0);
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
		x_init = x_now;
		y_init = y_now;		
		count_++;		// advance state counter
		last_state_complete = 0;	
	}

	// Declaration of subscription_ attribute
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
         
	// Declaration of publisher_ attribute      
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	
	// Declaration of the timer_ attribute
	rclcpp::TimerBase::SharedPtr timer_;
	
	// Declaration of Class Variables
	double x_vel = 0.2;
	double x_now = 0, x_init = 0, y_now = 0, y_init = 0;
	double d_now = 0, d_aim = 0;
	size_t count_ = 0;
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



