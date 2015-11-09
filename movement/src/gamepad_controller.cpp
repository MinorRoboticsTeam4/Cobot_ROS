/*
 * gamepad_controller.cpp
 *
 *      Author: aclangerak
 */

//External
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

//Internal
#include "movement/gamepad_controller.h"


/**
 * GamePadTeleop initializes the variables inside and
 * setups the NodeHandler for publishing and listening.
 *
 */
GamePadTeleop::GamePadTeleop():
  linear(1),
  angular(2)
{

  //Read in the params from teleop.yaml
  nh.param("axis_linear", linear, linear);
  nh.param("axis_angular", angular, angular);
  nh.param("scale_angular", angular_scale, angular_scale);
  nh.param("scale_linear", linear_scale, linear_scale);

  //Advertise to the turtle's cmd_vel
  vel_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

  //Subscribe to the messages of the gamepad
  //and call joyCallback with incoming messages.
  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &GamePadTeleop::joyCallback, this);

  ROS_INFO("Starting gamepad with: linear %d, angular %d", linear, angular);
}

/**
 * Callback function that converts the joy message received from the
 * gamepad to a twist message.
 */
void GamePadTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

  ROS_DEBUG("Read gamepad command: linear %f, angular %f",
		    	joy->axes[linear],
				joy->axes[angular]
		    );


  //New twist message
  geometry_msgs::Twist twist;
  twist.linear.x = linear_scale*joy->axes[linear];
  twist.angular.z = angular_scale*joy->axes[angular];

  //Publish the message
  vel_pub.publish(twist);
}

int main(int argc, char** argv)
{
  //Initialize ros node
  ros::init(argc, argv, "gamepad_teleop");

  //Initialize gamepad_teleop class
  GamePadTeleop gamepad_teleop;

  ROS_INFO("Started gamepad_controller Node");

  //Run ros node continuous
  ros::spin();
}

