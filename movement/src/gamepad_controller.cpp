/*
 * gamepad_controller.cpp
 *
 *      Author: aclangerak
 */

//=======================================================
// predefined headers
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>

//=======================================================
// new defined headers
#include "movement/gamepad_controller.h"


namespace movement
{

	/**
	 * GamePadTeleop initializes the variables inside and
	 * setups the NodeHandler for publishing and listening.
	 *
	 * The default publish topic is "cmd_vel".
	 * This can be changed by adding the parameter "pub_topic" with
	 * the desired topic name.
	 *
	 */
	GamePadTeleop::GamePadTeleop():
		linear(0),
		angular(1),
		linear_scale(1.0),
		angular_scale(1.0)
	{
	  //Read in the params from teleop.yaml
	  //cannot get private namespaces to work,
	  //so using this method instead
	  nh.param<int>("gamepad_controller/axis_linear", linear, linear);
	  nh.param<int>("gamepad_controller/axis_angular", angular, angular);
	  nh.param<double>("gamepad_controller/scale_linear", linear_scale, linear_scale);
	  nh.param<double>("gamepad_controller/scale_angular", angular_scale, angular_scale);

	  //If a specific publish topic is defined, use that one
	  //Otherwise use the default name
	  std_msgs::String topic_name;
	  nh.param<std::string>("gamepad_controller/pub_topic",topic_name.data,"cmd_vel");
	  vel_pub = nh.advertise<geometry_msgs::Twist>(topic_name.data, 1);

	  ROS_INFO("Publishing on topic: %s",topic_name.data.c_str());

	  //Subscribe to the messages of the gamepad
	  //and call joyCallback with incoming messages.
	  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &GamePadTeleop::joyCallback, this);

	  ROS_INFO("Starting gamepad with: linear %d, angular %d, "
			  "linear scale %f, angular scale %f",
			  linear,angular ,
			  linear_scale, angular_scale);
	}

	/**
	 * Get the value of the linear axis.
	 * This value is either 0(default) or from
	 * config file(teleop.yaml)
	 * @return linear axis value
	 */
	const int GamePadTeleop::getLinearAxis()
	{
		return linear;
	}
	/**
	 * Get the value of the angular axis.
	 * This value is either 1(default) or from
	 * config file(teleop.yaml)
	 * @return angular axis value
	 */
	const int GamePadTeleop::getAngularAxis()
	{
		return angular;
	}

	/**
	 * Get the value of the linear scale.
	 * This value is either 1.0(default) or from
	 * config file(teleop.yaml)
	 * @return linear scale value
	 */
	const double GamePadTeleop::getLinearScale()
	{
		return linear_scale;
	}

	/**
	 * Get the value of the angular scale.
	 * This value is either 1.0(default) or from
	 * config file(teleop.yaml)
	 * @return angular scale value
	 */
	const double GamePadTeleop::getAngularScale()
	{
		return angular_scale;
	}


	/**
	 * (Callback) Function that converts the Joy message received from the
	 * gamepad to a Twist message.
	 *
	 * The Twist message that is only generated for differential drive, so only
	 * the following messages are defined:
	 * <li> linear.x  = linear speed
	 * <li> angular.z = angle speed
	 *
	 * @param joy   incoming joy message
	 */
	void GamePadTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
	{

	  ROS_DEBUG("Read gamepad command: linear %f angular %f " ,
					joy->axes[this->getLinearAxis()],
					joy->axes[this->getAngularAxis()]
				);

	  //New twist message
	  geometry_msgs::Twist twist;

	  twist.linear.x = linear_scale*joy->axes[this->getLinearAxis()];
	  twist.angular.z = angular_scale*joy->axes[this->getAngularAxis()];

	  //Publish the message
	  vel_pub.publish(twist);

	  ROS_DEBUG("Publish twist message: linear %f, angular %f",
			  twist.linear.x,
			  twist.angular.z
				);
	}

}
/** @brief Main function
 *
 * @param argc  An integer argument count of the command line arguments
 * @param argv  An argument vector of the command line arguments
 * @return  EXIT_SUCCESS if exit is success
 */
int main(int argc, char** argv)
{
  //Initialize ros node
  ros::init(argc, argv, "gamepad_teleop");

  //Initialize gamepad_teleop class
  movement::GamePadTeleop gamepad_teleop;

  ROS_INFO("Started gamepad_controller Node");

  //Run ros node
  while(ros::ok()){
	  ros::spin();
  }

  return EXIT_SUCCESS;
}

