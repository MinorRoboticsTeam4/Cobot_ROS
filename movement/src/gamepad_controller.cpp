/*
 * gamepad_controller.cpp
 *
 *      Author: aclangerak
 */

//External
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>


//Internal
#include "movement/gamepad_controller.h"


/**
 * GamePadTeleop initializes the variables inside and
 * setups the NodeHandler for publishing and listening.
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
  nh.param("gamepad_controller/axis_linear", linear, linear);
  nh.param("gamepad_controller/axis_angular", angular, angular);
  nh.param("gamepad_controller/scale_linear", linear_scale, linear_scale);
  nh.param("gamepad_controller/scale_angular", angular_scale, angular_scale);

  //Advertise to the turtle's cmd_vel
  vel_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

  //Subscribe to the messages of the gamepad
  //and call joyCallback with incoming messages.
  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &GamePadTeleop::joyCallback, this);

  ROS_INFO("Starting gamepad with: linear %d, angular %d, "
		  "linear scale %f, angular scale %f",
		  linear,angular ,
		  linear_scale, angular_scale);
}

/**
 * Returns the set linear axis(from teleop.yaml)
 */
int GamePadTeleop::getLinearAxis()
{
	return linear;
}
/**
 * Returns the set angular axis(from teleop.yaml)
 */
int GamePadTeleop::getAngularAxis()
{
	return angular;
}

/**
 * Returns the set linear scale(from teleop.yaml)
 */
double GamePadTeleop::getLinearScale()
{
	return linear_scale;
}

/**
 * Returns the set angular scale(from teleop.yaml)
 */
double GamePadTeleop::getAngularScale()
{
	return angular_scale;
}


/**
 * Callback function that converts the joy message received from the
 * gamepad to a twist message.
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

