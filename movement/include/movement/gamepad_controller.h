/*
 * gamepad_controller.h
 *
 *  Created on: Nov 10, 2015
 *      Author: aclangerak
 */

#ifndef MOVEMENT_INCLUDE_MOVEMENT_GAMEPAD_CONTROLLER_H_
#define MOVEMENT_INCLUDE_MOVEMENT_GAMEPAD_CONTROLLER_H_

#include <sensor_msgs/Joy.h>
#include <ros/ros.h>


/**
 * This class contains the needed Nodehandler and related code.
 * Also it contains a callback function to convert joy -> twist messages.
 * Lastly, it holds the linear, angular and the scaling factors of
 * the gamepad.
 */
class GamePadTeleop
{
public:
	/**
	 * Default Consctructor
	 */
	GamePadTeleop();

	/**
	 * Returns the set linear axis(from teleop.yaml)
	 */
	int getLinearAxis();

	/**
	 * Returns the set angular axis(from teleop.yaml)
	 */
	int getAngularAxis();

	/**
	 * Returns the set linear scale(from teleop.yaml)
	 */
	double getLinearScale();

	/**
	 * Returns the set angular scale(from teleop.yaml)
	 */
	double getAngularScale();

private:
	/**
	 * Callback function to convert joy -> twist messages.
	 */
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  /**
   * NodeHandler for ROS usage
   */
  ros::NodeHandle nh;

  /**
   * holds linear and angular values
   */
  int linear, angular;

  /**
   * scaling factors
   */
  double linear_scale, angular_scale;

  /**
   * Publisher published messages.
   */
  ros::Publisher vel_pub;

  /**
   * Subscriber listens to messages and calls a callback messages.
   */
  ros::Subscriber joy_sub;

};





#endif /* MOVEMENT_INCLUDE_MOVEMENT_GAMEPAD_CONTROLLER_H_ */
