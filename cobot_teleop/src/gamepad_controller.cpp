/*
 * gamepad_controller.cpp
 *
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
#include "cobot_teleop/gamepad_controller.h"

namespace movement
{

/**
 * GamePadTeleop initializes the variables inside and
 * setups the NodeHandler for publishing and listening.
 *
 * The default publish topic is "gamepad_controller/cmd_vel".
 * This can be remapped to the desired topic.
 *
 */
GamePad_controller::GamePad_controller() :
    linearAxis(0), angularAxis(1), stopButton(0), linear_scale(1.0), angular_scale(1.0), nh(
        "~"), twist(), inverse_linAxis(0), inverse_angAxis(0)
{
  //Read in the params from teleop.yaml
  nh.param<int>("linear_axis", linearAxis, linearAxis);
  nh.param<int>("inverse_linear_axis", inverse_linAxis, inverse_linAxis);
  nh.param<int>("angular_axis", angularAxis, angularAxis);
  nh.param<int>("inverse_angular_axis", inverse_angAxis, inverse_angAxis);
  nh.param<int>("stop_button", stopButton, stopButton);
  nh.param<double>("linear_scale", linear_scale, linear_scale);
  nh.param<double>("angular_scale", angular_scale, angular_scale);

  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ROS_INFO("[Gamepad] Publishing on topic: %s",  vel_pub.getTopic().c_str());

  //Subscribe to the messages of the gamepad
  //and call joyCallback with incoming messages.
  //Listen to global topic "joy"
  joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &GamePad_controller::joyCallback, this);

  ROS_INFO("[Gamepad] start with : linear %d, angular %d, "
           "linear scale %f, angular scale %f",
           linearAxis, angularAxis, linear_scale, angular_scale);
}

/**
 * Shutdowns internal NodeHandler
 */
GamePad_controller::~GamePad_controller()
{
  nh.shutdown();
}

/**
 * Get the value of the linear axis.
 * This value is either 0(default) or from
 * config file(teleop.yaml)
 * @return linear axis value
 */
const int GamePad_controller::getLinearAxis()
{
  return linearAxis;
}
/**
 * Get the value of the angular axis.
 * This value is either 1(default) or from
 * config file(teleop.yaml)
 * @return angular axis value
 */
const int GamePad_controller::getAngularAxis()
{
  return angularAxis;
}

/**
 * Get the value of the linear scale.
 * This value is either 1.0(default) or from
 * config file(teleop.yaml)
 * @return linear scale value
 */
const double GamePad_controller::getLinearScale()
{
  return linear_scale;
}

/**
 * Get the value of the angular scale.
 * This value is either 1.0(default) or from
 * config file(teleop.yaml)
 * @return angular scale value
 */
const double GamePad_controller::getAngularScale()
{
  return angular_scale;
}

/**
 * (Callback) Function that converts the Joy message received from the
 * gamepad to a Twist message.
 *
 * The Twist message is only generated for differential drive, so only
 * the following messages are used:
 * <li> linear.x  = linear speed </li>
 * <li> angular.z = angle speed </li>
 *
 * @param joy incoming joy message
 */
void GamePad_controller::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  ROS_DEBUG("[Gamepad] command: linear %f angular %f ", joy->axes[this->getLinearAxis()],
            joy->axes[this->getAngularAxis()]);

  //Stop the robot
  if (joy->buttons[stopButton] == 1.0)
  {
    twist.linear.x = 0;
    twist.angular.z = 0;
  }
  else
  {
    twist.linear.x = linear_scale * joy->axes[this->getLinearAxis()];
    twist.angular.z = angular_scale * joy->axes[this->getAngularAxis()];

    //Inverse linear axis values
    if (inverse_linAxis == 1)
    {
      twist.linear.x = -1 * linear_scale * joy->axes[this->getLinearAxis()];
    }
    //Inverse angular axis values
    if (inverse_angAxis == 1)
    {
      twist.angular.z = -1 * angular_scale * joy->axes[this->getAngularAxis()];
    }

  }

  ROS_DEBUG("[Gamepad] Create twist message on topic %s, with values: linear %f, angular %f", vel_pub.getTopic().c_str(), twist.linear.x,
            twist.angular.z);
}

/**
 * Run this ROS node.
 * The (default) frequency that it will run is 30 Hz.
 */
void GamePad_controller::spin()
{
  ROS_INFO("[Gamepad] start gamepad_controller Node");

  ros::Rate loop_rate(30);       //Hz
  while (ros::ok())
  {
    ros::spinOnce();
    //Publish the message
    vel_pub.publish(twist);
    loop_rate.sleep();
  }
}

}

