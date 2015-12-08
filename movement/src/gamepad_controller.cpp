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
#include "movement/gamepad_controller.h"

namespace movement
{

/**
 * GamePadTeleop initializes the variables inside and
 * setups the NodeHandler for publishing and listening.
 *
 * The default publish topic is "/controller_vel".
 * This can be changed by adding the parameter "pub_topic" with
 * the desired topic name.
 *
 */
GamePad_controller::GamePad_controller() :
    linear(0), angular(1), linear_scale(1.0), angular_scale(1.0), pub_name("controller_vel"), nh("~"), twist()
{
  //Read in the params from teleop.yaml
  nh.param<int>("linear_axis", linear, linear);
  nh.param<int>("angular_axis", angular, angular);
  nh.param<double>("linear_scale", linear_scale, linear_scale);
  nh.param<double>("angular_scale", angular_scale, angular_scale);
  nh.param<std::string>("pub_topic", pub_name, pub_name);

  //Publish on a global topic
  vel_pub = nh.advertise<geometry_msgs::Twist>("/" + pub_name, 1);

  ROS_INFO("Publishing on topic: %s", pub_name.c_str());

  //Subscribe to the messages of the gamepad
  //and call joyCallback with incoming messages.
  //Listen to global topic "joy"
  joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &GamePad_controller::joyCallback, this);

  ROS_INFO("Starting gamepad with: linear %d, angular %d, "
           "linear scale %f, angular scale %f",
           linear, angular, linear_scale, angular_scale);
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
  return linear;
}
/**
 * Get the value of the angular axis.
 * This value is either 1(default) or from
 * config file(teleop.yaml)
 * @return angular axis value
 */
const int GamePad_controller::getAngularAxis()
{
  return angular;
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
 * Get the topic where this is publishing to.
 * This value is either "controller_vel"(default) or from
 * config file(teleop.yaml)
 * @return publishing topic
 */
const std::string GamePad_controller::getPublishTopic()
{
  return pub_name;
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
  //TODO inverse axis parameter??
  ROS_DEBUG("Read gamepad command: linear %f angular %f ", joy->axes[this->getLinearAxis()],
            joy->axes[this->getAngularAxis()]);

  twist.linear.x = linear_scale * joy->axes[this->getLinearAxis()];
  twist.angular.z = angular_scale * joy->axes[this->getAngularAxis()];

  ROS_DEBUG("Create twist message on topic %s, with values: linear %f, angular %f", pub_name.c_str(), twist.linear.x,
            twist.angular.z);
}

/**
 * Run this ROS node.
 * The (default) frequency that it will run is 30 Hz.
 */
void GamePad_controller::spin()
{
  ROS_INFO("Started gamepad_controller Node");

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

