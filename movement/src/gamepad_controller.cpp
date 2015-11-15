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
 * The default publish topic is "controller_vel".
 * This can be changed by adding the parameter "pub_topic" with
 * the desired topic name.
 *
 */
GamePad_controller::GamePad_controller() :
    linear(0), angular(1), linear_scale(1.0), angular_scale(1.0), pub_name("controller_vel")
{
  //Read in the params from teleop.yaml
  //cannot get private namespaces to work,
  //so using this method instead
  nh.param<int>("gamepad_controller/linear_axis", linear, linear);
  nh.param<int>("gamepad_controller/angular_axis", angular, angular);
  nh.param<double>("gamepad_controller/linear_scale", linear_scale, linear_scale);
  nh.param<double>("gamepad_controller/angular_scale", angular_scale, angular_scale);
  nh.param<std::string>("gamepad_controller/pub_topic", pub_name, pub_name);

  vel_pub = nh.advertise<geometry_msgs::Twist>(pub_name, 1);

  ROS_INFO("Publishing on topic: %s", pub_name.c_str());

  //Subscribe to the messages of the gamepad
  //and call joyCallback with incoming messages.
  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &GamePad_controller::joyCallback, this);

  ROS_INFO("Starting gamepad with: linear %d, angular %d, "
           "linear scale %f, angular scale %f",
           linear, angular, linear_scale, angular_scale);
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
 * The Twist message that is only generated for differential drive, so only
 * the following messages are defined:
 * <li> linear.x  = linear speed
 * <li> angular.z = angle speed
 *
 * @param joy   incoming joy message
 */
void GamePad_controller::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

  ROS_DEBUG("Read gamepad command: linear %f angular %f ", joy->axes[this->getLinearAxis()],
            joy->axes[this->getAngularAxis()]);

  //New twist message
  geometry_msgs::Twist twist;

  twist.linear.x = linear_scale * joy->axes[this->getLinearAxis()];
  twist.angular.z = angular_scale * joy->axes[this->getAngularAxis()];

  //Publish the message
  vel_pub.publish(twist);

  ROS_DEBUG("Publish twist message: linear %f, angular %f", twist.linear.x, twist.angular.z);
}

/**
 * Run this ROS node.
 * The (default) frequency that it will run is 10 Hz.
 */
void GamePad_controller::spin()
{
  ROS_INFO("Started gamepad_controller Node");

  ros::Rate loop_rate(10);       //10 Hz
  while (ros::ok())
  {
    //Place additional run code for ROS here.
    ros::spinOnce();
    loop_rate.sleep();
  }
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
  ros::init(argc, argv, "gamepad_controller");

  //Initialize gamepad_controller class
  movement::GamePad_controller gamepad_controller;
  gamepad_controller.spin();

  return EXIT_SUCCESS;
}

