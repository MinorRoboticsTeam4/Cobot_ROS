/*
 * motor_controller.cpp
 *
 *  Created on: Nov 12, 2015
 *      Author: aclangerak
 */

//=======================================================
// predefined headers
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <threemxl/platform/hardware/dynamixel/CDxlConfig.h>

//=======================================================
// new defined headers
#include "movement/motor_controller.h"

namespace movement
{


Motor_controller::Motor_controller() :
    motorL(), motorR(), serial_port(), sub_name("cmd_vel"), nh("~"), baseFrame("/base_link"), odomFrame("/odom")
{

  nh.param<std::string>("sub_topic", sub_name, sub_name);
  cmdVel_sub = nh.subscribe("/" + sub_name, 10, &Motor_controller::cmdVelCallback, this);

  ROS_INFO("Listening on topic: %s", sub_name.c_str());

  //Publish on odom topic
  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 2);

}

Motor_controller::~Motor_controller()
{
  if (serial_port->is_port_open())
  {
    serial_port->port_close();
  }

  //Remove motors
  delete motorL;
  delete motorR;

  //Remove connection
  delete serial_port;

  nh.shutdown();

}

bool Motor_controller::init_connection(std::string port_address, int baudrate)
{
  ROS_INFO("Opening connection to: %s ; baudrate: %d", port_address.c_str(), baudrate);

  bool port_open = serial_port->port_open(port_address, LxSerial::RS485_FTDI);
  serial_port->set_speed_int(baudrate);

  return port_open;
}

bool Motor_controller::init_Motors()
{
  if (serial_port->is_port_open())
  {
    CDxlConfig *configL = new CDxlConfig();
    CDxlConfig *configR = new CDxlConfig();

    motorL->setSerialPort(serial_port);
    motorR->setSerialPort(serial_port);

    //TODO find IDs of the motors
    /*    motorL->setConfig(config->setID(100));
     motorL->init(false);

     motorR->setConfig(config->setID(101));
     motorR->init(false);*/

    delete configL;
    delete configR;

  }
  else
  {
    ROS_ERROR("The serial port is not open");
    return false;
  }

  return true;
}

void Motor_controller::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &twist)
{
  double v = twist->linear.x;                           //(m/s)
  double th = twist->angular.z;                         //(rad/s)

  double v_left = v - th * (AXLE_TRACK / 2.0);          //(m/s)
  double v_right = v + th * (AXLE_TRACK / 2.0);         //(m/s)

  drive(v_left, v_right);
}

void Motor_controller::drive(double v_left, double v_right)
{

  //TODO Clamping to maximum speed
  //TODO Check bumper is hit
  //TODO Check for emergency button
  //TODO Timeout when no commands are received in x time

  motorL->setLinearSpeed(v_left);
  motorR->setLinearSpeed(v_right);
}

void Motor_controller::updateOdomTF()
{

}

void Motor_controller::spin()
{

  ros::Rate loop_rate(1); // 1 Hz

  while (ros::ok())
  {
    ros::spinOnce();

    //Do additional work here

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
  ros::init(argc, argv, "motor_controller");

//Initialize motor controller and open port
  movement::Motor_controller motor_controller;
  motor_controller.init_connection("USB ID");
  motor_controller.init_Motors();

//Run ros node
  motor_controller.spin();

  return EXIT_SUCCESS;
}

