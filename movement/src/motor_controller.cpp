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

/**
 * Default constructor, setups the Nodehandler for publishing / listening.
 *
 * The default topic to listen to is "/cmd_vel"
 * This can be changed by adding the parameter "sub_topic" with
 * the desired topic name.
 *
 * Additionally, it defines the base frame("/base_link") and the odometry frame("/odom") needed for tf
 */
Motor_controller::Motor_controller() :
    motorL(), motorR(), serial_port(), sub_name("cmd_vel"), nh("~"), baseFrame("/base_link"), odomFrame("/odom"), last_cmd_vel_time(
        0)
{

  nh.param<std::string>("sub_topic", sub_name, sub_name);
  cmdVel_sub = nh.subscribe("/" + sub_name, 10, &Motor_controller::cmdVelCallback, this);

  ROS_INFO("Listening on topic: %s", sub_name.c_str());

  //Publish on odom topic
  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 2);

}

/**
 * The destructor closes the serial port if it is open.
 *
 * It also deletes the left and right motor interfaces,
 * serial port.
 *
 * Closes down the Nodehandler
 */
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

/**
 * Open a new connection to the 3Mxel board.
 * You can find the port_address by typing  "lsusb" in the Linux terminal.
 *
 * @param port_address  USB ID of the board
 * @param baudrate  the baud rate(default = 921600)
 * @return true if connection was successful
 */
bool Motor_controller::init_connection(std::string port_address, int baudrate)
{
  ROS_INFO("Opening connection to: %s ; baudrate: %d", port_address.c_str(), baudrate);

  bool port_open = serial_port->port_open(port_address, LxSerial::RS485_FTDI);
  serial_port->set_speed_int(baudrate);

  return port_open;
}

/**
 * Initialize the motors with serial port, id, set3MxlMode
 *
 * <li> left motor id:
 * <li> right motor id:
 *
 *  3MxlMode: SPEED_MODE
 *
 * @return true if succesful
 */
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
     motorL->set3MxlMode(SPEED_MODE);

     motorR->setConfig(config->setID(101));
     motorR->init(false);
     motorR->set3MxlMode(SPEED_MODE);
     */

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

/**
 * (Callback) Function to convert Twist->motor velocity commands
 *
 * It only uses the linear.x and angular.z information, because
 * this robot can only move forwar/backward and can do rotations.
 *
 * @param twist  Twist message to respond to
 */
void Motor_controller::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &twist)
{
  double v = twist->linear.x;                           //(m/s)
  double th = twist->angular.z;                         //(rad/s)

  double v_left = v - th * (AXLE_TRACK / 2.0);          //(m/s)
  double v_right = v + th * (AXLE_TRACK / 2.0);         //(m/s)

  //Reset time for timeout
  last_cmd_vel_time = ros::Time::now();

  drive(v_left, v_right);
}

/**
 * Give the command to drive this robot.
 *
 * It checks if the bumper(outer shell) is touched
 * It checks if the emergency button is checked.
 *
 * @param v_left speed of left motor (m/s)
 * @param v_right speed of right motor (m/s)
 */
void Motor_controller::drive(double v_left, double v_right)
{
  //TODO Check bumper is hit

  //TODO Test if this works
  if (motorL->getLastError() == M3XL_STATUS_EM_STOP_ERROR || motorR->getLastError() == M3XL_STATUS_EM_STOP_ERROR)
  {
    ROS_WARN("Emergency Button pressed, can't set speed");
  }
  else
  {
    //Maybe check if speed is clamped?
    motorL->setLinearSpeed(v_left);
    motorR->setLinearSpeed(v_right);
  }
}

void Motor_controller::updateOdomTF()
{

}

/**
 * Run this node
 */
void Motor_controller::spin()
{

  ros::Rate loop_rate(UPDATE_RATE);

  while (ros::ok())
  {
    ros::spinOnce();

    //Do additional work here

    ros::Time current_time = ros::Time::now();

    //Timeout has occurred, stop
    //TODO needs verification
    if (current_time.sec - last_cmd_vel_time.sec > CMD_VEL_TIMEOUT)
    {
      drive(0, 0);
      ROS_DEBUG("No cmd_vel received, Timeout");
    }

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

