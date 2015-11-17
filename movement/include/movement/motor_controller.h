/*
 * motor_controller.h
 *
 *  Created on: Nov 13, 2015
 *      Author: aclangerak
 */

//=======================================================
// include guard
#ifndef MOVEMENT_INCLUDE_MOVEMENT_MOTOR_CONTROLLER_H_
#define MOVEMENT_INCLUDE_MOVEMENT_MOTOR_CONTROLLER_H_

//=======================================================
// predefined headers
#include <threemxl/C3mxl.h>
#include <threemxl/LxFTDI.h>

//=======================================================
// new defined headers
#include "robot_params.h"


namespace movement
{

/**
 * This class is responsible for:
 * <li> Translate Twist messages to motor control velocity commands
 * <li> Read motor information e.g. encoder information
 * <li> Publish odometry and TF
 *
 */
class Motor_controller
{
public:

  /**
   * Default constructor
   */
  Motor_controller();

  /**
   * Default destructor
   */
  ~Motor_controller();

  /**
   * Open a new connection to the 3Mxel board.
   *
   * @param port_address  USB ID of the board, (use lsusb to find it)
   * @param baudrate  the baud rate(default = 921600)
   * @return true if connection was successful
   */
  bool init_connection(std::string port_address, int baudrate = 921600);

  bool init_Motors();

  void spin();

private:

  /**
   * (Callback) Function to convert Twist->motor velocity commands
   * @param twist  Twist message to respond to
   */
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &twist);


  void drive(double v_left, double v_right);


  /**
   * Calculate the odometry + tf and publish the result
   */
  void updateOdomTF();

  /**
   * The name of the Twist topic to listen to
   */
  std::string sub_name;

  /**
   *Name of the base frame (tf)
   */
  std::string baseFrame;

  /**
   * Name of the odom frame (tf)
   */
  std::string odomFrame;

  /**
   * The left motor object
   */
  C3mxl *motorL;

  /**
   * The right motor object
   */
  C3mxl *motorR;

  /**
   * Serial port for communication
   */
  LxSerial *serial_port;

  /**
   * NodeHandler for ROS usage
   */
  ros::NodeHandle nh;

  /**
   * A Subscriber that listens for Twist messages
   */
  ros::Subscriber cmdVel_sub;

  /**
   * A Publisher that sends odometry data
   */
  ros::Publisher odom_pub;


};

}

#endif /* MOVEMENT_INCLUDE_MOVEMENT_MOTOR_CONTROLLER_H_ */
