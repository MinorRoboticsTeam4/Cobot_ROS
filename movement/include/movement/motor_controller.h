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

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

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
   * @param port_address  USB ID of the board
   * @param baudrate  the baud rate(default = 921600)
   * @return true if connection was successful
   */
  bool init_connection(std::string port_address, int baudrate = 921600);

  /**
   * Initialize the motors with serial port, id, set3MxlMode
   *
   * @return true if succesful
   */
  bool init_Motors();

  /**
   * Run this node
   */
  void spin();

private:

  /**
   * (Callback) Function to convert Twist->motor velocity commands
   * @param twist  Twist message to respond to
   */
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &twist);

  /**
   * Give the command to drive this robot.
   * @param v_left speed of left motor (m/s)
   * @param v_right speed of right motor (m/s)
   */
  void drive(double v_left, double v_right);

  /**
   * Calculates Odometry information and publishes both Odom and tf transform
   */
  void updateOdom();

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



  //TODO For Delta distances maybe not Needed
  double last_dist_left;
  double last_dist_right;
  double last_angle_left;
  double last_angle_right;

  CDxlGroup *motors;

  double v_left;
  double v_right;

  /**
   * Serial port for communication
   */
  //LxSerial serial_port;
  LxSerial serial_port;

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

  /**
   * Time that last velocty command is send.
   */
  ros::Time last_cmd_vel_time;

  /**
   * Last time Odometry was updated.
   */
  ros::Time last_odom_time;

  /**
   * Current position of the robot
   */
  geometry_msgs::Pose2D pos2d;

};

}

#endif /* MOVEMENT_INCLUDE_MOVEMENT_MOTOR_CONTROLLER_H_ */
