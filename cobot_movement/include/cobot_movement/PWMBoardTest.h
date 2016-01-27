/*
 * PWMBoardTest.h
 */

//=======================================================
// include guard
#ifndef MOVEMENT_INCLUDE_MOVEMENT_TEST3MXLBOARD_H_INCLUDED
#define MOVEMENT_INCLUDE_mOVEMENT_TEST3MXLBOARD_H_INCLUDED

//=======================================================
// predefined headers
#include <ros/ros.h>
#include <threemxl/platform/hardware/dynamixel/CDxlGeneric.h>
#include <threemxl/platform/hardware/dynamixel/CDxlGroup.h>

//=======================================================
// new defined headers

/**
 * Testing class for 3mxl board
 */
class PWMBoardTest
{
private:
  /**
   * ROS node handle
   */
  ros::NodeHandle nh;
  /**
   * Multiple motor interface
   */
  CDxlGroup *motors;
  /**
   * Serial port interface
   */
  LxSerial serial_port;

  /**
   * Maximal PWM(or speed) the motors has to climb to
   */
  double maxPWM;

public:
  /**
   * (Default) Constructor
   */
  PWMBoardTest();

  /**
   * (Default) Destructor
   *
   */
  ~PWMBoardTest();

  /**
   * Initialize node
   */
  void init(char *maxpwm);

  /**
   * Set speed of motors
   * @param v_left left speed (PWM or SPEED)
   * @param v_right right speed (PWM or SPEED)
   */
  void setSpeed(double v_left, double v_right);

  /**
   * Check if the emergency stop is active and
   * show other statuses
   */
  void check_EM_STOP();

  /**
   * Run this node
   */
  void spin();
};

#endif /*MOVEMENT_INCLUDE_MOVEMENT_TEST3MXLBOARD_H_INCLUDED*/
