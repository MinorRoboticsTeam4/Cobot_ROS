/*
 * TurtleBoard.cpp
 * 
 */

//=======================================================
// predefined headers
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

//=======================================================
// new defined headers
#include "cobot_movement/board/turtleBoard.h"

namespace cobot_movement
{

/**
 * Create a new Board based on the ThreeMxlBoard with following robot parameters:
 *
 * <li>  axleTrack = 1.0 m </li>
 * <li>  wheelDiameter = 1.0m </li>
 * <li>  cmdVelTimeout = 5.0s </li>
 * <li>  max linear speed = 0.5 m/s </li>
 * <li>  max reverse linear speed = 0.5 m/s </li>
 */
TurtleBoard::TurtleBoard() :
    robotParams(1.0d, 1.0d, 5.0d, 0.5d, 0.5d)
{
}

/**
 * (Default) Destructor
 */
TurtleBoard::~TurtleBoard()
{
}

/**
 * (Stub) no function
 */
bool TurtleBoard::init_connection(std::string port_address, int baudrate)
{
  return true;
}

/**
 * Connect to the TurtleSim environment
 * @return true
 */
bool TurtleBoard::init_motors()
{
  turtle_sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose", 10, &TurtleBoard::poseCb, this);
  turtlespeed_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
  return true;
}

/**
 * Callback to get the pose of the turtle from the environment
 * @param msg pose of the turtle
 */
void TurtleBoard::poseCb(turtlesim::Pose msg)
{
  pos2d.x = msg.x;
  pos2d.y = msg.y;
  pos2d.theta = msg.theta;
}

/**
 * Move the turtle around in (m/s)
 * @param v_left speed of the turtle
 * @param v_right (stub), does noting
 */
void TurtleBoard::drive_linearSpeed(double v_left, double v_right)
{
  geometry_msgs::Twist twist;

  v_left = clip(v_left, getParams().MAX_REV_LIMIT_LINSPEED, getParams().MAX_LIMIT_LINSPEED);
  v_right = clip(v_right, getParams().MAX_REV_LIMIT_LINSPEED, getParams().MAX_LIMIT_LINSPEED);

  twist.linear.x = (v_left + v_right) / 2.0d;
  twist.angular.z = (v_right - v_left) / ((double)getParams().AXLE_TRACK);

  turtlespeed_pub.publish(twist);
}

/**
 * (Stub) not used
 */
void TurtleBoard::drive_angularSpeed(double v_left, double v_right)
{
}

/**
 * Stop the turtle from moving
 */
void TurtleBoard::stop()
{
  geometry_msgs::Twist msg;
  msg.linear.x = 0.0d;
  msg.linear.y = 0.0d;
  msg.linear.z = 0.0d;
  msg.angular.x = 0.0d;
  msg.angular.y = 0.0d;
  msg.angular.z = 0.0d;
  turtlespeed_pub.publish(msg);
}
/**
 *
 * @return (simulated) left linear position of the turtle
 */
double TurtleBoard::get_linearPos_left()
{
  double dist = sqrt(pow(pos2d.x, 2) + pow(pos2d.y, 2)) - ((getParams().AXLE_TRACK / 2.0d) + pos2d.theta) / 2;
  return dist;
}
/**
 *
 @return (simulated) right linear position of the turtle
 */
double TurtleBoard::get_linearPos_right()
{
  double dist = sqrt(pow(pos2d.x, 2) + pow(pos2d.y, 2)) + ((getParams().AXLE_TRACK / 2.0d) + pos2d.theta) / 2;
  return dist;
}
/**
 * (Stub) does nothing
 */
double TurtleBoard::get_angularPos_left()
{
  return 0;
}
/**
 * (Stub) does nothing
 */
double TurtleBoard::get_angularPos_right()
{
  return 0;
}

/**
 *
 * @return the set parameters of the "turtle platform"
 */
Robot_Params TurtleBoard::getParams()
{
  return robotParams;
}

/**
 * (stub) does nothing
 */
int TurtleBoard::get_status_left()
{
  return 0;
}
/**
 * (stub) does nothing
 */
int TurtleBoard::get_status_right()
{
  return 0;
}

/**
 * Translate status codes to human readable status codes
 *
 * @param status status of the motor
 * @return readable code of status
 */
std::string TurtleBoard::translateStatus(int status)
{
  switch(status)
  {
    default: return "Not Implemented" ;
  };
}

}
