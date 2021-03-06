/*
 * gamepad_controller.h
 *
 */

//=======================================================
// include guard
#ifndef MOVEMENT_INCLUDE_MOVEMENT_GAMEPAD_CONTROLLER_H_INCLUDED
#define MOVEMENT_INCLUDE_MOVEMENT_GAMEPAD_CONTROLLER_H_INCLUDED

//=======================================================
// predefined headers
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace movement
{

/**
 * This class contains the following:
 * <li> A Nodehandler for interfacing with ROS </li>
 * <li> A callback function to convert joy -> twist messages. </li>
 * <li> Parameters read from teleop.yaml: linear,angular and scale </li>
 */
class GamePad_controller
{

//Typedefs
//public:
//private:

//Public Functions
public:
  /**
   * Default constructor
   */
  GamePad_controller();

  /**
   * Default destructor
   */
  ~GamePad_controller();

  /**
   * Get the linear axis
   * @return linear axis value
   */
  const int getLinearAxis();

  /**
   * Get the angular axis
   * @return angular axis value
   */
  const int getAngularAxis();

  /**
   * Get the linear scale
   * @return linear scale value
   */
  const double getLinearScale();

  /**
   * Get the angular scale
   * @return angular scale value
   */
  const double getAngularScale();

  /**
   * Run the ROS node.
   */
  void spin();

//Private Functions
private:
  /**
   * (Callback) Function to convert Joy -> Twist messages.
   * @param joy  incoming joy message
   */
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

//Public Variables
//public:

//Private Variables
private:
  /**
   * Axis used for linear control
   */
  int linearAxis;

  /**
   * Axis used for angular control
   */
  int angularAxis;

  /**
   * Button to be used to stop the robot
   */
  int stopButton;

  /**
   * Inverse linear axis readout (0 false, 1 true)
   */
  int inverse_linAxis;

  /**
   * Inverse angular axis readout (0 false, 1 true)
   */
  int inverse_angAxis;

  /**
   * linear scale factors
   */
  double linear_scale;

  /**
   * angular scale factors
   */
  double angular_scale;

  /**
   * Personal NodeHandler for ROS usage
   */
  ros::NodeHandle nh;

  /**
   * Publisher published messages.
   */
  ros::Publisher vel_pub;

  /**
   * Subscriber listens to messages and calls a callback messages.
   */
  ros::Subscriber joy_sub;

  /**.
   * Twist message to send
   */
  geometry_msgs::Twist twist;

};

}  //namespace end

#endif /* MOVEMENT_INCLUDE_MOVEMENT_GAMEPAD_CONTROLLER_H_ */
