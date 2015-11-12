/*
 * gamepad_controller.h
 *
 */

//=======================================================
// include guard
#ifndef MOVEMENT_INCLUDE_MOVEMENT_GAMEPAD_CONTROLLER_H_
#define MOVEMENT_INCLUDE_MOVEMENT_GAMEPAD_CONTROLLER_H_

//=======================================================
// predefined headers
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

namespace movement
{

/**
 * This class contains the following:
 * <li> A Nodehlander for interfacing with ROS
 * <li> A callback function to convert joy -> twist messages.
 * <li> Parameters read from teleop.yaml: linear,angular and scale
 */
class GamePadTeleop
{
public:

  /**
   * Default (empty) constructor
   */
  GamePadTeleop();

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

private:

  /**
   * (Callback) Function to convert Joy -> Twist messages.
   * @param joy  incoming joy message
   */
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  /**
   * NodeHandler for ROS usage
   */
  ros::NodeHandle nh;

  /**
   * current linear value
   */
  int linear;

  /**
   * current angular value
   */
  int angular;

  /**
   * linear scale factors
   */
  double linear_scale;

  /**
   * angular scale factors
   */
  double angular_scale;

  /**
   * Publisher published messages.
   */
  ros::Publisher vel_pub;

  /**
   * (Default) publisher topic for vel_pub
   */
  std::string pub_name;

  /**
   * Subscriber listens to messages and calls a callback messages.
   */
  ros::Subscriber joy_sub;

};
}

#endif /* MOVEMENT_INCLUDE_MOVEMENT_GAMEPAD_CONTROLLER_H_ */
