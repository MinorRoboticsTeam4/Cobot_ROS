/*
 * driver.h
 * 
 */

//=======================================================
// include guard
#ifndef MOVEMENT_INCLUDE_MOVEMENT_DRIVER_H_INCLUDED
#define MOVEMENT_INCLUDE_MOVEMENT_DRIVER_H_INCLUDED

//=======================================================
// predefined headers
#include <ros/ros.h>
#include <tf/transform_listener.h>

//=======================================================
// new defined headers

namespace movement
{
/**
 * This class will read transformations from a robot platform
 * and use it to move that platform around.
 */
class Driver
{
//Typedefs
//public:
//private:

//Public Functions
public:
  /**
   * (Default) Constructor
   */
  Driver();
  /**
   * (Default) Destructor
   */
  ~Driver();

  /**
   * Move the robot platform straight forward. If the distance is made negative,
   * the robot will move backwards.
   * @param distance distance to travel (e.g. in m for 3MxlBoard)
   * @param linearSpeed the speed of the robot
   */
  void drive_straight(const double distance, const double linearSpeed);
  /**
   * Turn the robot counterclockwise around it's axis. If the distance is made negatve,
   * the robot will turn clockwise.
   * @param distance turns to make (always in rad)
   * @param angularSpeed the speed of the robot
   */
  void turn(const double distance, const double angularSpeed);

  /**
   *
   * @return the read in commands
   */
  std::vector<std::string> getCommands();

  /**
   *
   * @return set linear speed
   */
  double getLinearSpeed();

  /**
   *
   * @return set angular speed
   */
  double getAngularSpeed();

//Private Functions
//private:

//Public Variables
//public:

//Private Variables
private:
  /**
   * Publisher to advertise the speed
   */
  ros::Publisher vel_pub;
  /**
   * Ros (interface)
   */
  ros::NodeHandle nh;
  /**
   * Listener for the transformations
   */
  tf::TransformListener TFlistener;

  /**
   * Stores the drive commands figures
   */
  std::vector<std::string> commands;

  /**
   * Linear speed the driver must use
   */
  double linearSpeed;
  /**
   * Angular speed the driver must use
   */
  double angularSpeed;
};
} //namespace end

#endif /* MOVEMENT_INCLUDE_MOVEMENT_DRIVER_H_INCLUDED */
