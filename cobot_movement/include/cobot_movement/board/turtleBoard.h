/*
 * dummyBoard.h
 * 
 */

//=======================================================
// include guard
#ifndef MOVEMENT_INCLUDE_MOVEMENT_TurtleBoard_H_INCLUDED
#define MOVEMENT_INCLUDE_MOVEMENT_TurtleBoard_H_INCLUDED

//=======================================================
// predefined headers
#include <turtlesim/Pose.h>
#include <geometry_msgs/Pose2D.h>

//=======================================================
// new defined headers
#include "cobot_movement/board/board.h"

namespace cobot_movement
{

/**
 * This Board uses the ROS Turtlesim environment for controlling the "robot".
 */
class TurtleBoard : public Board
{

//Typedefs
//public:
//private:

//Public Functions
public:
  /**
   * (Default) Constructor
   */
  TurtleBoard();
  /**
   * (Default) Destructor
   */
  ~TurtleBoard();
  /**
   * Connect to the TurtleSim environment
   * @return true
   */
  bool init_motors();
  /**
   * (Stubbed method, does nothing)
   */
  bool init_connection(std::string port_address, int baudrate);
  /**
   * Move the turtle around in (m/s)
   * @param v_left speed of the turtle
   * @param v_right speed of the turtle
   */
  void drive_linearSpeed(double v_left, double v_right);
  /**
   * (Stub) not used
   */
  void drive_angularSpeed(double v_left, double v_right);
  /**
   * stop the turtle
   */
  void stop();

  /**
   *
   * @return (simulated) left linear position of the turtle
   */
  double get_linearPos_left();
  /**
   *
   @return (simulated) right linear position of the turtle
   */
  double get_linearPos_right();
  /**
   * (stub) does nothing
   */
  double get_angularPos_left();
  /**
   * (stub) does nothing
   */
  double get_angularPos_right();

  /**
   *
   * @return the set parameters of the "turtle platform"
   */
  Robot_Params getParams();

  /**
   * (stub) does nothing
   */
  int get_status_left();
  /**
   * (stub) does nothing
   */
  int get_status_right();

//Private Functions
private:
  /**
   * Callback to get the pose of the turtle from the environment
   * @param msg pose of the turtle
   */
  void poseCb(turtlesim::Pose msg);

  /** Clip the value between min<=x<=max
   *
   * @param n value to be clipped
   * @param lower lower bound
   * @param upper upper bound
   * @return clipped value
   */
  template<typename T>
    T clip(const T& n, const T& lower, const T& upper)
    {
      return std::max(lower, std::min(n, upper));
    }

  /**
   * Translate status codes to human readable status codes
   *
   * @param status status of the motor
   * @return readable code of status
   */
  std::string translateStatus(int status);

//Public Variables
//public:

//Private Variables
private:
  /**
   * Stored "turtle platform" parameters
   */
  Robot_Params robotParams;
  /**
   * Subscriber for the Pose of the turtle
   */
  ros::Subscriber turtle_sub;
  /**
   * Publisher for the speed of the turtle
   */
  ros::Publisher turtlespeed_pub;
  /**
   * Ros (interface)
   */
  ros::NodeHandle nh;

  /**
   * The pose of the turtle
   */
  geometry_msgs::Pose2D pos2d;

};

} //namespace end

#endif /* MOVEMENT_INCLUDE_MOVEMENT_TurtleBoard_H_INCLUDED */
