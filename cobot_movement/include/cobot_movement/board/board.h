/*
 * board.h
 * 
 */

//=======================================================
// include guard
#ifndef MOVEMENT_INCLUDE_MOVEMENT_BOARD_H_INCLUDED
#define MOVEMENT_INCLUDE_MOVEMENT_BOARD_H_INCLUDED

//=======================================================
// predefined headers
#include <boost/shared_ptr.hpp>
#include <iostream>

//=======================================================
// new defined headers
#include "cobot_movement/robot_params.h"

namespace cobot_movement
{

/**
 * This Interface defines what information and actions a Board must implement.
 * These are needed to be able to move the robot and to read out additional information.
 *
 * The Board class is designed for differential drive only.
 *
 * If the information is not available on the robot, then implement the function
 * and let it return return '0'.
 *
 */
class Board
{
//Typedefs
public:
  typedef boost::shared_ptr<Board> SharedPtr;
//private

//Public Functions
public:
  /**
   * Interface
   * (default) destructor
   */
  virtual ~Board() = 0;
  /**
   * Interface
   *
   * Startup function to initialize (startup) the motors, if needed.
   * @return true if successful
   */
  virtual bool init_motors() = 0;
  /**
   * Interface
   *
   * Startup function to initialize (startup) the connection to the robot,
   * if needed.
   * @param port_address connection address
   * @param baudrate speed of the connection
   * @return true if successful
   */
  virtual bool init_connection(std::string port_address, int baudrate) = 0;

  /**
   * Interface
   *
   * Move the robot around, given the speed of the left and right motor in (m/s)
   * @param v_left speed of the left motor (m/s)
   * @param v_right speed of the right motor (m/s)
   */
  virtual void drive_linearSpeed(double v_left, double v_right) = 0;
  /**
   * Interface
   *
   * Move the robot around, given the speed of the left and right motor in (rad/s)
   * @param v_left speed of the left motor (rad/s)
   * @param v_right speed of the right motor (rad/s)
   */
  virtual void drive_angularSpeed(double v_left, double v_right) = 0;
  /**
   * Interface
   *
   * Stop the robot.
   */
  virtual void stop() = 0;

  /**
   * Interface
   *
   * @return the position of the left motor (m)
   */
  virtual double get_linearPos_left() = 0;
  /**
   * Interface
   *
   * @return the position of the right motor (m)
   */
  virtual double get_linearPos_right() = 0;
  /**
   * Interface
   *
   * @return the position of the left motor (rad)
   */
  virtual double get_angularPos_left() = 0;
  /**
   * Interface
   *
   * @return the position of the right motor (rad)
   */
  virtual double get_angularPos_right() = 0;

  /**
   * Interface
   *
   * @return the parameters of the robot, stored in Robot_Params.
   */
  virtual Robot_Params getParams() = 0;

  //TODO Implement in SensorHandler
  /**
   * Interface
   *
   * @return the status of the left motor
   */
  virtual int get_status_left() = 0;
  /**
   * Interface
   *
   * @return the status of the right motor
   */
  virtual int get_status_right() = 0;

//Private Functions
private:

  /**
   * Translate status codes to human readable output
   * @param status status to translate
   * @return human readable status
   */
  virtual std::string translateStatus(int status) = 0;

//Public Variables
//public:

//Private Variables
//private:
};

} //namespace end

#endif /* MOVEMENT_INCLUDE_MOVEMENT_BOARD_H_INCLUDED */
