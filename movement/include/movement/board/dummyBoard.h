/*
 * dummyBoard.h
 * 
 */

//=======================================================
// include guard
#ifndef MOVEMENT_INCLUDE_MOVEMENT_DUMMYBOARD_H_INCLUDED
#define MOVEMENT_INCLUDE_MOVEMENT_DUMMYBOARD_H_INCLUDED

//=======================================================
// predefined headers

//=======================================================
// new defined headers
#include "movement/board/board.h"

namespace movement
{

/**
 * This Board is an example Board.
 * The only thing it does is echoing the called functions to the terminal.
 */
class DummyBoard : public Board
{

//Enums
public:
  /**
   * Enumerators that defines the status codes
   */
  enum Status
  {
    MOTOR_OK = 0x01, MOTOR_OVERHEATING = 0x16
  };

//Typedefs
//public:
//private:

//Public Functions
public:
  /**
   * (Default) Constructor
   */
  DummyBoard();
  /**
   * (Default) Destructor
   */
  ~DummyBoard();
  /**
   * Start the motors
   * @return true
   */
  bool init_motors();
  /**
   * Starts the connection
   * @param port_address address of the port
   * @param baudrate speed of the connection
   * @return true
   */
  bool init_connection(std::string port_address, int baudrate);
  /**
   * Set the speed of the board
   * @param v_left left speed (m/s)
   * @param v_right right speed (m/s)
   */
  void drive_linearSpeed(double v_left, double v_right);
  /**
   * Set the speed of the board
   * @param v_left left speed (rad/s)
   * @param v_right right speed (rad/s)
   */
  void drive_angularSpeed(double v_left, double v_right);
  /**
   * Stop the robot.
   */
  void stop();

  /**
   *
   * @return the position of the left motor (m)
   */
  double get_linearPos_left();
  /**
   *
   * @return the position of the right motor (m)
   */
  double get_linearPos_right();
  /**
   *
   * @return the position of the left motor (rad)
   */
  double get_angularPos_left();
  /**
   *
   * @return the position of the right motor (rad)
   */
  double get_angularPos_right();

  /**
   * Get the parameters of the robot
   */
  Robot_Params getParams();

  /**
   *
   * @return the status of the left motor
   */
  int get_status_left();
  /**
   *
   * @return the status of the right motor
   */
  int get_status_right();

//Private Functions
private:

  /**
   * Translate status codes to human readable output
   * @param status status to translate
   * @return human readable status
   */
  std::string translateStatus(int status);

//Public Variables
//public:

//Private Variables
private:
  Robot_Params robotParams;

};

} //namespace end

#endif /* MOVEMENT_INCLUDE_MOVEMENT_DUMMYBOARD_H_INCLUDED */
