/*
 * threeMxlboard.h
 * 
 */

//=======================================================
// include guard
#ifndef MOVEMENT_INCLUDE_MOVEMENT_THREEMXLBOARD_H_INCLUDED
#define MOVEMENT_INCLUDE_MOVEMENT_THREEMXLBOARD_H_INCLUDED

//=======================================================
// predefined headers
#include <threemxl/C3mxl.h>
#include <threemxl/platform/hardware/dynamixel/3mxl/3mxlControlTable.h>
#include <boost/scoped_ptr.hpp>

//=======================================================
// new defined headers
#include "cobot_movement/board/board.h"

namespace cobot_movement
{

/**
 * This Board uses the (physical) 3MxlBoard together with the library.
 */
class ThreeMxlBoard : public Board
{
//Enums
public:
  enum Baudrate
  {
    S4800 = LxSerial::S4800, S9600 = LxSerial::S9600, S19200 = LxSerial::S19200
  };

  //all these statuses are found from the 3mxlControlTable.h
  //and only status codes that are used directly in the code are stored here
  enum Status
  {
    STATUS_OK = M3XL_NO_ERROR, STATUS_EM_STOP = M3XL_STATUS_EM_STOP_ERROR, STATUS_OVERHEATING = M3XL_OVERHEATING_ERROR

  };

//Typedefs
//public:
private:
  /**
   * Pointer to the motors interface
   */
  typedef boost::scoped_ptr<CDxlGroup> MotorsPtr;

//Public Functions
public:
  /**
   * (Default) Constructor
   */
  ThreeMxlBoard();
  /**
   * (Default) Destructor
   */
  ~ThreeMxlBoard();
  /**
   * Initialize the motors and connect then to the 3mxlboard if connection is open.
   * @return true if successful
   */
  bool init_motors();
  /**
   * Initialize the connection with the 3mxl board.
   * @param port_address address of the port
   * @param baudrate speed of the connection
   * @return true if successful
   */
  bool init_connection(std::string port_address, int baudrate);
  /**
   * Set the speed of the left motor and the right motor in (m/s)
   * @param v_left speed of the left motor (m/s)
   * @param v_right speed of the right motor (m/s)
   */
  void drive_linearSpeed(double v_left, double v_right);
  /**
   * Set the speed of the left motor and the right motor in (rad/s)
   * @param v_left speed of the left motor (rad/s)
   * @param v_right speed of the right motor (rad/s)
   */
  void drive_angularSpeed(double v_left, double v_right);
  /**
   * Stop the robot
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
   * @return the parameters of the robot, stored in Robot_Params.
   */
  Robot_Params getParams();

  /**
   * @return the status of the left motor
   */
  int get_status_left();
  /**
   * @return the status of the right motor
   */
  int get_status_right();

//Private Functions
private:

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
   * Motors interface
   */
  MotorsPtr motors;
  /**
   * Connection interface
   */
  LxSerial serial_port;
  /**
   * Stored robot parameters
   */
  Robot_Params robotParams;
};

} //namespace end

#endif /* MOVEMENT_INCLUDE_MOVEMENT_THREEMXLBOARD_H_INCLUDED */
