/*
 * dummyBoard.cpp
 * 
 */

//=======================================================
// predefined headers
#include <iostream>

//=======================================================
// new defined headers
#include "movement/board/dummyBoard.h"

namespace movement
{

/**
 * Create a new Dummyboard with following robot parameters:
 *
 * <li> axleTrack = 1.0 m </li>
 * <li>  wheelDiameter = 10m </li>
 * <li>  cmdVelTimeout = 8.0s </li>
 * <li>  max linear speed = 0.3 m/s </li>
 * <li>  max reverse linear speed = 0.3 m/s </li>
 */
DummyBoard::DummyBoard() :
    robotParams(1.0d, 10.0d, 8.0d, 0.3d, 0.3d)
{
  std::cout << "Constructor DummyBoard called" << std::endl;
}

/**
 * Destruct the Board
 */
DummyBoard::~DummyBoard()
{
  std::cout << "Destructor DummyBoard called" << std::endl;
}

/**
 * (Stub) Suppose to create a connection
 * @param port_address supposed to be the port address
 * @param baudrate supposed to be the speed of the connection
 * @return  Suppose to return true if succesfull
 */
bool DummyBoard::init_connection(std::string port_address, int baudrate)
{
  std::cout << "Opening connection on: " << port_address << " : " << baudrate << std::endl;
  return true;
}

/**
 * (Stub) Suppose to stop the motors
 */
void DummyBoard::stop()
{
  std::cout << "stop" << std::endl;
}

/**
 * (Stub) Suppose to set settings to use the motors
 * @return Suppose to return true if succesfull
 */
bool DummyBoard::init_motors()
{
  std::cout << "Initialize motors" << std::endl;
  return true;
}

/**
 * Suppose to set the speed of the motors
 * @param v_left speed of left motor (m/s)
 * @param v_right speed of the right motor (m/s)
 */
void DummyBoard::drive_linearSpeed(double v_left, double v_right)
{
  std::cout << "Driving with leftSpeed: " << v_left << " (m/s) rightSpeed: " << v_right << "(m/s)" << std::endl;
}
/**
 * Suppose to set the speed of the motors
 * @param v_left speed of left motor (rad/s)
 * @param v_right speed of the right motor (rad/s)
 */
void DummyBoard::drive_angularSpeed(double v_left, double v_right)
{
  std::cout << "Driving with leftSpeed: " << v_left << " (rad/s) rightSpeed: " << v_right << "(rad/s)" << std::endl;
}

/**
 * Suppose to return the position the left wheel
 * @return position in (m)
 */
double DummyBoard::get_linearPos_left()
{
  std::cout << "linear distance left is " << 21 << std::endl;
  return 21;
}
/**
 * Suppose to return the position the right wheel
 * @return position in (m)
 */
double DummyBoard::get_linearPos_right()
{
  std::cout << "linear distance right is " << 21.03 << std::endl;
  return 21.03;
}
/**
 * Suppose to return the position the left wheel
 * @return position in (rad)
 */
double DummyBoard::get_angularPos_left()
{
  std::cout << "angular distance left is " << 6.68 << std::endl;
  return 6.68;
}
/**
 * Suppose to return the position the right wheel
 * @return position in (rad)
 */
double DummyBoard::get_angularPos_right()
{
  std::cout << "angular distance right is " << 6.69 << std::endl;
  return 6.69;
}

/**
 * @return returns the parameters of this board(robot)
 */
Robot_Params DummyBoard::getParams()
{
  return robotParams;
}

/**
 * Suppose to return the status of the left motor
 * @return status
 */
int DummyBoard::get_status_left()
{
  std::cout << "status left is " << translateStatus(MOTOR_OK) << std::endl;
  return MOTOR_OK;
}

/**
 * Suppose to return the status of the left motor
 * @return status
 */
int DummyBoard::get_status_right()
{
  std::cout << "status right is " << translateStatus(MOTOR_OVERHEATING) << std::endl;
  return MOTOR_OVERHEATING;
}

/**
 * Translate status codes to human readable output
 * @param status status to translate
 * @return human readable status
 */
std::string DummyBoard::translateStatus(int status)
{
  switch(status)
  {
    case MOTOR_OK:          return "Motor OK";
    case MOTOR_OVERHEATING: return "Motor Overheating";
    default:                return "Unknown status code";
  }
}


}
