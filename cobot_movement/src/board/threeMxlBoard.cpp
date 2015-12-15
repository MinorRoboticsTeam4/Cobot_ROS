/*
 * threeMxlBoard.cpp
 * 
 */

//=======================================================
// predefined headers
#include <iostream>
#include <ros/ros.h>

//=======================================================
// new defined headers
#include "cobot_movement/board/threeMxlBoard.h"
#include "cobot_movement/robot_params.h"

namespace cobot_movement
{

//TODO read from file??
/**
 * Create a new Board based on the ThreeMxlBoard with following robot parameters:
 *
 * <li>  axleTrack = 0.6 m </li>
 * <li>  wheelDiameter = 0.297m </li>
 * <li>  cmdVelTimeout = 2.0s </li>
 * <li>  max linear speed = 1.0 m/s </li>
 * <li>  max reverse linear speed = 1.0 m/s </li>
 */
ThreeMxlBoard::ThreeMxlBoard() :
    motors(new CDxlGroup()), robotParams(0.600d, 0.297d, 2.0d, 1.0d, 1.0d)
{
}

/**
 * First, it will stop the motors
 * It will then remove the motors interface and
 * finally close the connection.
 */
ThreeMxlBoard::~ThreeMxlBoard()
{
  //Make sure that robot isn't moving
  stop();

  motors.~scoped_ptr();

  //Close port
  if (serial_port.is_port_open())
  {
    serial_port.port_close();
  }

}

/**
 * Initialize the connection to the 3MxlBoard
 *
 * The port address (or actually device address in this case)
 * is found be using dmesg in the terminal right after the board
 * is plugged in.
 *
 * @param port_address address of the board (device address)
 * @param baudrate speed of the connection
 * @return true, if successful
 */
bool ThreeMxlBoard::init_connection(std::string port_address, int baudrate)
{
  ROS_INFO("Opening connection on: %s : %d", port_address.c_str(), baudrate);

  bool is_open = serial_port.port_open("/dev/ttyUSB0", LxSerial::RS485_FTDI);
  serial_port.set_speed_int(baudrate);
  return is_open;
}

/**
 * Initialize the motors interface and add the established connection
 * to the interface.
 * The motors are set to SPEED_MODE
 *
 * The first motor from the motors interface array is the left motor,
 * The second motor from the motors interface array is the right motor.
 *
 * @return true if successful
 */
bool ThreeMxlBoard::init_motors()
{
  ROS_DEBUG("Initialize motors with id 107 an 106");

  //Check for connection
  if (serial_port.is_port_open())
  {
    CDxlConfig *config = new CDxlConfig();

    config->mDxlTypeStr = "3MXL";

    //First motor, left
    motors->addNewDynamixel(config->setID(107));
    //Second motor, right
    motors->addNewDynamixel(config->setID(106));
    //Set connection
    motors->setSerialPort(&serial_port);

    //Initialize and set mode to SPEED
    ROS_ASSERT(motors->init());
    ROS_ASSERT(motors->getDynamixel(0)->set3MxlMode(SPEED_MODE) == DXL_SUCCESS);
    ROS_ASSERT(motors->getDynamixel(1)->set3MxlMode(SPEED_MODE) == DXL_SUCCESS);

    //TODO check if needed
    //Make sure to set speed to 0
    drive_angularSpeed(0, 0);

    delete config;
  }
  else
  {
    ROS_ERROR("The serial port is not open");
    return false;
  }

  return true;
}

/**
 * Set the speed of the left and right motor.
 * It will clip the speed to the specified values from the
 * robot parameters.
 *
 * @param v_left left speed (m/s)
 * @param v_right right speed (m/s)
 */
void ThreeMxlBoard::drive_linearSpeed(double v_left, double v_right)
{
  ROS_DEBUG("Driving with leftSpeed: %f (m/s) , rightSpeed: %f (m/s)", v_left, v_right);

  //TODO look into problems with steering if speed is clipped
  v_left = clip(v_left, getParams().MAX_REV_LIMIT_LINSPEED, getParams().MAX_LIMIT_LINSPEED);
  v_right = clip(v_right, getParams().MAX_REV_LIMIT_LINSPEED, getParams().MAX_LIMIT_LINSPEED);

  motors->getDynamixel(0)->setLinearSpeed(v_left);
  motors->getDynamixel(1)->setLinearSpeed(v_right);
}

/**
 * Set the speed of the left and right motor.
 * It will clip the speed to the specified values from the
 * robot parameters.
 *
 * @param v_left left speed (m/s)
 * @param v_right right speed (m/s)
 */
void ThreeMxlBoard::drive_angularSpeed(double v_left, double v_right)
{
  ROS_DEBUG("Driving with leftSpeed: %f (rad/s) , rightSpeed: %f (rad/s)", v_left, v_right);

  v_left = clip(v_left, getParams().MAX_REV_LIMIT_ANGSPEED, getParams().MAX_LIMIT_ANGSPEED);
  v_right = clip(v_right, getParams().MAX_REV_LIMIT_ANGSPEED, getParams().MAX_LIMIT_ANGSPEED);

  motors->getDynamixel(0)->setSpeed(v_left);
  motors->getDynamixel(1)->setSpeed(v_right);
}

/**
 * Stop the robot from moving.
 */
void ThreeMxlBoard::stop()
{
  motors->getDynamixel(0)->set3MxlMode(STOP_MODE);
  motors->getDynamixel(1)->set3MxlMode(STOP_MODE);

  //TODO test stability
  motors->getDynamixel(0)->set3MxlMode(SPEED_MODE);
  motors->getDynamixel(1)->set3MxlMode(SPEED_MODE);
}

/**
 *
 * @return the position of the left motor (m)
 */
double ThreeMxlBoard::get_linearPos_left()
{
  motors->getDynamixel(0)->getLinearPos();
  double distance = motors->getDynamixel(0)->presentLinearPos();

  ROS_DEBUG("linear distance left is %f (m)", distance);
  return distance;
}
/**
 *
 * @return the position of the right motor (m)
 */
double ThreeMxlBoard::get_linearPos_right()
{
  motors->getDynamixel(1)->getLinearPos();
  double distance = motors->getDynamixel(1)->presentLinearPos();

  ROS_DEBUG("linear distance right is %f (m)", distance);
  return distance;
}

/**
 *
 * @return the position of the left motor (rad)
 */
double ThreeMxlBoard::get_angularPos_left()
{
  motors->getDynamixel(0)->getPos();
  double distance = motors->getDynamixel(0)->presentPos();

  ROS_DEBUG("linear distance right is %f (rad)", distance);
  return distance;
}

/**
 *
 * @return the position of the right motor (rad)
 */
double ThreeMxlBoard::get_angularPos_right()
{
  motors->getDynamixel(1)->getPos();
  double distance = motors->getDynamixel(1)->presentPos();

  ROS_DEBUG("linear distance right is %f (rad)", distance);
  return distance;
}

/**
 *
 * @return the stored parameters of this board(robot)
 */
Robot_Params ThreeMxlBoard::getParams()
{
  return robotParams;
}

/**
 * Translate status codes to human readable status codes
 *
 * @param status status of the motor
 * @return readable code of status
 */
std::string ThreeMxlBoard::translateStatus(int status)
{
  switch (status)
  {
    case STATUS_OK:
      return "No problems";
    case STATUS_EM_STOP:
      return "Emergency Button Pressed";
    case STATUS_OVERHEATING:
      return "Board or Motor is too hot";
    default:
      return motors->getDynamixel(0)->translateErrorCode(status);
  }
}

/**
 * @return the status of the left motor
 */
int ThreeMxlBoard::get_status_left()
{
  motors->getDynamixel(0)->getStatus();
  int status = motors->getDynamixel(0)->presentStatus();

  std::cout << "status left is " <<  translateStatus(status) << std::endl;
  return status;
}

/**
 *
 * @return the status of the right motor
 */
int ThreeMxlBoard::get_status_right()
{
  motors->getDynamixel(1)->getStatus();
  int status = motors->getDynamixel(1)->presentStatus();

  std::cout << "status right is " << translateStatus(status) << std::endl;
  return status;
}

}

