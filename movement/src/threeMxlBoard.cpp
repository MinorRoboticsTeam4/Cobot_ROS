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
#include "movement/threeMxlBoard.h"
#include "movement/robot_params.h"

namespace movement
{

ThreeMxlBoard::ThreeMxlBoard() :
    motors(new CDxlGroup())
{
  std::cout << "Constructor ThreeMxlBoard called" << std::endl;
}

ThreeMxlBoard::~ThreeMxlBoard()
{
  std::cout << "Destructor ThreeMxlBoard called" << std::endl;

  //Make sure that robot isn't moving
  drive_angularSpeed(0,0);
  motors.~scoped_ptr();

  if (serial_port.is_port_open())
  {
    serial_port.port_close();
  }

}

bool ThreeMxlBoard::init_connection(std::string port_address, int baudrate)
{
  std::cout << "Opening connection on: " << port_address << " : " << baudrate << std::endl;

  bool port_open = serial_port.port_open(port_address, LxSerial::RS485_FTDI);
  serial_port.set_speed_int(baudrate);

  return port_open;
}

bool ThreeMxlBoard::init_motors()
{
  std::cout << "Initialize motors" << std::endl;
  if (serial_port.is_port_open())
  {
    CDxlConfig *config = new CDxlConfig();

    config->mDxlTypeStr = "3MXL";

    //First motor to add must be the Left Motor
    //Second motor to add must be the Right Motor
    motors->addNewDynamixel(config->setID(107));
    motors->addNewDynamixel(config->setID(106));

    motors->setSerialPort(&serial_port);

    ROS_ASSERT(motors->init());
    ROS_ASSERT(motors->getDynamixel(0)->set3MxlMode(SPEED_MODE) == DXL_SUCCESS);
    ROS_ASSERT(motors->getDynamixel(1)->set3MxlMode(SPEED_MODE) == DXL_SUCCESS);

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

void ThreeMxlBoard::drive_linearSpeed(double v_left, double v_right)
{
  std::cout << "Driving with leftSpeed: " << v_left << " (m/s) rightSpeed: " << v_right << "(m/s)" << std::endl;

  v_left = clip(v_left, MAX_LIMIT_LINSPEED, MAX_REV_LIMIT_LINSPEED);
  v_right = clip(v_right, MAX_LIMIT_LINSPEED, MAX_REV_LIMIT_LINSPEED);

  motors->getDynamixel(0)->setLinearSpeed(v_left);
  motors->getDynamixel(1)->setLinearSpeed(v_right);
}

void ThreeMxlBoard::drive_angularSpeed(double v_left, double v_right)
{
  std::cout << "Driving with leftSpeed: " << v_left << " (rad/s) rightSpeed: " << v_right << "(rad/s)" << std::endl;

  v_left = clip(v_left, MAX_LIMIT_ANGSPEED, MAX_REV_LIMIT_ANGSPEED);
  v_right = clip(v_right, MAX_LIMIT_ANGSPEED, MAX_REV_LIMIT_ANGSPEED);

  motors->getDynamixel(0)->setSpeed(v_left);
  motors->getDynamixel(1)->setSpeed(v_right);
}

int ThreeMxlBoard::get_status_left()
{
  motors->getDynamixel(0)->getStatus();
  return motors->getDynamixel(0)->presentStatus();
}

int ThreeMxlBoard::get_status_right()
{
  motors->getDynamixel(1)->getStatus();
  return motors->getDynamixel(1)->presentStatus();
}

}

