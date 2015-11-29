/*
 * dummyBoard.cpp
 * 
 */

//=======================================================
// predefined headers
#include <iostream>

//=======================================================
// new defined headers
#include "movement/dummyBoard.h"

namespace movement
{

DummyBoard::DummyBoard()
{
  std::cout << "Constructor DummyBoard called" << std::endl;
}

DummyBoard::~DummyBoard()
{
  std::cout << "Destructor DummyBoard called" << std::endl;
}

bool DummyBoard::init_connection(std::string port_address, int baudrate)
{
  std::cout << "Opening connection on: " << port_address << " : " << baudrate << std::endl;
  return true;
}

bool DummyBoard::init_motors()
{
  std::cout << "Initialize motors" << std::endl;
  return true;
}

void DummyBoard::drive_linearSpeed(double v_left, double v_right)
{
  std::cout << "Driving with leftSpeed: " << v_left << " (m/s) rightSpeed: " << v_right << "(m/s)" << std::endl;
}

void DummyBoard::drive_angularSpeed(double v_left, double v_right)
{
  std::cout << "Driving with leftSpeed: " << v_left << " (rad/s) rightSpeed: " << v_right << "(rad/s)" << std::endl;
}

int DummyBoard::get_status_left()
{
  std::cout << "status left is 0" << std::endl;
  return 0;
}

int DummyBoard::get_status_right()
{
  std::cout << "status right is 0" << std::endl;
  return 0;
}



}
