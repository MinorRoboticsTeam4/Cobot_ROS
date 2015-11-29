/*
 * motor_controllerNode.cpp
 * 
 */

//=======================================================
// predefined headers
#include <boost/shared_ptr.hpp>

//=======================================================
// new defined headers
#include "movement/board.h"
#include "movement/dummyBoard.h"
#include "movement/threeMxlBoard.h"
#include "movement/motor_controller.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_controller");


  movement::Board::SharedPtr board(new movement::DummyBoard);
  board->init_connection("/dev/usb0",921600);
  board->init_motors();
  board->get_status_left();

  movement::Motor_controller controller(board);
  controller.spin();

  return 0;
}
