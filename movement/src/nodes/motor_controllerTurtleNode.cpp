/*
 * motor_controllerTurtleNode.cpp
 * 
 */

//=======================================================
// predefined headers
#include <boost/shared_ptr.hpp>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

//=======================================================
// new defined headers
#include "movement/board/board.h"
#include "movement/board/dummyBoard.h"
#include "movement/board/threeMxlBoard.h"
#include "movement/board/turtleBoard.h"

#include "movement/motor_controller.h"

int main(int argc, char** argv)
{
  //Init Node
  ros::init(argc, argv, "turtle_controller");
  movement::Board::SharedPtr board(new movement::TurtleBoard);

  //Init Board
  board->init_connection("/dev/usb0", 921600);
  board->init_motors();

  movement::Motor_controller controller(board);
  //Run the controller
  controller.spin();

  return 0;
}
