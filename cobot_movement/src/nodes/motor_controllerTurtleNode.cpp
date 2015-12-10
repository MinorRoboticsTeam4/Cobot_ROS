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
#include "cobot_movement/board/board.h"
#include "cobot_movement/board/turtleBoard.h"

#include "cobot_movement/motor_controller.h"

int main(int argc, char** argv)
{
  //Init Node
  ros::init(argc, argv, "turtle_controller");
  cobot_movement::Board::SharedPtr board(new cobot_movement::TurtleBoard);

  //Init Board
  board->init_connection("/dev/usb0", 921600);
  board->init_motors();

  cobot_movement::Motor_controller controller(board);
  //Run the controller
  controller.spin();

  return 0;
}
