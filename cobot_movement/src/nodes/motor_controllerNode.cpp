/*
 * motor_controllerNode.cpp
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
#include "cobot_movement/board/threeMxlBoard.h"

#include "cobot_movement/motor_controller.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_controller");

   cobot_movement::Board::SharedPtr board(new cobot_movement::ThreeMxlBoard);

   //TODO add option to read connection from default parameters
   board->init_connection("/dev/usb0",921600);
   board->init_motors();
   cobot_movement::Motor_controller controller(board);

   controller.spin();

  //TODO move to board destructors??
   board->stop();

  return 0;
}
