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
#include "movement/board/board.h"
#include "movement/board/threeMxlBoard.h"

#include "movement/motor_controller.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_controller");

   movement::Board::SharedPtr board(new movement::ThreeMxlBoard);
   board->init_connection("/dev/usb0",921600);
   board->init_motors();
   movement::Motor_controller controller(board);

   controller.spin();

  //TODO move to board destructors??
   board->stop();

  return 0;
}
