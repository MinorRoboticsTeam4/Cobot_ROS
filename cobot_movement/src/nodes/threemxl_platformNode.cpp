/*
 * motor_controllerNode.cpp
 * 
 */

//=======================================================
// predefined headers
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

//=======================================================
// new defined headers
#include "cobot_movement/board/board.h"
#include "cobot_movement/board/threeMxlBoard.h"

#include "cobot_movement/motor_controller.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "threemxl_platform");
  ros::NodeHandle nh("~");

   cobot_movement::Board::SharedPtr board(new cobot_movement::ThreeMxlBoard);

   //Read serial port from launch file
   std::string connection = "/dev/usb0";
   nh.param("serial_port",connection,connection);
   board->init_connection(connection,921600);
   board->init_motors();
   cobot_movement::Motor_controller controller(board);

   controller.spin();

  //TODO move to board destructors??
   board->stop();

  return 0;
}
