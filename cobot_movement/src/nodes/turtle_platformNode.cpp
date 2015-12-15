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
  ros::init(argc, argv, "turtle_platform");
  ros::NodeHandle nh("~");

  cobot_movement::Board::SharedPtr board(new cobot_movement::TurtleBoard);

  //Read serial port from launch file
  std::string connection = "/dev/usb0";
  nh.param("serial_port",connection,connection);
  board->init_connection(connection,921600);
  board->init_motors();

  cobot_movement::Motor_controller controller(board);
  //Run the controller
  controller.spin();

  return 0;
}
