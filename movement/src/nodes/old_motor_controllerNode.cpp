//TODO delete is deprecated
/*
 * motor_controllerNode.cpp
 *
 *  Created on: Nov 22, 2015
 *      Author: aclangerak
*/


//=======================================================
// predefined headers
#include <ros/ros.h>
//=======================================================
// new defined headers

#include "movement/old_motor_controller.h"

/** @brief Main function
 *
 * @param argc  An integer argument count of the command line arguments
 * @param argv  An argument vector of the command line arguments
 * @return  EXIT_SUCCESS if exit is success*/

int main(int argc, char** argv)
{
//Initialize ros node
  ros::init(argc, argv, "motor_controller");

//Initialize motor controller and open port
  old_movement::Motor_controller motor_controller;
  motor_controller.init_connection("/dev/ttyUSB0");
  motor_controller.init_Motors();

//Run ros node
  motor_controller.spin();

  return EXIT_SUCCESS;
}

