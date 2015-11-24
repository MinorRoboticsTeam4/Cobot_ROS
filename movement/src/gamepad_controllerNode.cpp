/*
 * gamepad_controllerNode.cpp
 *
 */

//=======================================================
// predefined headers
#include <ros/ros.h>

//=======================================================
// new defined headers
#include <movement/gamepad_controller.h>


/** @brief Main function
 * @param argc  An integer argument count of the command line arguments
 * @param argv  An argument vector of the command line arguments
 * @return  EXIT_SUCCESS if exit is success
 */
int main(int argc, char** argv)
{
  //Initialize ros node
  ros::init(argc, argv, "gamepad_controller");

  //Initialize gamepad_controller class
  movement::GamePad_controller gamepad_controller;
  gamepad_controller.spin();

  return EXIT_SUCCESS;
}


