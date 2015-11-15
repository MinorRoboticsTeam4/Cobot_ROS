/*
 * motor_controller.cpp
 *
 *  Created on: Nov 12, 2015
 *      Author: aclangerak
 */

//=======================================================
// predefined headers
#include <ros/ros.h>

//=======================================================
// new defined headers
namespace movement
{

Motor_controller::Motor_controller() :
    serial_port()
{
cmdVel_sub = nh.subscribe("cmd_vel",);

}

Motor_controller::~Motor_controller()
{

}

bool Motor_controller::init_connection(std::string port_address, int baudrate)
{
ROS_INFO("Opening direction connection to: %s ; baudrate: %d", port_address.c_str(), baudrate);

bool port_open = serial_port->port_open(port_address, LxSerial::RS485_FTDI);
serial_port->set_speed_int(baudrate);

return port_open;
}

bool Motor_controller::shutdown_connection()
{

bool port_close = serial_port->port_close();

return port_close;
}

void Motor_controller::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &twist)
{
double x = twist->linear.x;            //(m/s)
double omega = twist->angular.z;       //(rad/s)
double speedLeft, speedRight;

//Stop
if (x == 0 && omega == 0)
{

  //Set speed of left = 0, right = 0
  return;
}

//Turn in place
if (x == 0)
{
  /*TODO wheel_track is distance between center of the wheels
   * and explanation of formula(s)
   */
  speedRight = omega * wheel_track / 2.0;
  //Mirror other wheel
  speedLeft = -speedRight;
}
//Forward or backward
else if (omega == 0)
{
  speedRight = speedLeft = x;
}
//Rotation about a point (not center of robot)
else
{
  speedLeft = x - omega * wheel_track / 2.0;
  speedRight = x + omega * wheel_track / 2.0;
}

//TODO set speed to motors

}

}

/** @brief Main function
 *
 * @param argc  An integer argument count of the command line arguments
 * @param argv  An argument vector of the command line arguments
 * @return  EXIT_SUCCESS if exit is success
 */
int main(int argc, char** argv)
{
//Initialize ros node
ros::init(argc, argv, "motor_controller");

//Initialize motor controller
movement::Motor_controller motor_controller;

motor_controller.init_connection("USB ID");

//Run ros node
while (ros::ok())
{
  ros::spin();
}

//Ros node is terminated
motor_controller.shutdown_connection();
delete motor_controller;

return EXIT_SUCCESS;
}

