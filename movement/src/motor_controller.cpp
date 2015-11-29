/*
 * motor_controller.cpp
 * 
 */

//=======================================================
// predefined headers
#include <iostream>
#include <geometry_msgs/Twist.h>

//=======================================================
// new defined headers
#include "movement/motor_controller.h"
#include "movement/robot_params.h"

namespace movement
{

Motor_controller::Motor_controller(Board::SharedPtr newBoard) :
    board(newBoard), nh("~"), des_v_left(0.0), des_v_right(0.0), last_cmdVel_update()
{
  cmdVel_sub = nh.subscribe("/turtle1/cmd_vel", 1, &Motor_controller::cmdVelCb, this);
  std::cout << "Subscribe to /turtle1/cmd_vel" << std::endl;
}

Motor_controller::~Motor_controller()
{
  std::cout << "Destructor motor controller called" << std::endl;
}

void Motor_controller::drive_linearSpeed(double v_left, double v_right)
{
  std::cout << "Motor Controller drive with left: " << v_left << "(m/s) right: " << v_right << "(m/s)" << std::endl;
  board->drive_linearSpeed(v_left, v_right);
}

void Motor_controller::drive_angularSpeed(double v_left, double v_right)
{
  std::cout << "Motor Controller drive with left: " << v_left << "(rad/s) right: " << v_right << "(rad/s)" << std::endl;
  board->drive_angularSpeed(v_left, v_right);
}

void Motor_controller::cmdVelCb(geometry_msgs::Twist::ConstPtr twist)
{
  std::cout << "Twist message received: " << std::endl;
  std::cout << "linear x: " << twist->linear.x << std::endl;
  std::cout << "linear y: " << twist->linear.y << std::endl;
  std::cout << "angular z: " << twist->angular.z << std::endl;

  double v = twist->linear.x;
  double theta = twist->angular.z;

  des_v_left = v - theta * ((double)AXLE_TRACK / (double)2.0d);          //(m/s)
  des_v_right = v + theta * ((double)AXLE_TRACK / (double)2.0d);         //(m/s)

  last_cmdVel_update = ros::Time::now();
}

void Motor_controller::spin()
{
  std::cout << "Spinning motor controller: " << std::endl;

  ros::Rate loop_rate(UPDATE_RATE);
  ros::Time current_time;

  while (ros::ok())
  {
    ros::spinOnce();
    current_time = ros::Time::now();

    if ((current_time - last_cmdVel_update).toSec() > CMD_VEL_TIMEOUT)
    {
      des_v_left = 0.0d;
      des_v_right = 0.0d;
    }

    drive_linearSpeed(des_v_left, des_v_right);

    loop_rate.sleep();
  }
}

}
