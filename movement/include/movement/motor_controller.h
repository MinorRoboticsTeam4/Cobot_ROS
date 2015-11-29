/*
 * motor_controller.h
 * 
 */

//=======================================================
// include guard
#ifndef MOVEMENT_INCLUDE_MOVEMENT_MOTOR_CONTROLLER_H__INCLUDED
#define MOVEMENT_INCLUDE_MOVEMENT_MOTOR_CONTROLLER_H__INCLUDED

//=======================================================
// predefined headers
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

//=======================================================
// new defined headers
#include "movement/board.h"


namespace movement
{
class Motor_controller
{

 //Public Functions
public:
  Motor_controller(Board::SharedPtr newBoard);
  ~Motor_controller();
  void drive_linearSpeed(double v_left, double v_right);
  void drive_angularSpeed(double v_left, double v_right);
  void spin();

  //Private Functions
private:
  Motor_controller();
  void cmdVelCb(geometry_msgs::Twist::ConstPtr twist);

  //Private Variables
private:
  Board::SharedPtr board;
  double des_v_left;
  double des_v_right;
  ros::NodeHandle nh;
  ros::Subscriber cmdVel_sub;
  ros::Time last_cmdVel_update;
};






}





#endif /* MOVEMENT_INCLUDE_MOVEMENT_MOTOR_CONTROLLER_H__INCLUDED */
