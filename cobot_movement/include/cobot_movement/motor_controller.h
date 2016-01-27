/*
 * motor_controller.h
 * 
 */

//=======================================================
// include guard
#ifndef MOVEMENT_INCLUDE_MOVEMENT_MOTOR_CONTROLLER_H_INCLUDED
#define MOVEMENT_INCLUDE_MOVEMENT_MOTOR_CONTROLLER_H_INCLUDED

//=======================================================
// predefined headers
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

//=======================================================
// new defined headers
#include "cobot_movement/board/board.h"
#include "cobot_movement/sensorHandler.h"

namespace cobot_movement
{
/**
 * The motor controller controls the board of the motor.
 * It is responsible for the following:
 * <li> Listening for speed commands </li>
 * <li>Settings the speed of the board</li>
 * <li>Calculating the odometry</li>
 * <li>Calculating the corresponding transformations of odometry</li>
 */
class Motor_controller
{

  //Public Functions
public:
  /**
   * Constructor
   * @param newBoard board to be used for the controller
   */
  Motor_controller(Board::SharedPtr newBoard);
  /**
   * (Default) Destructor
   */
  ~Motor_controller();

  /**
   * Set the speed for the robot, using the associated board.
   * @param v_left left speed of the motor (m/s)
   * @param v_right right speed of the motor (m/s)
   */
  void drive_linearSpeed(double v_left, double v_right);
  /**
   * Set the speed for the robot, using the associated board.
   * @param v_left left speed of the motor (rad/s)
   * @param v_right right speed of the motor (rad/s)
   */
  void drive_angularSpeed(double v_left, double v_right);

  /**
   * Run the motor controller
   */
  void spin();

  //Private Functions
private:
  /**
   * (Default) Constructor, do not use
   */
  Motor_controller();
  /**
   * Callback function to listen for sources that publish speed messages
   * @param twist received message
   */
  void cmdVelCb(geometry_msgs::Twist::ConstPtr twist);
  /**
   * Calculate the odometry with the associated Transformation
   * @param prev_time the last time this calculation was made
   * @param odometry the odometry message to fill in
   * @return the Transformation ssociated with the odometry
   */
  geometry_msgs::TransformStamped calculateOdomTF(ros::Time prev_time, nav_msgs::Odometry::Ptr odometry);
  /**
   * @param transform the transformation to be published
   */
  void publish_OdomTF(geometry_msgs::TransformStamped transform);

  //Private Variables
private:
  /**
   * (Interface) board
   */
  Board::SharedPtr board;
  /**
   * The "label frame" of the robot platform
   */
  std::string baseFrame;
  /**
   * The "label frame" for the odometry axis
   */
  std::string odomFrame;
  /**
   * The 2D position of the robot
   */
  geometry_msgs::Pose2D pos2d;

  /**
   * The desired velocity of the left motor
   * (m/s) or (rad/s) , use with proper drive function
   */
  double des_v_left;
  /**
   * The desired velocity of the right motor
   * (m/s) or (rad/s) , use with proper drive function
   */
  double des_v_right;

  /**
   * Rate of the ros loop
   */
  double loopRate;

  /**
   * "Controller" that handles the input of different sensors e.g
   * 3mxl motor statuses, arduino etc..
   */
  SensorHandler sensorHandler;

  /**
   * Ros (interface)
   */
  ros::NodeHandle nh;
  /**
   * Subscriber that listens for velocity commands
   */
  ros::Subscriber cmdVel_sub;
  /**
   * Publisher that advertises the odometry
   */
  ros::Publisher odom_pub;
  /**
   * Last time that the controller received a velocity message.
   */
  ros::Time last_cmdVel_update;
};

}

#endif /* MOVEMENT_INCLUDE_MOVEMENT_MOTOR_CONTROLLER_H_INCLUDED */
