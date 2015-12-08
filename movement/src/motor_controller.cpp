/*
 * motor_controller.cpp
 * 
 */

//=======================================================
// predefined headers
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

//=======================================================
// new defined headers
#include "movement/motor_controller.h"

namespace movement
{

/**
 * Default constructor, setups the Nodehandler for publishing / listening.
 *
 * The default topic to listen to is "/cmd_vel"
 * This can be changed by adding the parameter "sub_topic" with
 * the desired topic name.
 *
 * The default topic for publishing odometry messages is "/odom"
 *
 * For odometry transformations, it uses as base frame = "/base_link"
 * and for odom frame = "/odom"
 *
 * @param newBoard board to be used for the controller
 */
Motor_controller::Motor_controller(Board::SharedPtr newBoard) :
    board(newBoard), nh("~"), des_v_left(0.0), des_v_right(0.0), last_cmdVel_update(), baseFrame("/base_link"), odomFrame(
        "/odom"), sensorHandler(newBoard),loopRate(30.0d)
{

  std::string sub_name;
  nh.param<std::string>("sub_topic", sub_name, sub_name);
  cmdVel_sub = nh.subscribe("/" + sub_name, 10, &Motor_controller::cmdVelCb, this);
  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
}

/**
 * Shutdown the connection with ROS
 */
Motor_controller::~Motor_controller()
{
  nh.shutdown();
}

/**
 * Set the speed for the robot, using the associated board.
 * @param v_left left speed of the motor (m/s)
 * @param v_right right speed of the motor (m/s)
 */
void Motor_controller::drive_linearSpeed(double v_left, double v_right)
{
  ROS_DEBUG("Motor Controller -> Board:  leftSpeed: %f (m/s) , rightSpeed: %f (m/s)", v_left, v_right);
  board->drive_linearSpeed(v_left, v_right);
}
/**
 * Set the speed for the robot, using the associated board.
 * @param v_left left speed of the motor (rad/s)
 * @param v_right right speed of the motor (rad/s)
 */
void Motor_controller::drive_angularSpeed(double v_left, double v_right)
{
  ROS_DEBUG("Motor Controller -> Board:  leftSpeed: %f (rad/s) , rightSpeed: %f (rad/s)", v_left, v_right);
  board->drive_angularSpeed(v_left, v_right);
}

/**
 * Callback function to listen for sources that publish speed messages
 * @param twist received message
 */
void Motor_controller::cmdVelCb(geometry_msgs::Twist::ConstPtr twist)
{
  double v = twist->linear.x;
  double theta = twist->angular.z;

  des_v_left = v - theta * ((double)board->getParams().AXLE_TRACK / (double)2.0d);          //(m/s)
  des_v_right = v + theta * ((double)board->getParams().AXLE_TRACK / (double)2.0d);         //(m/s)

  last_cmdVel_update = ros::Time::now();
}

/**
 * Calculate the odometry with the associated Transformation
 * @param prev_time the last time this calculation was made
 * @param odometry the odometry message to fill in
 * @return the Transformation ssociated with the odometry
 */
geometry_msgs::TransformStamped Motor_controller::calculateOdomTF(ros::Time prev_time, nav_msgs::Odometry::Ptr odometry)
{
  //################################
  // Calculate information needed //
  //################################

  //TODO decide the best place to put this read function or change the sensorhandler function to get a "buffered" version
  sensorReadings sensorReadings = sensorHandler.getRead_All();

  ros::Time now = sensorReadings.header.stamp;

  //Calculate delta's
  double delta_time = (now - prev_time).toSec();
  double delta_distance = sensorReadings.distance;
  double delta_angle = sensorReadings.angle;

  //Calculate (relative) distance traveled in x and y direction
  double x = cos(delta_angle) * delta_distance;
  double y = sin(delta_angle) * delta_distance;

  //Update current position x,y and theta
  double prev_angle = pos2d.theta;
  pos2d.x += cos(prev_angle) * x - sin(prev_angle) * y;
  pos2d.y += sin(prev_angle) * x + cos(prev_angle) * y;
  pos2d.theta += delta_angle;

  //Linear velocity
  geometry_msgs::Vector3 linearVel;
  linearVel.x = delta_distance / delta_time;
  linearVel.y = 0;
  linearVel.z = 0;

  //Angular velocity
  geometry_msgs::Vector3 angularVel;
  angularVel.x = 0;
  angularVel.y = 0;
  angularVel.z = delta_angle / delta_time;

  //##############################
  // Odom publisher information //
  //##############################

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pos2d.theta);

  //Header + Time stamp
  odometry->header.frame_id = odomFrame;
  odometry->child_frame_id = baseFrame;
  odometry->header.stamp = now;

  //"Actual" information
  odometry->pose.pose.position.x = pos2d.x;
  odometry->pose.pose.position.y = pos2d.y;
  odometry->pose.pose.position.z = 0.0d;

  odometry->pose.pose.orientation = odom_quat;

  odometry->twist.twist.linear = linearVel;
  odometry->twist.twist.angular = angularVel;

  //###################################
  // Odom TF broadcaster information //
  //###################################

  geometry_msgs::TransformStamped transform;

  //Header + Time stamp
  transform.header.frame_id = odomFrame;
  transform.child_frame_id = baseFrame;
  transform.header.stamp = prev_time;

  //"Actual" information
  transform.transform.translation.x = pos2d.x;
  transform.transform.translation.y = pos2d.y;
  transform.transform.translation.z = 0.0d;

  transform.transform.rotation = odom_quat;

  return transform;
}

/**
 * @param transform the transformation to be published
 */
void Motor_controller::publish_OdomTF(geometry_msgs::TransformStamped transform)
{
  //Ros TF wants this to be static for some reason
  static tf::TransformBroadcaster br;
  br.sendTransform(transform);
}

/**
 * Run this node
 */
void Motor_controller::spin()
{
  ros::Rate loop_rate(loopRate);
  ros::Time current_time = ros::Time::now();
  ros::Time last_time_sensor_read = ros::Time::now();

  while (ros::ok())
  {
    ros::spinOnce();
    //Update time
    current_time = ros::Time::now();

    /*
     * Calculate
     */
    nav_msgs::Odometry::Ptr odometry(new nav_msgs::Odometry);
    geometry_msgs::TransformStamped trans = calculateOdomTF(last_time_sensor_read, odometry);
    last_time_sensor_read = ros::Time::now();

    if ((current_time - last_cmdVel_update).toSec() > board->getParams().CMD_VEL_TIMEOUT)
    {
      des_v_left = 0.0d;
      des_v_right = 0.0d;
    }

    /*
     * Publish
     */
    publish_OdomTF(trans);
    odom_pub.publish(odometry);

    /*
     * Drive
     */

    drive_linearSpeed(des_v_left, des_v_right);


    loop_rate.sleep();
  }
}

}
