/*

 * motor_controller.cpp
 *
 * Code is based on the turtlebot for the IRobot
 */

//=======================================================
// predefined headers
#include "../include/movement/old_motor_controller.h"

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

#include <threemxl/platform/hardware/dynamixel/CDxlConfig.h>

//=======================================================
// new defined headers

namespace old_movement
{

/* * Clip the value between min<=x<=max
 *
 * @param n value to be clipped
 * @param lower lower bound
 * @param upper upper bound
 * @return clipped value*/

template<typename T>
  T clip(const T& n, const T& lower, const T& upper)
  {
    return std::max(lower, std::min(n, upper));
  }

/*
 * Default constructor, setups the Nodehandler for publishing / listening.
 *
 * The default topic to listen to is "/cmd_vel"
 * This can be changed by adding the parameter "sub_topic" with
 * the desired topic name.
 *
 * Additionally, it defines the base frame("/base_link") and the odometry frame("/odom") needed for tf
 */

Motor_controller::Motor_controller() :
    serial_port(), sub_name("cmd_vel"), nh("~"), baseFrame("/base_link"), odomFrame("/odom"), last_cmd_vel_time(0), last_dist_left(
        0), last_dist_right(0), motors(), des_v_left(0), des_v_right(0), last_angle_left(0), last_angle_right(0)
{

  nh.param<std::string>("sub_topic", sub_name, sub_name);
  cmdVel_sub = nh.subscribe("/" + sub_name, 10, &Motor_controller::cmdVelCallback, this);

  ROS_INFO("Listening on topic: %s", sub_name.c_str());

  //Publish on odom topic
  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 2);

}

/*
 * The destructor closes the serial port if it is open.
 *
 * It also deletes the left and right motor interfaces.
 *
 * Closes down the Nodehandler*/

Motor_controller::~Motor_controller()
{
  if (serial_port.is_port_open())
  {
    serial_port.port_close();
  }

  //Remove motors

  nh.shutdown();

}

/* * Open a new connection to the 3Mxel board.
 * You can find the port_address by typing  "lsusb" in the Linux terminal.
 *
 * @param port_address  USB ID of the board
 * @param baudrate  the baud rate(default = 921600)
 * @return true if connection was successful*/

bool Motor_controller::init_connection(std::string port_address, int baudrate)
{
  ROS_INFO("Opening connection to: %s ; baudrate: %d", port_address.c_str(), baudrate);

  bool port_open = serial_port.port_open(port_address, LxSerial::RS485_FTDI);
  serial_port.set_speed_int(baudrate);

  return port_open;
}

/* * Initialize the motors with serial port, id, set3MxlMode
 *
 * <li> left motor id: (106/107) </li>
 * <li> right motor id: (106/107) </li>
 *
 *  3MxlMode: SPEED_MODE
 *
 * @return true if succesfull*/

bool Motor_controller::init_Motors()
{
  if (serial_port.is_port_open())
  {
    CDxlConfig *config = new CDxlConfig();

    config->mDxlTypeStr = "3MXL";
    motors = new CDxlGroup();
    motors->addNewDynamixel(config->setID(107));
    motors->addNewDynamixel(config->setID(106));

    motors->setSerialPort(&serial_port);

    ROS_ASSERT(motors->init());
    ROS_ASSERT(motors->getDynamixel(0)->set3MxlMode(SPEED_MODE) == DXL_SUCCESS);
    ROS_ASSERT(motors->getDynamixel(1)->set3MxlMode(SPEED_MODE) == DXL_SUCCESS);

    motors->getDynamixel(0)->setLinearSpeed(0);
    motors->getDynamixel(1)->setLinearSpeed(0);

    delete config;

  }
  else
  {
    ROS_ERROR("The serial port is not open");
    return false;
  }

  return true;
}

/* * (Callback) Function to convert Twist->motor velocity commands
 *
 * It only uses the linear.x and angular.z information, because
 * this robot can only move forwar/backward and can do rotations.
 *
 * @param twist  Twist message to respond to*/

void Motor_controller::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &twist)
{
  double v = twist->linear.x;                           //(m/s)
  double th = -1 * twist->angular.z;                         //(rad/s)

  //TODO Either flip angular of joystick or flip values here
  des_v_left = v - th * ((double)0.600d / (double)2.0d);          //(m/s)
  des_v_right = v + th * ((double)0.600d / (double)2.0d);         //(m/s)

  //Reset time for timeout
  last_cmd_vel_time = ros::Time::now();

//  drive_linearSpeed(v_left, v_right);
//  drive_angularSpeed(v_left / (0.5*WHEEL_DIAMETER) , v_right / (0.5*WHEEL_DIAMETER));

}

/* * Give the command to drive this robot.
 *
 * It checks if the bumper(outer shell) is touched
 * It checks if the emergency button is checked.
 *
 * @param v_left speed of left motor (m/s)
 * @param v_right speed of right motor (m/s)*/

void Motor_controller::drive_linearSpeed(double v_left, double v_right)
{
  //TODO Check bumper is hit
  motors->getDynamixel(0)->getStatus();
  motors->getDynamixel(1)->getStatus();

  motors->getDynamixel(0)->translateErrorCode(motors->getLastError());

  if (motors->getDynamixel(0)->presentStatus() == M3XL_STATUS_EM_STOP_ERROR)
  {
    ROS_WARN("Left says: Emergency Button pressed, can't set speed");
  }

  else if (motors->getDynamixel(1)->presentStatus() == M3XL_STATUS_EM_STOP_ERROR)
  {
    ROS_WARN("Right says: Emergency Button pressed, can't set speed");
  }
  else
  {
    //TODO is clip needed?
    v_left = clip(v_left, -1.0, 1.0);
    v_right = clip(v_right, -1.0, 1.0);

    ROS_INFO("Setting speed Left: %f m/s, Right: %f m/s", v_left, v_right);

    motors->getDynamixel(0)->setLinearSpeed(v_left);
    motors->getDynamixel(1)->setLinearSpeed(v_right);
  }

}

/* * Give the command to drive this robot.
 *
 * It checks if the bumper(outer shell) is touched
 * It checks if the emergency button is checked.
 *
 * @param v_left speed of left motor (rad/s)
 * @param v_right speed of right motor (rad/s)*/

void Motor_controller::drive_angularSpeed(double v_left, double v_right)
{
  //TODO Check bumper is hit

  //TODO Test if stop status works
  motors->getDynamixel(0)->getStatus();
  motors->getDynamixel(1)->getStatus();

  motors->getDynamixel(0)->translateErrorCode(motors->getLastError());

  if (motors->getDynamixel(0)->presentStatus() == M3XL_STATUS_EM_STOP_ERROR)
  {
    ROS_WARN("Left says: Emergency Button pressed, can't set speed");
  }
  else if (motors->getDynamixel(1)->presentStatus() == M3XL_STATUS_EM_STOP_ERROR)
  {
    ROS_WARN("Right says: Emergency Button pressed, can't set speed");
  }
  else
  {
    //Maybe check if speed is clamped?
    v_left = clip(v_left, -5.0, 5.0);
    v_right = clip(v_right, -5.0, 5.0);

    ROS_INFO("Setting speed for robot Left: %f rad/s, Right: %f rad/s", v_left, v_right);

    motors->getDynamixel(0)->setSpeed(v_left);
    motors->getDynamixel(1)->setSpeed(v_right);

  }
}

/* * Calculates Odometry and TF information for ROS.
 *
 * It also publish both the odometry and tf transform.*/

void Motor_controller::updateOdom()
{
  //################################
  // Calculate information needed //
  //################################

  ros::Time now = ros::Time::now();

  //Time between last measurement and now
  //TODO Check if time is correct
  double dt = (now - last_odom_time).toSec();
  last_odom_time = now;

  //TODO check if delta distance travelled of wheels needed

  motors->getDynamixel(0)->getLinearPos();
  double dist_left =   motors->getDynamixel(0)->presentLinearPos();
  last_dist_left = dist_left;
  double dt_dist_left = dist_left - last_dist_left;

  motors->getDynamixel(1)->getLinearPos();
  double dist_right =   motors->getDynamixel(1)->presentLinearPos();
  last_dist_right = dist_right;
  double dt_dist_right = dist_right - last_dist_right;

  double dist_avg = (dt_dist_left - dt_dist_right) / 2.0d;

/*
  motors->getDynamixel(0)->getLinearPos();               //(m/s)
  double dist_left = motors->getDynamixel(0)->presentLinearPos();

  motors->getDynamixel(1)->getLinearPos();                               //(m/s)
  double dist_right = motors->getDynamixel(1)->presentLinearPos();

  //Average linear distance of both wheels
  //TODO Maybe use delta version instead
  double dist_avg = (dist_left + dist_right) / 2.0d;    //(m/s)
*/

  //Angle rotated
  //TODO Seems there are 2 options, test them both, check also if delta needed

  motors->getDynamixel(0)->getPos();
  motors->getDynamixel(1)->getPos();

  double ang_left = motors->getDynamixel(0)->presentPos();
  double ang_right = motors->getDynamixel(1)->presentPos();

//  double dt_angle = (ang_left + ang_right) / 2.0d;

  double dt_angle_left = (ang_left - last_angle_left);
  double prev_ang_left = ang_left;

  double dt_angle_right = (ang_right - last_angle_right);
  double prev_ang_right = ang_right;

  double dt_angle = (dt_angle_left - dt_angle_right) / 2.0d;

  //TODO Maybe use delta version instead
//  double dt_angle = (dist_right - dist_left) / AXLE_TRACK;

  //Delta distances x and y direction(how far did the robot move)
  double dx = cos(dt_angle) * dist_avg;
  double dy = -sin(dt_angle) * dist_avg;

  double last_angle = pos2d.theta;
  pos2d.x += cos(last_angle) * dx - sin(last_angle) * dy;
  pos2d.y += sin(last_angle) * dx + cos(last_angle) * dy;
  pos2d.theta += dt_angle;

  //Linear velocity
  //TODO maybe use delta version instead
  geometry_msgs::Vector3 linearVel;
  linearVel.x = dist_avg / dt;
  linearVel.y = 0;
  linearVel.z = 0;

  //Angular velocity
  geometry_msgs::Vector3 angularVel;
  linearVel.x = 0;
  linearVel.y = 0;
  linearVel.z = dt_angle / dt;

  //##############################
  // Odom publisher information //
  //##############################

  nav_msgs::Odometry odom;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pos2d.theta);
  //Header + Time stamp
  odom.header.frame_id = odomFrame;
  odom.child_frame_id = baseFrame;
  odom.header.stamp = now;

  //"Actual" information
  odom.pose.pose.position.x = pos2d.x;
  odom.pose.pose.position.y = pos2d.y;
  odom.pose.pose.position.z = 0.0d;

  odom.pose.pose.orientation = odom_quat;

  odom.twist.twist.linear = linearVel;
  odom.twist.twist.angular = angularVel;

  odom_pub.publish(odom);

  //###################################
  // Odom TF broadcaster information //
  //###################################
  tf::TransformBroadcaster odom_tf;

  geometry_msgs::TransformStamped transform;
  //Header + Time stamp  (same data as above)
  transform.header.frame_id = odomFrame;
  transform.child_frame_id = baseFrame;
  transform.header.stamp = now;

  //"Actual" information
  transform.transform.translation.x = pos2d.x;
  transform.transform.translation.y = pos2d.y;
  transform.transform.translation.z = 0.0d;
  transform.transform.rotation = odom_quat;

  std::cout << transform << std::endl;

  odom_tf.sendTransform(transform);
}

/* * Run this node*/

void Motor_controller::spin()
{

  ros::Rate loop_rate(5.0);

  while (ros::ok())
  {
    //Do additional work here

    ros::Time current_time = ros::Time::now();

    //Timeout has occurred, stop
    //TODO needs verification
    if (current_time.toSec() - last_cmd_vel_time.toSec() > 2.0d)
    {
      drive_linearSpeed(0, 0);
      ROS_INFO("No cmd_vel received, Timeout");
    }
    else
    {
      drive_linearSpeed(des_v_left, des_v_right);
    }

    updateOdom();
    ros::spinOnce();
    loop_rate.sleep();
  }

  //Node is stopped, stop also the motors
  motors->getDynamixel(0)->set3MxlMode(STOP_MODE);
  motors->getDynamixel(1)->set3MxlMode(STOP_MODE);
}

}

