/*
 * driver.cpp
 * 
 *
 * Inspired from https://code.google.com/p/drh-robotics-ros/source/browse/trunk/ros/ardros/nodes/DeadReckoning.py
 */

//=======================================================
// predefined headers
#include <tf/transform_listener.h>
#include <cmath>
#include <ros/ros.h>

//=======================================================
// new defined headers
#include "movement/driver.h"

//Constants
#define PI 3.141592653589793238462643383279502884

namespace movement
{

/**
 * Execute rate
 */
const double loopSpeed = 30.0d; //(Hz)

//TODO add configurable topic name
/**
 * Create a Driver that publish velocity commands on topic "/cmd_vel"
 * and listens for transformation with source: "/base_link" and target "/odom"
 */
Driver::Driver()
{
  vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  TFlistener.waitForTransform("/odom", "/base_link", ros::Time(), ros::Duration(4.0d));
}

/**
 * (Default) Destructor
 */
Driver::~Driver()
{
}

/**
 * Move the robot platform straight forward. If the distance is made negative,
 * the robot will move backwards.
 *
 * It will use the transformation to determine the speed and distance
 * of the robot.
 *
 * @param distance distance to travel (e.g. in m for 3MxlBoard)
 * @param linearSpeed the speed of the robot
 */
void Driver::drive_straight(const double distance, const double linearSpeed)
{
  tf::StampedTransform start_transform;

  try
  {
    TFlistener.lookupTransform("/odom", "/base_link", ros::Time(0), start_transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  geometry_msgs::Twist cmdVel;
  double startX = start_transform.getOrigin()[0];
  double startY = start_transform.getOrigin()[1];

  //Check the direction
  if (distance >= 0)
  {
    cmdVel.linear.x = fabs(linearSpeed);
  }
  else
  {
    cmdVel.linear.x = -fabs(linearSpeed);
  }
  //Only straight forward
  cmdVel.angular.z = 0.0d;

  ros::Rate loop_rate(loopSpeed);
  while (ros::ok())
  {
    tf::StampedTransform current_transform;
    try
    {
      TFlistener.lookupTransform("/odom", "/base_link", ros::Time(0), current_transform);

      //Get the distance moved from the start
      double distance_x = current_transform.getOrigin()[0] - startX;
      double distance_y = current_transform.getOrigin()[1] - startY;

      //Add the x and y components together
      double distance_moved = sqrt(pow(distance_x, 2) + pow(distance_y, 2));

      std::cout << "moved: " << distance_moved << std::endl;

      //Check if goal distance is reached
      //TODO find cleaner way to check
      if (fabs(distance_moved) >= fabs(distance))
      {
        break;
      }
      else
      {
        vel_pub.publish(cmdVel);
      }
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    loop_rate.sleep();
  }

  //Stop the robot
  cmdVel.linear.x = 0;
  cmdVel.angular.z = 0;
  vel_pub.publish(cmdVel);
}

/**
 * Turn the robot counterclockwise around it's axis. If the distance is made negatve,
 * the robot will turn clockwise.
 *
 * It will use the transformation to determine the speed and distance
 * of the robot.
 *
 * @param distance turns to make (always in rad)
 * @param angularSpeed the speed of the robot
 */
void Driver::turn(const double angle, const double angularSpeed)
{
  tf::StampedTransform start_transform;
  try
  {
    TFlistener.lookupTransform("/odom", "/base_link", ros::Time(0), start_transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  double startAngle = 2 * atan2(start_transform.getRotation().getZ(), start_transform.getRotation().getW());
  double previousAngle = startAngle;
  //"full" rotations
  double angleOffset = 0.0;

  geometry_msgs::Twist cmdVel;
  //Check counter clockwise turn
  if (angle >= 0)
  {
    cmdVel.angular.z = fabs(angularSpeed);
  }
  else
  {
    cmdVel.angular.z = -fabs(angularSpeed);
  }
  //Only turning around base
  cmdVel.linear.x = 0.0;

  double angleTurned = 0;

  ros::Rate loop_rate(loopSpeed);
  while (ros::ok())
  {
    tf::StampedTransform current_transform;
    try
    {
      TFlistener.lookupTransform("/odom", "/base_link", ros::Time(0), current_transform);

      double currentAngle = 2 * atan2(current_transform.getRotation().getZ(), current_transform.getRotation().getW());

      //Check roll over
      if (currentAngle * previousAngle < 0 && fabs(currentAngle - previousAngle) > (PI / 2.0d))
      {
        if (currentAngle > previousAngle)
        {
          angleOffset -= 2.0d * PI;
        }
        else
        {
          angleOffset += 2.0d * PI;
        }
      }

      angleTurned = (currentAngle + angleOffset) - startAngle;
      previousAngle = currentAngle;

      std::cout << "Turned: " << angleTurned / PI << "PI" << std::endl;

      if (fabs(angleTurned) >= fabs(angle))
      {
        break;
      }
      else
      {
        // send the drive command
        vel_pub.publish(cmdVel);
      }
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    loop_rate.sleep();
  }
  //stop
  cmdVel.linear.x = 0.0;
  cmdVel.angular.z = 0.0;
  vel_pub.publish(cmdVel);
}

}

