/*
 * sensorHandler.h
 * 
 */

//=======================================================
// include guard
#ifndef MOVEMENT_INCLUDE_MOVEMENT_SENSORHANDLER_H__INCLUDED
#define MOVEMENT_INCLUDE_MOVEMENT_SENSORHANDLER_H__INCLUDED

//=======================================================
// predefined headers
#include <movement/sensorReadings.h>
#include <ros/ros.h>

//=======================================================
// new defined headers
#include "movement/board/board.h"
#include "movement/sensorValues.h"

namespace movement
{

/**
 * The SensorHandler deals with the sensors of the robot and
 * information pulled from Board (e.g. motor status)
 *
 * It will read the sensors and (if needed) it will combine and average several
 * readings (e.g. left+right motor to one value)
 */
class SensorHandler
{

//Typedefs
//public:
//private:

//Public Functions
public:
  /**
   * Constructor
   * @param currentBoard board to pull information from
   */
  SensorHandler(Board::SharedPtr currentBoard);
  /**
   * (Default) Destructor
   */
  ~SensorHandler();

  //TODO filter startup junk readings
  /**
   * @return the readings of all sensors (and board)
   */
  sensorReadings getRead_All();
  /**
   *
   * @return the readings from only the board
   */
  sensorReadings getRead_Board();


//Private Functions
private:
  /**
   * (Default) Constructor, do not use
   */
  SensorHandler();
  /**
   * Read all sensors and board
   * @return the read values, stored inside SensorValues
   */
  SensorValues requestRead();
  /**
   * Calculate the difference(delta) of two values
   * @param oldVal the previous value
   * @param newVal the new value
   * @return the difference
   */
  double calculateDeltaValue(double oldVal, double newVal);
  /**
   * Calculate the average of two values
   * @param val1 value 1
   * @param val2 value 2
   * @return the average
   */
  double calculateAverage(double val1, double val2);
  /**
   * Use the values from the sensors(and board), stored inside SensorValues,
   * and apply calculations to it. The results will be stored inside the
   * sensorReadings.
   *
   * @param values values from the sensors(and board)
   * @return results of calculations, stored inside sensorReadings
   */
  sensorReadings createMessage(SensorValues values);

  //TODO for compatibility, make method more generic
  //TODO other message maybe?
  /**
   * (Callback) Read the values that the Arduino has published (Arduino reads in
   * all the sensor values)
   * @param readings the values pulled from ROS
   */
  void arduinoReadCb(sensorReadings::ConstPtr readings);

//Public Variables
//public:

//Private Variables
private:

  /**
   * Last read position of the left motor (rad)
   */
  double last_radPos_left;
  /**
   * Last read position of the right motor (rad)
   */
  double last_radPos_right;

  /**
   * Last read position of the left motor (m)
   */
  double last_linPos_left;
  /**
   * Last read position of the right motor (m)
   */
  double last_linPos_right;

private:
  //TODO make more generic
  /**
   * The readings pulled from the Arduino
   */
  SensorValues arduinoReadings;

  /**
   * Board to use
   */
  Board::SharedPtr board;

  //TODO make more generic
  /**
   * Subscriber that listens for arduino messages
   */
  ros::Subscriber arduino_sub;
  /**
   * ros interface
   */
  ros::NodeHandle nh;

};

} //namespace end

#endif /* MOVEMENT_INCLUDE_MOVEMENT_SENSORHANDLER_H__INCLUDED */
