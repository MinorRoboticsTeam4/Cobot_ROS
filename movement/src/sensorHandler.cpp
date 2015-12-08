/*
 * sensorHandler.cpp
 * 
 */

//=======================================================
// predefined headers
//=======================================================
// new defined headers
#include "movement/sensorHandler.h"
#include "movement/board/board.h"
#include "movement/sensorReadings.h"

double dist_left_junk;
double dist_right_junk;

double ang_left_junk;
double ang_right_junk;

namespace movement
{

SensorHandler::SensorHandler(Board::SharedPtr currentBoard) :
    board(currentBoard), last_radPos_left(0), last_radPos_right(0), last_linPos_left(0), last_linPos_right(0), arduinoReadings()
{

  dist_left_junk = board->get_linearPos_left();
  dist_right_junk = board->get_linearPos_right();

  ang_left_junk = board->get_angularPos_left();
  ang_right_junk = board->get_angularPos_right();
}

SensorHandler::~SensorHandler()
{

}

double SensorHandler::calculateDeltaValue(double oldVal, double newVal)
{
  //TODO check if better function is needed
  return newVal - oldVal;
}

double SensorHandler::calculateAverage(double val1, double val2)
{
  return ((val1 + val2) / 2.0d);
}

void SensorHandler::arduinoReadCb(sensorReadings::ConstPtr readings)
{
  //TODO Add a timestamp ??
  arduinoReadings.capacity = readings->capacity;
  arduinoReadings.charge = readings->charge;

}

SensorValues SensorHandler::requestRead()
{
  SensorValues values;

  //TODO better way to filter junk?
  values.linearPos_Left = board->get_linearPos_left() - dist_left_junk;
  values.linearPos_Right = board->get_linearPos_right()- dist_right_junk;
  values.angularPos_Left = board->get_angularPos_left() - ang_left_junk;
  values.angularPos_Right = board->get_angularPos_right() - ang_right_junk;;

  values.capacity = arduinoReadings.capacity;
  values.charge = arduinoReadings.charge;

  return values;
}

sensorReadings SensorHandler::createMessage(SensorValues values)
{
  sensorReadings readings;

  readings.header.stamp = ros::Time::now();

  double delta_dist_left = calculateDeltaValue(last_linPos_left, values.linearPos_Left);
  double delta_dist_right = calculateDeltaValue(last_linPos_right, values.linearPos_Right);

  readings.distance = calculateAverage(delta_dist_left,delta_dist_right);
  readings.angle = (delta_dist_right - delta_dist_left) / board->getParams().AXLE_TRACK;

  last_radPos_left = values.angularPos_Left;
  last_radPos_right = values.angularPos_Right;
  last_linPos_left = values.linearPos_Left;
  last_linPos_right = values.linearPos_Right;

  readings.capacity = values.capacity;
  readings.charge = values.charge;

  return readings;
}

sensorReadings SensorHandler::getRead_All()
{
  return createMessage(requestRead());
}

sensorReadings SensorHandler::getRead_Board()
{
  return createMessage(requestRead());
}

}

