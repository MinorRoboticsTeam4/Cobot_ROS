/*
 * sensorHandler.cpp
 * 
 */

//=======================================================
// predefined headers

//=======================================================
// new defined headers
#include "cobot_movement/sensorHandler.h"
#include "cobot_movement/board/board.h"

double dist_left_junk;
double dist_right_junk;
double ang_left_junk;
double ang_right_junk;

namespace cobot_movement
{

/**
 * Create a new sensorHandler. It must have a specific board as parameter
 * to pull information from, like encoder information or status information.
 * @param currentBoard board to pull information from
 */
SensorHandler::SensorHandler(Board::SharedPtr currentBoard) :
    board(currentBoard), last_radPos_left(0), last_radPos_right(0), last_linPos_left(0), last_linPos_right(0), arduinoReadings()
{
  //First values that are read are non-zero but always different values
  dist_left_junk = board->get_linearPos_left();
  dist_right_junk = board->get_linearPos_right();

  ang_left_junk = board->get_angularPos_left();
  ang_right_junk = board->get_angularPos_right();

  arduino_sub = nh.subscribe("/arduino_topic", 10, &SensorHandler::arduinoRead, this);
  battery_pub = nh.advertise<cobot_movement::BatteryState>("/cobotBattery",10);


  pub_range1 = nh.advertise<sensor_msgs::Range>("/ultrasoon1", 1);
  pub_range2 = nh.advertise<sensor_msgs::Range>("/ultrasoon2", 1);
  pub_range3 = nh.advertise<sensor_msgs::Range>("/ultrasoon3", 1);
  pub_range4 = nh.advertise<sensor_msgs::Range>("/ultrasoon4", 1);
  pub_range5 = nh.advertise<sensor_msgs::Range>("/ultrasoon5", 1);

}

/**
 * (Default) destructor
 */
SensorHandler::~SensorHandler()
{

}

/**
 * Calculate the difference(delta) of two values
 * @param oldVal the previous value
 * @param newVal the new value
 * @return the difference
 */
double SensorHandler::calculateDeltaValue(double oldVal, double newVal)
{
  return newVal - oldVal;
}
/**
 * Calculate the average of two values
 * @param val1 value 1
 * @param val2 value 2
 * @return the average
 */
double SensorHandler::calculateAverage(double val1, double val2)
{
  return ((val1 + val2) / 2.0d);
}

/**
 * (Callback) Read the values that the Arduino has published (Arduino reads in
 * all the sensor values)
 * @param readings the values pulled from ROS
 */
//TODO look into method to "attach" new sensors to this handler
void SensorHandler::arduinoRead(SensorReadings::ConstPtr readings)
{
  //TODO Add a timestamp ??
  arduinoReadings.capacity = readings->capacity;
  arduinoReadings.charge = readings->charge;
  arduinoReadings.cups = readings->cups;
  arduinoReadings.consumptions = readings->consumptions;

  std::vector<double> ultrasoon_readings = readings->ultrasoonReadings;

  //Convert and publish ultrasoon readings to Range messages
  pub_range1.publish(convertToRangeMsg(ultrasoon_readings[0],readings->header.stamp));
  pub_range2.publish(convertToRangeMsg(ultrasoon_readings[1],readings->header.stamp));
  pub_range3.publish(convertToRangeMsg(ultrasoon_readings[2],readings->header.stamp));
  pub_range4.publish(convertToRangeMsg(ultrasoon_readings[3],readings->header.stamp));
  pub_range5.publish(convertToRangeMsg(ultrasoon_readings[4],readings->header.stamp));


  BatteryState battery;
  battery.header.stamp = ros::Time::now();
  battery.capacity = arduinoReadings.capacity;
  battery.charge = arduinoReadings.charge;

  battery_pub.publish(battery);
}

/**
 * Convert Ultrasonic range value to a Range message
 */
sensor_msgs::Range SensorHandler::convertToRangeMsg(double sensor_value,  ros::Time now)
{
	sensor_msgs::Range range_msg;

	range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
	range_msg.field_of_view = 0.1;  //stub value
    range_msg.min_range = 0.0;
	range_msg.max_range = 2.0;

	range_msg.range = sensor_value;

	range_msg.header.stamp = now;
	range_msg.header.frame_id = "/ultrasound";

	return range_msg;
}


/**
 * Read all sensors that are attached to this Handler (this includes also the board).
 * @return the read values, stored inside SensorValues
 */
SensorValues SensorHandler::requestRead()
{
  SensorValues values;

  //TODO better way to filter junk?
  //TODO average readings?
  values.linearPos_Left = board->get_linearPos_left() - dist_left_junk;
  values.linearPos_Right = board->get_linearPos_right() - dist_right_junk;
  values.angularPos_Left = board->get_angularPos_left() - ang_left_junk;
  values.angularPos_Right = board->get_angularPos_right() - ang_right_junk;

  values.capacity = arduinoReadings.capacity;
  values.charge = arduinoReadings.charge;

  values.cups = arduinoReadings.cups;
  values.consumptions = arduinoReadings.consumptions;

  return values;
}
/**
 * Use the values from the sensors(and board), stored inside SensorValues,
 * and apply calculations to it. The results will be stored inside the
 * sensorReadings.
 *
 * @param values values from the sensors(and board)
 * @return results of calculations, stored inside sensorReadings
 */
SensorReadings SensorHandler::createMessage(SensorValues values)
{
  SensorReadings readings;

  double delta_dist_left = calculateDeltaValue(last_linPos_left, values.linearPos_Left);
  double delta_dist_right = calculateDeltaValue(last_linPos_right, values.linearPos_Right);

  readings.distance = calculateAverage(delta_dist_left, delta_dist_right);
  readings.angle = (delta_dist_right - delta_dist_left) / board->getParams().AXLE_TRACK;

  //Needed to calculate the difference between reads to determine a delta distance and angle
  last_radPos_left = values.angularPos_Left;
  last_radPos_right = values.angularPos_Right;
  last_linPos_left = values.linearPos_Left;
  last_linPos_right = values.linearPos_Right;

  readings.capacity = values.capacity;
  readings.charge = values.charge;

  readings.header.stamp = ros::Time::now();

  return readings;
}

/**
 * @return the readings of all sensors (and board)
 */
SensorReadings SensorHandler::getRead_All()
{
  return createMessage(requestRead());
}
//TODO create a method that only requests and read information from the board only
/**
 *
 * @return the readings from only the board
 */
SensorReadings SensorHandler::getRead_Board()
{
  return createMessage(requestRead());
}

}

