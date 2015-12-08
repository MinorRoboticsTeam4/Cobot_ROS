/*
 * motor_params.h
 *
 */

//=======================================================
// include guard
#ifndef MOVEMENT_INCLUDE_MOVEMENT_ROBOT_PARAMS_H_INCLUDED
#define MOVEMENT_INCLUDE_MOVEMENT_ROBOT_PARAMS_H_INCLUDED

//=======================================================
// predefined headers

//=======================================================
// new defined headers

namespace movement
{

struct Robot_Params
{
  //Robot parameters, pulled of Technical Manual
  const double AXLE_TRACK;                                      //(m)
  const double WHEEL_DIAMETER;                                  //(m)
  const double CMD_VEL_TIMEOUT;                                 //(s)

  //Based on that robot moves at max 1 meter per 1 second
  //TODO test values
  const double MAX_LIMIT_LINSPEED;                //(m/s)
  const double MAX_REV_LIMIT_LINSPEED;            //(m/s)
  const double MAX_LIMIT_ANGSPEED;                //(rad/s)
  const double MAX_REV_LIMIT_ANGSPEED;            //(rad/s)

  /**
   * Store the Robot Parameters as constants
   *
   * The maximal speed in (rad/s) is calculated as
   * maxLinSpeed / (0.5 * WHEEL_DIAMETER)
   *
   * @param axleTrack distance between the centers of the wheels (m)
   * @param wheelDiameter diameter of the wheel (m)
   * @param cmdVelTimeout the maximal time between velocity commands before timeout
   * @param maxLimitLinSpeed maximal speed (m/s)
   * @param maxRevLimitLinSpeed maximal reverse speed (m/s) , (number is positive
   */
  Robot_Params(double axleTrack, double wheelDiameter, double cmdVelTimeout, double maxLimitLinSpeed,
               double maxRevLimitLinSpeed) :
      AXLE_TRACK(axleTrack), WHEEL_DIAMETER(wheelDiameter), CMD_VEL_TIMEOUT(cmdVelTimeout), MAX_LIMIT_LINSPEED(
          maxLimitLinSpeed), MAX_REV_LIMIT_LINSPEED(-maxRevLimitLinSpeed), MAX_LIMIT_ANGSPEED(
          maxLimitLinSpeed / (0.5 * WHEEL_DIAMETER)), MAX_REV_LIMIT_ANGSPEED(
          -maxRevLimitLinSpeed / (0.5 * WHEEL_DIAMETER))
  {
    if (maxRevLimitLinSpeed < 0)
    {
      //TODO c++ errors
      std::cout << "maxRevLimitLinSpeed must be a positive number" << std::endl;
    }
  }
  ;

  /**
   * Store the Robot Parameters as constants
   *
   * The maximal speed in (rad/s) is calculated as
   * maxLinSpeed / (0.5 * WHEEL_DIAMETER)
   *
   * @param axleTrack distance between the centers of the wheels (m)
   * @param wheelDiameter diameter of the wheel (m)
   * @param cmdVelTimeout the maximal time between velocity commands before timeout
   * @param maxLimitLinSpeed maximal speed (m/s)
   * @param maxRevLimitLinSpeed maximal reverse speed (m/s) , (number is positive
   * @param maxLimitAngSpeed maximal speed (rad/s)
   * @param maxRevLimitAngSpeed maximal speed (rad/s)
   */
  Robot_Params(double axleTrack, double wheelDiameter, double cmdVelTimeout, double maxLimitLinSpeed,
               double maxRevLimitLinSpeed, double maxLimitAngSpeed, double maxRevLimitAngSpeed) :
      AXLE_TRACK(axleTrack), WHEEL_DIAMETER(wheelDiameter), CMD_VEL_TIMEOUT(cmdVelTimeout), MAX_LIMIT_LINSPEED(
          maxLimitLinSpeed), MAX_REV_LIMIT_LINSPEED(maxRevLimitLinSpeed), MAX_LIMIT_ANGSPEED(maxLimitAngSpeed), MAX_REV_LIMIT_ANGSPEED(
          maxRevLimitAngSpeed)
  {
  }
  ;

};

//Typedefs
//public:
//private:
/**
 * Robot_params struct
 */
typedef struct Robot_Params Robot_Params;

//Public Functions
//public:

//Private Functions
//private:

//Public Variables
//public:

//Private Variables
//private:

}//namespace end

#endif /* MOVEMENT_INCLUDE_MOVEMENT_ROBOT_PARAMS_H_INCLUDED */
