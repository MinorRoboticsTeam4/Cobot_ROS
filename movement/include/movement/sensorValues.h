/*
 * sensorValues.h
 * 
 */

//=======================================================
// include guard
#ifndef MOVEMENT_INCLUDE_MOVEMENT_SENSORVALUES_H__INCLUDED
#define MOVEMENT_INCLUDE_MOVEMENT_SENSORVALUES_H__INCLUDED

//=======================================================
// predefined headers
//=======================================================
// new defined headers

namespace movement
{

/**
 * Stores the values of the read sensors
 */
struct SensorValues
{

  /**
   * the position of the left motor (m)
   */
  double linearPos_Left;
  /**
   * the position of the right motor (m)
   */
  double linearPos_Right;
  /**
   * the position of the left motor (rad)
   */
  double angularPos_Left;
  /**
   * the position of the right motor (rad)
   */
  double angularPos_Right;

  /**
   * The charge of the batteries
   */
  unsigned int charge;
  /**
   * the capacity of the batteries
   */
  unsigned int capacity;
};

//Typedefs
//public:
//private:
/**
 * sensorvalues struct
 */
typedef struct SensorValues SensorValues;

//Public Functions
//public:

//Private Functions
//private:

//Public Variables
//public:

//Private Variables
//private:

}//namespace end

#endif /* MOVEMENT_INCLUDE_MOVEMENT_SENSORVALUES_H__INCLUDED */
