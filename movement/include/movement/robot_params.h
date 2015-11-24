/*
 * motor_params.h
 *
 *  Created on: Nov 16, 2015
 *      Author: aclangerak
 */

//=======================================================
// include guard
#ifndef MOVEMENT_INCLUDE_MOVEMENT_ROBOT_PARAMS_H_
#define MOVEMENT_INCLUDE_MOVEMENT_ROBOT_PARAMS_H_

//=======================================================
// predefined headers

//=======================================================
// new defined headers


//Robot parameters, pulled of Technical Manual
//TODO verify
const double AXLE_TRACK = 0.600d;                         //(m)
const double WHEEL_DIAMETER = 0.297d;                     //(m)

const double CMD_VEL_TIMEOUT = 5.0d;                         //(s)
const double UPDATE_RATE = 1.0d;                          //(Hz)



#endif /* MOVEMENT_INCLUDE_MOVEMENT_ROBOT_PARAMS_H_ */
