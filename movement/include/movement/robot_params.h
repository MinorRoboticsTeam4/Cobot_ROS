/*
 * motor_params.h
 *
 *  Created on: Nov 16, 2015
 *      Author: aclangerak
 */

//=======================================================
// include guard
#ifndef MOVEMENT_INCLUDE_MOVEMENT_ROBOT_PARAMS_H_INCLUDED
#define MOVEMENT_INCLUDE_MOVEMENT_ROBOT_PARAMS_H_INCLUDED

//=======================================================
// predefined headers

//=======================================================
// new defined headers


//Robot parameters, pulled of Technical Manual
//TODO verify
const double AXLE_TRACK = 0.600d;                                        //(m)
const double WHEEL_DIAMETER = 0.297d;                                    //(m)

const double CMD_VEL_TIMEOUT = 2.0d;                                     //(s)
const double UPDATE_RATE = 30.0d;                                         //(Hz)

//Based on that robot moves at max 1 meter per 1 second
//TODO test values
const double MAX_LIMIT_LINSPEED = 1.0d;                                  //(m/s)
const double MAX_REV_LIMIT_LINSPEED = -1.0d;                             //(m/s)
const double MAX_LIMIT_ANGSPEED = 1.0d / (0.5*WHEEL_DIAMETER);           //(rad/s)
const double MAX_REV_LIMIT_ANGSPEED = -1.0d / (0.5*WHEEL_DIAMETER);      //(rad/s)

#endif /* MOVEMENT_INCLUDE_MOVEMENT_ROBOT_PARAMS_H_INCLUDED */
