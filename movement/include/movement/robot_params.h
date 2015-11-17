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


//AutoStop robot if no command has received (ms)
#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = AUTO_STOP_INTERVAL;



//Robot parameters, pulled of Technical Manual
//TODO verify
const double AXLE_TRACK = 0.600d;                         //(m)
const double WHEEL_DIAMETER = 0.297d;                     //(m)




#endif /* MOVEMENT_INCLUDE_MOVEMENT_ROBOT_PARAMS_H_ */
