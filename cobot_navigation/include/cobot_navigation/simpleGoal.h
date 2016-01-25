/*
 * simpleGoal.h
 *
 *  Created on: Jan 25, 2016
 *      Author: aclangerak
 */

#ifndef INCLUDE_COBOT_NAVIGATION_SIMPLEGOAL_H_
#define INCLUDE_COBOT_NAVIGATION_SIMPLEGOAL_H_

namespace navigation
{


	struct simpleGoal
	{
		double x;
		double y;
		double theta;

		simpleGoal(double x_co, double y_co, double direction) : x(x_co),y(y_co),theta(direction){}
	};




	typedef struct simpleGoal simpleGoal;

}



#endif /* INCLUDE_COBOT_NAVIGATION_SIMPLEGOAL_H_ */
