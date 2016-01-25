/*
 * navgoal.h
 *
 *  Created on: Jan 25, 2016
 *      Author: aclangerak
 */

#ifndef SRC_NAVGOAL_H_
#define SRC_NAVGOAL_H_

#include <iostream>

#include <ros/ros.h>
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib_msgs/GoalStatus.h>
#include <geometry_msgs/Quaternion.h>


#include "simpleGoal.h"


namespace navigation
{

class Nav_goal {
public:
	Nav_goal(int argc, char** argv);
	~Nav_goal();

	bool navigateToGoal(simpleGoal goal);
	simpleGoal createSimpleGoal(double x_co, double y_co, double direction);

private:


	ros::NodeHandle nh;

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client;
	move_base_msgs::MoveBaseGoal createMoveBaseGoal(simpleGoal goal);

	std::string goalFrameId;

};

}

#endif /* SRC_NAVGOAL_H_ */
