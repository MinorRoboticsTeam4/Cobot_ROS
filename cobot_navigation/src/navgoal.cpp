/*
 * navgoal.cpp
 *
 *  Created on: Jan 25, 2016
 *      Author: aclangerak
 */

#include "cobot_navigation/navgoal.h"

namespace navigation {

Nav_goal::Nav_goal(int argc, char** argv) :
		goalFrameId("/map"), client("move_base", false), nh() {
	ros::init(argc, argv, "Nav_Goal");

	//TODO UNCOMMENT
	/*
	while(!client.waitForServer(ros::Duration(5.0d)))
	{
	    ROS_INFO("Waiting for the move_base action server to come up");
	}*/

}

Nav_goal::~Nav_goal() {

}

simpleGoal Nav_goal::createSimpleGoal(double x_co, double y_co,
		double direction) {
	simpleGoal newGoal(x_co, y_co, direction);
	return newGoal;
}

bool Nav_goal::navigateToGoal(simpleGoal goal) {

	std::cout << "Navigation goal sending" <<std::endl;
	client.sendGoal(createMoveBaseGoal(goal));


	//TODO look into cancelling this goal with general BATTERY MONITOR
	client.waitForResult();


	if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("[navgoal] Success");
		return true;
	}
	else
	{
		ROS_ERROR("[navgoal] could not complete goal");
		return false;
	}

}

move_base_msgs::MoveBaseGoal Nav_goal::createMoveBaseGoal(simpleGoal goal) {
	move_base_msgs::MoveBaseGoal move_base_goal;

	move_base_goal.target_pose.header.frame_id = goalFrameId;
	move_base_goal.target_pose.header.stamp = ros::Time::now();

	move_base_goal.target_pose.pose.position.x = goal.x;
	move_base_goal.target_pose.pose.position.y = goal.y;

	geometry_msgs::Quaternion direction = tf::createQuaternionMsgFromYaw(
			goal.theta);
	move_base_goal.target_pose.pose.orientation = direction;

	ROS_INFO("[navgoal] Created a Goal with x: %f , y: %f , th: %f", goal.x,
			goal.y, goal.theta);

	return move_base_goal;
}



}


