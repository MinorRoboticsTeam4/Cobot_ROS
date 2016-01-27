/*
 * testPub.cpp
 *
 *  Created on: Jan 26, 2016
 *      Author: aclangerak
 */

#include <ros/ros.h>

#include <cobot_navigation/ArduinoOrder.h>
#include <cobot_navigation/ArduinoOrderStatus.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "testPub");

	ros::NodeHandle nh;
	ros::Publisher order_pub;

	//order_pub = nh.advertise<cobot_navigation::ArduinoOrder>("/arduino_receive",1);
	order_pub = nh.advertise<cobot_navigation::ArduinoOrderStatus>("/arduino_send",1);

	//cobot_navigation::ArduinoOrder order;
	cobot_navigation::ArduinoOrderStatus status;
	status.isCompleted = true;

	bool send = false;


	ros::Rate poll_rate(100);
	while(order_pub.getNumSubscribers() == 0){
	    poll_rate.sleep();
	}

	while(ros::ok()){
		ros::spinOnce();

		if(!send){
			order_pub.publish(status);
			send = true;
		}
		ros::spinOnce();
	}

	return true;
}
