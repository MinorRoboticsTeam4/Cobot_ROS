/*
 * orderNode.cpp
 *
 *  Created on: Jan 25, 2016
 *      Author: aclangerak
 */

#include <ros/ros.h>
#include <cobot_navigation/ArduinoOrder.h>
#include <cobot_movement/BatteryState.h>

#include <cobot_api_client/GetOrderCount.h>


#include "cobot_navigation/navgoal.h"
#include "cobot_navigation/laptopBatteryMonitor.h"



double loopRate = 0.1d;

double cobotBat_Vcc_charge = 12.24;
double cobotBat_Vcc_critical = 11.96;

double laptopBat_Cap_charge = 98;
double laptopBat_Cap_critical = 94;

navigation::Laptop_Battery_Monitor monitor;

ros::Publisher order_pub;
ros::Subscriber battery_sub;
cobot_movement::BatteryState cobotBattery;

void sendOrderToArduino()
{
	cobot_navigation::ArduinoOrder order;

	order.header.stamp = ros::Time::now();

//	order.containerType
//	order.nfcID
//	order.coffeeTypes

	order_pub.publish(order);
}

void batteryArduino(cobot_movement::BatteryState battery){
	cobotBattery = battery;
}

/**
 * Status codes:
 *
 *
 * batteriesOk				0
 * batteriesChargeNeed		1
 * batteriesCritical		2
 * batteriesCharging		3
 *
 */
int checkBatteryStatus()
{
	int batCode = 2;
	if(battery_sub.getNumPublishers() > 0)
	{
	  if(cobotBattery.charge > cobotBat_Vcc_charge){
		  //Ok
		  ROS_INFO("[Battery] Cobot battery is ok");
		  batCode = 0;
	  }
	  else if(cobotBattery.charge <= cobotBat_Vcc_charge && cobotBattery.charge > cobotBat_Vcc_critical){
		  //Recommend charging
		  ROS_INFO("[Battery] Cobot battery is low, recommend charging");
		  batCode = 1;
	  }
	  else {
		  //Stop
		  ROS_ERROR("[Battery] Cobot battery is too low");
		  batCode = 2;
	  }
	}

	int laptopBatteryStatus = monitor.getCurrentCapacity();

	if(monitor.isCharging())
	{
	    ROS_INFO("[Battery] Laptop battery is charging");
		batCode = 3;
	}
	else if(laptopBatteryStatus > laptopBat_Cap_charge)
	{
	  //Ok
      ROS_INFO("[Battery] Laptop battery is ok");
	  batCode = 0;
	}
	else if(laptopBatteryStatus <= laptopBat_Cap_charge && laptopBatteryStatus > laptopBat_Cap_critical)
	{
	  //Recommend charging
	  ROS_INFO("[Battery] Laptop battery is low, recommend charging");
	  batCode = 1;
	}
	else
	{
	  //Stop
	  ROS_ERROR("[Battery] Laptop battery is too low");
	  batCode = 2;
	}

	return batCode;

}


void spin(navigation::Nav_goal& navNode)
{
	ros::Rate loop_rate(loopRate);

	//TODO location check????
	bool atDockingStation = true;

	  while (ros::ok())
	  {
	    ros::spinOnce();

	    int batt_status_code = checkBatteryStatus();

	    if(batt_status_code == 0)
	    {
		    //TODO Call here relevant stuff for getting a new navigation goal
		    //bool location_reached = navNode.navigateToGoal(navNode.createSimpleGoal(0,0,1));


	    	bool location_reached = true;
		    if(location_reached)
		    {
		    	//sendOrderToArduino();
		    	atDockingStation = false;
		    }
		    else
		    {
		    	ROS_ERROR("[orderNode] Could not send order to Arduino, location not reached");
		    }
	    }
	    else if(batt_status_code == 1 && !atDockingStation)
	    {
		    //TODO Get Location of "Docking station"
		    //bool location_reached = navNode.navigateToGoal(navNode.createSimpleGoal(0,0,1));
		    bool location_reached = true;
	    	if(!location_reached)
		    {
		    	ROS_ERROR("[orderNode] Could not drive back to the Docking Station");
		    }
	    	else
	    	{
	    		atDockingStation = true;
	    	}
	    }
	    else if(batt_status_code == 2)
	    {
	    	ROS_ERROR("PLEASE, PUT THE ROBOT AND LAPTOP IN CHARGER OR TURN OFF EVERYTHING");

	    }
	    else if(batt_status_code == 3)
	    {
	    	ROS_INFO("[Battery] Charging... cannot move the robot");
	    }


	    loop_rate.sleep();
	  }

}



int main(int argc, char** argv) {
	ros::init(argc, argv, "OrderNode");

	ros::NodeHandle nh;

	order_pub = nh.advertise<cobot_navigation::ArduinoOrder>("/arduino_topic", 1);
	battery_sub = nh.subscribe<cobot_movement::BatteryState>("/cobotBattery",10,&batteryArduino);

	navigation::Nav_goal navGoal(argc, argv);
	spin(navGoal);

}
