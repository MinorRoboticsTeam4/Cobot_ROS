/*
 * OrderController.h
 *
 *  Created on: Jan 25, 2016
 *      Author: aclangerak
 */

#ifndef SRC_ORDERCONTROLLER_H_
#define SRC_ORDERCONTROLLER_H_

#include <ros/ros.h>
#include <cobot_navigation/ArduinoOrder.h>
#include <cobot_navigation/ArduinoOrderStatus.h>
#include <cobot_movement/BatteryState.h>

#include <cobot_api_client/OrderWithProducts.h>
#include <cobot_api_client/SetOrderStatus.h>

#include "cobot_navigation/navgoal.h"
#include "cobot_navigation/laptopBatteryMonitor.h"

namespace navigation {

	enum statusCodes{
		in_queue = 0,
		navigating = 1,
		coffee_preperation = 2,
		completed = 3,
	};


typedef struct Order {

	int orderId;
	int deliveryStatus;

	int locationId;
	std::string locationName;
	double x;
	double y;
	double theta;

	int amount;
	std::vector<long int> coffee_type;
	std::vector<unsigned char> isMug;

	Order():locationId(-42){};

	Order(double x_co,double y_co, double angle):x(
			x_co), y(y_co), theta(angle){};

	Order(int order_id,int status, int loc_id, std::string loc_name, double x_co,
			double y_co, double angle, double coffee_Amount, int types[8], unsigned char mugs[8]) :
				orderId(order_id),deliveryStatus(status), locationId(loc_id), locationName(loc_name), x(
					x_co), y(y_co), theta(angle),amount(coffee_Amount) {
		for(int i=0;i<8;i++)
		{
			coffee_type.push_back(types[i]);
			std::cout << "Coffee type ["<< i << "] " << types[i] << ",";
		}
		std::cout << std::endl;
		for(int i=0;i<8;i++)
		{
			isMug.push_back(mugs[i]);
			//std::cout << std::boolalpha << "Mug ["<< i << "] " << isMug[i] << ",";
		}
		//std::cout << std::endl;

	}
	;

} Order;

class OrderController {
public:
	OrderController(int argc, char** argv);
	~OrderController();

	void sendOrderToArduino(Order receivedOrder);
	void receivedOrderFromArduino(cobot_navigation::ArduinoOrderStatus status);

	void setOrderStatus(navigation::statusCodes statusCode);

	void batteryArduinoCb(cobot_movement::BatteryState battery);
	int checkBatteryStatus();

	Order getOrderFromApi();

	void spin();

private:

	double loopRate;

	double cobotBat_Vcc_charge;
	double cobotBat_Vcc_critical;

	double laptopBat_Cap_charge;
	double laptopBat_Cap_critical;

	Order toDockingStation;

	cobot_navigation::ArduinoOrderStatus arduinoOrderStatus;

	navigation::Laptop_Battery_Monitor monitor;
	cobot_movement::BatteryState cobotBattery;
	navigation::Nav_goal navNode;

	ros::ServiceClient getOrderClient;
	ros::ServiceClient setOrderStatusClient;

	ros::Publisher order_pub;
	ros::Subscriber order_sub;

	ros::Subscriber battery_sub;
	ros::NodeHandle nh;

};

} /* namespace navigation */

#endif /* SRC_ORDERCONTROLLER_H_ */
