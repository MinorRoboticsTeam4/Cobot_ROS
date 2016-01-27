/*
 * OrderController.cpp
 *
 *  Created on: Jan 25, 2016
 *      Author: aclangerak
 */

#include "cobot_navigation/orderController.h"

namespace navigation {

OrderController::OrderController(int argc, char** argv) :
		loopRate(0.1d), cobotBat_Vcc_charge(12.24), cobotBat_Vcc_critical(
				11.96), laptopBat_Cap_charge(30), laptopBat_Cap_critical(20), nh(
				"~"), navNode(argc, argv), toDockingStation(0, 0, 1) { //TODO dockingstationCoordinate read from file/parameter

	order_pub = nh.advertise<cobot_navigation::ArduinoOrder>("/arduino_receive",
			1);
	order_sub = nh.subscribe<cobot_navigation::ArduinoOrderStatus>("/arduino_send",
			100, &OrderController::receivedOrderFromArduino, this);

	battery_sub = nh.subscribe<cobot_movement::BatteryState>("/cobotBattery",
			10, &OrderController::batteryArduinoCb, this);

	getOrderClient = nh.serviceClient<cobot_api_client::OrderWithProducts>(
			"/get_current_order");

	setOrderStatusClient = nh.serviceClient<cobot_api_client::SetOrderStatus>(
			"/set_current_order_status");

	arduinoOrderStatus.isCompleted = true;

}

OrderController::~OrderController() {

}

void OrderController::setOrderStatus(navigation::statusCodes statusCode)
{
	ROS_INFO("[OrderClient]Updating status of order to %d" , statusCode);

	cobot_api_client::SetOrderStatus setOrderStatus_srv;
	setOrderStatus_srv.request.delivery_status = statusCode;

	if (setOrderStatusClient.call(setOrderStatus_srv)) {
		//Call to server succeeded
		if (setOrderStatus_srv.response.http_response_code == 200) {


		}
		//Server could not find requested item
		else if (setOrderStatus_srv.response.http_response_code == 404) {
			ROS_ERROR("[OrderController] 404, Could not find order: %s",
					setOrderStatus_srv.response.error_message.c_str());
		}
		//Cannot find server
		else if (setOrderStatus_srv.response.http_response_code == -1) {
			ROS_ERROR("[OrderController] Cannot find server error: %s",
					setOrderStatus_srv.response.error_message.c_str());
		}

	} else {
		ROS_WARN("[OrderController] Cobot api client is offline");
	}
}

void OrderController::sendOrderToArduino(Order receivedOrder) {

	cobot_navigation::ArduinoOrder arduino_Order;
	arduino_Order.header.stamp = ros::Time::now();

//	order.nfcID




	for(int i=0;i<8;i++)
	{
		arduino_Order.isMug[i] = receivedOrder.isMug[i];
	}
	for(int i=0;i<8;i++)
	{
		arduino_Order.coffeeTypes[i] = receivedOrder.coffee_type[i];
	}

	arduino_Order.orderAmount = receivedOrder.amount;

	arduinoOrderStatus.isCompleted = false;

	ROS_INFO("[OrderController] sending order to Arduino");
	order_pub.publish(arduino_Order);

	setOrderStatus(navigation::coffee_preperation);
}

void OrderController::receivedOrderFromArduino(
		cobot_navigation::ArduinoOrderStatus status) {
	ROS_INFO("Message received");

	if(status.isCompleted){
		arduinoOrderStatus.isCompleted = true;
		setOrderStatus(navigation::completed);
	}
}

void OrderController::batteryArduinoCb(cobot_movement::BatteryState battery) {
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
int OrderController::checkBatteryStatus() {
	int batCode = 2;
	if (battery_sub.getNumPublishers() > 0) {
		if (cobotBattery.charge > cobotBat_Vcc_charge) {
			ROS_INFO("[Battery] Cobot battery is ok");
			batCode = 0;
		} else if (cobotBattery.charge <= cobotBat_Vcc_charge
				&& cobotBattery.charge > cobotBat_Vcc_critical) {
			ROS_INFO("[Battery] Cobot battery is low, recommend charging");
			batCode = 1;
		} else {
			ROS_ERROR("[Battery] Cobot battery is too low");
			batCode = 2;
		}
	}

	int laptopBatteryStatus = monitor.getCurrentCapacity();

	if (monitor.isConnectedToMains()) {
		ROS_INFO("[Battery] Laptop is connected to mains");
		batCode = 3;
	} else if (laptopBatteryStatus > laptopBat_Cap_charge) {
		ROS_INFO("[Battery] Laptop battery is ok");
		batCode = 0;
	} else if (laptopBatteryStatus <= laptopBat_Cap_charge
			&& laptopBatteryStatus > laptopBat_Cap_critical) {
		ROS_INFO("[Battery] Laptop battery is low, recommend charging");
		batCode = 1;
	} else {
		ROS_ERROR("[Battery] Laptop battery is too low");
		batCode = 2;
	}

	return batCode;

}

Order OrderController::getOrderFromApi() {
	cobot_api_client::OrderWithProducts getOrder_srv;
	getOrder_srv.request.request = true;

	ROS_INFO("[OrderClient]Requesting Order ...");

	if (getOrderClient.call(getOrder_srv)) {
		//Call to server succeeded

		std::cout << "Error message:"<< getOrder_srv.response.error_message<< std::endl;


		if (getOrder_srv.response.http_response_code == 200) {

			Order newOrder(getOrder_srv.response.order_id,
					getOrder_srv.response.delivery_status,
					getOrder_srv.response.location_id,
					getOrder_srv.response.location_name,
					getOrder_srv.response.coordinate_x,
					getOrder_srv.response.coordinate_y,
					getOrder_srv.response.orientation_z,
					getOrder_srv.response.coffee_size,
					getOrder_srv.response.coffee_type.c_array(),
					getOrder_srv.response.coffee_mug.elems);
			/*
			 //Id's
			 std::cout << "User id:  " << getOrder_srv.response.user_id
			 << std::endl;

			 //Delivery info
			 std::cout << "Delivered At " << getOrder_srv.response.delivered_at
			 << std::endl;
			 */
			return newOrder;

		}
		//Server could not find requested item
		else if (getOrder_srv.response.http_response_code == 404) {
			ROS_WARN("[OrderController] There  are no new orders: %s",
					getOrder_srv.response.error_message.c_str());
		}
		//Cannot find server
		else if (getOrder_srv.response.http_response_code == -1) {
			ROS_ERROR("[OrderController] Cannot find server error: %s",
					getOrder_srv.response.error_message.c_str());
		}

	} else {
		ROS_WARN("[OrderController] Cobot api client is offline");
	}
	Order fake;
	return fake;
}

void OrderController::spin() {
	ros::Rate loop_rate(loopRate);

	//TODO location check????
	bool atDockingStation = true;

	while (ros::ok()) {
		ros::spinOnce();

		int batt_status_code = checkBatteryStatus();

		if (batt_status_code == 0) {

			if (!arduinoOrderStatus.isCompleted) {
				ROS_INFO("[OrderController] Waiting for order to complete...");
			} else {
				Order order = getOrderFromApi();
				//Order order;
				//order.locationId = 21;

				//Received order
				if (order.locationId != -42) {
					setOrderStatus(navigation::navigating);
					//bool location_reached = navNode.navigateToGoal(navNode.createSimpleGoal(order.x,order.y,order.theta));

					bool location_reached = true;
					if (location_reached) {
						sendOrderToArduino(order);
						atDockingStation = false;
					} else {
						ROS_ERROR(
								"[orderController] Could not send order to Arduino, location not reached");
					}
				}

			}

		} else if (batt_status_code == 1 && !atDockingStation) {

			if (arduinoOrderStatus.isCompleted == false) {
				ROS_INFO("[OrderController] Waiting for order to complete...");
			} else {
				ROS_WARN(
						"[orderController] Battery getting low, returning to Docking Station");
				//TODO Get Location of "Docking station"
				bool location_reached = navNode.navigateToGoal(
						navNode.createSimpleGoal(toDockingStation.x,
								toDockingStation.y, toDockingStation.theta));

				if (!location_reached) {
					ROS_ERROR(
							"[orderController] Could not drive back to the Docking Station");
				} else {
					atDockingStation = true;
				}
			}

		} else if (batt_status_code == 2) {
			ROS_ERROR(
					"PLEASE, PUT THE ROBOT AND LAPTOP IN CHARGER OR TURN OFF EVERYTHING");

		} else if (batt_status_code == 3 && arduinoOrderStatus.isCompleted) {
			ROS_INFO("[Battery] Connected to Mains... cannot take new orders");
		}

		loop_rate.sleep();
	}

}

} /* namespace navigation */
