/*
 * orderNode.cpp
 *
 *  Created on: Jan 25, 2016
 *      Author: aclangerak
 */

#include <ros/ros.h>

#include "cobot_navigation/orderController.h"


int main(int argc, char** argv) {
	ros::init(argc, argv, "OrderNode");

	navigation::OrderController orderController(argc,argv);

	orderController.spin();

/*
	ros::ServiceClient getOrderCntClient = nh.serviceClient<cobot_api_client::GetOrderCount>("/get_order_count");
	cobot_api_client::GetOrderCount getOrderCnt_srv;
	getOrderCnt_srv.request.request = true;

	if(getOrderCntClient.call(getOrderCnt_srv))
	{
		std::cout << "Response code " << getOrderCnt_srv.response.http_response_code << std::endl;
		std::cout << "Error message " << getOrderCnt_srv.response.error_message << std::endl;
		std::cout << "Order count " << getOrderCnt_srv.response.order_count << std::endl;
	}
*/


}
