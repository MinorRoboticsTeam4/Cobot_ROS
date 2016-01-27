/*
 * laptopBatteryMonitor.h
 *
 *  Created on: Jan 25, 2016
 *      Author: aclangerak
 */

#ifndef SRC_LAPTOPBATTERYMONITOR_H_
#define SRC_LAPTOPBATTERYMONITOR_H_

#include <iostream>
#include <string>

namespace navigation {

class Laptop_Battery_Monitor {
public:
	Laptop_Battery_Monitor();
	~Laptop_Battery_Monitor();

	bool isConnectedToMains();
	int getCurrentCapacity();

private:
	std::string batt_cap_path;
	std::string batt_status_path;
};

}

#endif /* SRC_LAPTOPBATTERYMONITOR_H_ */
