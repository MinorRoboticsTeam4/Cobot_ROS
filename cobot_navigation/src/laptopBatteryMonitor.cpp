/*
 * laptopBatteryMonitor.cpp
 *
 *  Created on: Jan 25, 2016
 *      Author: aclangerak
 */


#include <stdio.h>
#include "cobot_navigation/laptopBatteryMonitor.h"

#include <iostream>
#include <fstream>

namespace navigation {

Laptop_Battery_Monitor::Laptop_Battery_Monitor() : batt_cap_path("/sys/class/power_supply/BAT1/capacity") , batt_status_path("/sys/class/power_supply/BAT1/status"){


}

Laptop_Battery_Monitor::~Laptop_Battery_Monitor() {

}

bool Laptop_Battery_Monitor::isConnectedToMains()
{
   std::ifstream myfile;
   myfile.open(batt_status_path.c_str());

   std::string batStatus;

   if (myfile.is_open())
    {
	  std::getline(myfile,batStatus);
      myfile.close();
    }

   if(batStatus == "Charging" || batStatus == "Full")
   {
	   return true;
   }
   else
   {
	   return false;
   }
}

int Laptop_Battery_Monitor::getCurrentCapacity()
{
	int batCap = -1;
	FILE* f = fopen(batt_cap_path.c_str(), "r" );
	if( f ) {
	fscanf( f, "%i", &batCap );
	fclose( f );
	}
	return batCap;
}

} /* namespace navigation */
