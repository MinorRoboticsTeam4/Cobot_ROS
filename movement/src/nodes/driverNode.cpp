/*
 * driverNode.cpp
 * 
 */

//=======================================================
// predefined headers
//=======================================================
// new defined headers
#include "movement/driver.h"

#define PI 3.141592653589793238462643383279502884

int main(int argc, char** argv)
{
  //Init rosNode
  ros::init(argc, argv, "test_driver");
  movement::Driver driver;

  //Drive

  /*
   std::cout << "LINE" << std::endl;
   sleep(1);
   driver.drive_straight(0.5, 0.2);
   driver.drive_straight(-0.5, 0.2);
   driver.drive_straight(0.25, 0.2);
   */

  std::cout << "Speed up LINE" << std::endl;
  sleep(1);
  for(int i=1; i < 10 ; i++)
  {
    driver.drive_straight(0.5, i * 0.2);
    driver.drive_straight(-0.5, i * 0.2);
  }
  driver.drive_straight(0.25, 0.2);

  /*  std::cout << "SPIN" << std::endl;
   sleep(1);
   driver.turn(2 * PI, 0.3);
   driver.turn(-2 * PI, 0.3);*/

  std::cout << "Speed up SPIN" << std::endl;
  sleep(1);
  for(int i=1; i < 10 ; i++)
  {
    driver.turn(2 * PI, i * 0.3);
    driver.turn(-2 * PI, i * 0.3);
  }


  /*  std::cout << "LINE + SPIN" << std::endl;
   sleep(1);
   for (int i = 0; i < 2; ++i)
   {
   driver.drive_straight(0.5, 0.2);
   driver.turn(1 * PI, 0.3);
   }
   driver.drive_straight(0.25, 0.2);*/

  //Can give problems with turtleboard because unknown error
  /*  std::cout << "TRIANGLE" << std::endl;
   sleep(1);
   for (int i = 0; i < 3; ++i)
   {
   driver.drive_straight(0.5, 0.2);
   driver.turn((2.0 / 3.0) * PI, 0.3);
   }
   driver.drive_straight(0.25, 0.2);*/

  /*  std::cout << "SQUARE" << std::endl;
   sleep(1);
   for (int i = 0; i < 4; ++i)
   {
   driver.drive_straight(0.5, 0.2);
   driver.turn((2.0 / 4.0) * PI, 0.3);
   }
   driver.drive_straight(0.25, 0.2);
   */
  /*
   std::cout << "PLUS" << std::endl;
   sleep(1);
   for (int i = 0; i < 4; ++i)
   {
   driver.drive_straight(0.5, 0.2);
   driver.drive_straight(-0.5, 0.2);
   driver.turn(0.5 * PI, 0.3);
   }*/

  std::cout << "Finished" << std::endl;

  return EXIT_SUCCESS;
}
