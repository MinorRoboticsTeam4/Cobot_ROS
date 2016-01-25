/*
 * driverNode.cpp
 * 
 */

//=======================================================
// predefined headers
#include <algorithm>

//=======================================================
// new defined headers
#include "cobot_teleop/driver.h"

#define PI 3.141592653589793238462643383279502884

int main(int argc, char** argv)
{
  //Init rosNode
  ros::init(argc, argv, "test_driver");
  movement::Driver driver;

  //Drive

  std::vector<std::string> commands = driver.getCommands();
  double linearSpeed = driver.getLinearSpeed();
  double angularSpeed = driver.getAngularSpeed();

  for (int i = 0; i < commands.size(); ++i)
  {
    if (std::find(commands.begin(), commands.end(), "STRAIGHT LINE") != commands.end())
    {
      driver.drive_straight(1.5, linearSpeed);
    }

    if (std::find(commands.begin(), commands.end(), "LINE") != commands.end())
    {
      driver.drive_straight(1.0, linearSpeed);
      driver.drive_straight(-1.5, linearSpeed);
      driver.drive_straight(0.25, linearSpeed);
    }

    if (std::find(commands.begin(), commands.end(), "SPEED LINE") != commands.end())
    {
      for (int i = 1; i < 10; i++)
      {
        driver.drive_straight(0.5, i  * linearSpeed);
        driver.drive_straight(-0.5, i * linearSpeed);
      }
      driver.drive_straight(0.25, linearSpeed);
    }

    if (std::find(commands.begin(), commands.end(), "SPIN") != commands.end())
    {
      driver.turn(2 * PI, angularSpeed);
      driver.turn(-2 * PI, angularSpeed);
    }

    if (std::find(commands.begin(), commands.end(), "SPEED SPIN") != commands.end())
    {
      for (int i = 1; i < 10; i++)
      {
        driver.turn(2 * PI, i * angularSpeed);
        driver.turn(-2 * PI, i * angularSpeed);
      }
    }

    if (std::find(commands.begin(), commands.end(), "LINE TURN") != commands.end())
    {
      driver.drive_straight(3.0, linearSpeed);
      driver.turn(1 * PI, angularSpeed);
      driver.drive_straight(3.0, linearSpeed);
      
    }

    if (std::find(commands.begin(), commands.end(), "LINE AND SPIN") != commands.end())
    {
      for (int i = 0; i < 2; ++i)
      {
        driver.drive_straight(0.5, linearSpeed);
        driver.turn(1 * PI, angularSpeed);
      }
      driver.drive_straight(0.25, linearSpeed);
    }

    if (std::find(commands.begin(), commands.end(), "TRIANGLE") != commands.end())
    {
      for (int i = 0; i < 3; ++i)
      {
        driver.drive_straight(0.5, linearSpeed);
        driver.turn((2.0 / 3.0) * PI, angularSpeed);
      }
      driver.drive_straight(0.25, linearSpeed);
    }

    if (std::find(commands.begin(), commands.end(), "SQUARE") != commands.end())
    {
      for (int i = 0; i < 4; ++i)
      {
        driver.drive_straight(0.5, linearSpeed);
        driver.turn((2.0 / 4.0) * PI, angularSpeed);
      }
      driver.drive_straight(0.25, linearSpeed);
    }

    if (std::find(commands.begin(), commands.end(), "PLUS") != commands.end())
    {
      for (int i = 0; i < 4; ++i)
      {
        driver.drive_straight(0.5, linearSpeed);
        driver.drive_straight(-0.5, linearSpeed);
        driver.turn((2.0 / 4.0) * PI, angularSpeed);
      }
    }

  }
  std::cout << "Finished" << std::endl;

  return EXIT_SUCCESS;
}
