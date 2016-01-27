/*
 * PWMBoardTest.cpp
 */

//=======================================================
// predefined headers
#include <ros/ros.h>
#include <threemxl/C3mxlROS.h>

//=======================================================
// new defined headers
#include "cobot_movement/PWMBoardTest.h"

//MODE 0 = PWM
//MODE other than 0 = SPEED
#define MODE 0

PWMBoardTest::PWMBoardTest() :
    nh("~"), motors(), maxPWM(0)
{
}

/**
 * (Default) Destructor
 * Delete motor interface, close serial port, and shut down node handle
 *
 */
PWMBoardTest::~PWMBoardTest()
{
  if (serial_port.is_port_open())
    serial_port.port_close();

  nh.shutdown();
}

/**
 * Initialize for testing
 * @param maxpwm maximal speed to climb to
 */
void PWMBoardTest::init(char *maxpwm)
{
  CDxlConfig *config = new CDxlConfig();

  maxPWM = boost::lexical_cast<double>(maxpwm);
  ROS_INFO("Set PWM to: %f",maxPWM);

  config->mDxlTypeStr = "3MXL";
  motors = new CDxlGroup();

  serial_port.port_open("/dev/ttyUSB0", LxSerial::RS485_FTDI);
  serial_port.set_speed(LxSerial::S921600);
  motors->setSerialPort(&serial_port);

  motors->addNewDynamixel(config->setID(107));
  motors->addNewDynamixel(config->setID(106));
  motors->init();

  if (MODE == 0)
  {
    motors->getDynamixel(0)->set3MxlMode(PWM_MODE);
    motors->getDynamixel(1)->set3MxlMode(PWM_MODE);
  }
  else
  {
    motors->getDynamixel(0)->set3MxlMode(SPEED_MODE);
    motors->getDynamixel(1)->set3MxlMode(SPEED_MODE);
  }
  delete config;
}

/**
 * Equals method for doubles
 * @param a number 1
 * @param b number 2
 * @param epsilon maximum difference
 * @return true if equals
 */
bool double_equals(double a, double b, double epsilon = 0.001)
{
  return std::abs(a - b) < epsilon;
}

/**
 * Run this Test
 */
void PWMBoardTest::spin()
{
  double rate = 5.0d;
  ros::Rate loop_rate(rate); //herz;

  double pwmSpeed = 0.0d;

  //Increment of each speed step
  double stepSize = 0.25;

  //Scale factor for setting speed at a lower rate
  //then the loop
  double modStepRate = 0.2;
  //For determining how many times the
  //loop has "looped"
  double cntRate = 0.0d;

  //Positive incrementing first
  if (maxPWM > 0)
  {
    //First, climb to max value
    bool toMax = true;

    //Set initial speed
    setSpeed(0, 0);

    while (ros::ok())
    {
      //Set speed to pwm speed
      setSpeed(pwmSpeed, pwmSpeed);
      ROS_DEBUG("Current PWM/SPEED value %f",pwmSpeed);

      //Check for errors
      check_EM_STOP();

      //Check if it is time for setting new speed
      if (double_equals(cntRate, rate))
      {
        //Check if max is not reached
        if (!(double_equals(pwmSpeed, maxPWM)) && (toMax)){
        pwmSpeed += stepSize;
        printf("Stepping up Speed, new value: %f till %f \n: ", pwmSpeed, maxPWM);
      }
      //Max is reached
      else
      {
        //Dont go to max value anymore
        toMax = false;

        //Check if 0 is not reached
        if (!double_equals(pwmSpeed, 0.0))
        {
          setSpeed(pwmSpeed,pwmSpeed);

          pwmSpeed -= stepSize;
          printf("Stepping down Speed from here: %f %f \n", pwmSpeed, 0.0);
        }
        //0 is reached, done
        else
        {
          printf("Done \n");
          break;
        }
      }

      //Reset countrate
      cntRate = 0.0d;
      std::cout << "cntrate " << cntRate << std::endl;
    }

    //Now it not the time for changing speed
    else
    {
      //Increment times loop has looped
      cntRate += modStepRate;
      std::cout << "cntrate " << cntRate << std::endl;
    }

      loop_rate.sleep();
    }

  }

  //Negative incrementing(decrementing) first
  else
  {
    //First go to low value
    bool toMin = true;

    //Set initial speed
    setSpeed(0, 0);
    ROS_DEBUG("Current PWM/SPEED value %f",pwmSpeed);

    while (ros::ok())
    {
      //Set speed to pwm speed
      setSpeed(pwmSpeed, pwmSpeed);
      ROS_DEBUG("Current PWM/SPEED value %f",pwmSpeed);

      //Check for errors
      check_EM_STOP();

      //Check if it is time for setting new speed
      if (double_equals(cntRate, rate))
      {
        //Check if min is not reached
        if (!(double_equals(pwmSpeed, maxPWM)) && toMin){
        pwmSpeed -= stepSize;
        printf("Stepping down Speed, new value: %f till %f \n: ", pwmSpeed, maxPWM);
      }
      //Min is reached
      else
      {
        //Go only up now
        toMin = false;

        //Check if 0 is not reached
        if (!double_equals(pwmSpeed, 0.0))
        {
          setSpeed(pwmSpeed,pwmSpeed);

          pwmSpeed += stepSize;
          printf("Stepping up Speed from here: %f %f \n", pwmSpeed, 0.0);
        }
        //0 is reached, done
        else
        {
          printf("Done \n");
          break;
        }

      }

      //Reset countrate
      cntRate = 0.0d;
      std::cout << "cntrate " << cntRate << std::endl;
    }

    //Now it not the time for changing speed
    else
    {
      //Increment times loop has looped
      cntRate += modStepRate;
      std::cout << "cntrate " << cntRate << std::endl;
    }

      loop_rate.sleep();
    }

  }

  setSpeed(0, 0);
  ROS_DEBUG("Current PWM/SPEED value %f",pwmSpeed);

}

/**
 * Set speed of motors
 * @param v_left left speed (PWM or SPEED)
 * @param v_right right speed (PWM or SPEED)
 */
void PWMBoardTest::setSpeed(double v_left, double v_right)
{
  if (MODE == 0)
  {
    motors->getDynamixel(0)->setPWM(v_left);
    motors->getDynamixel(1)->setPWM(v_right);
  }
  else
  {
    motors->getDynamixel(0)->setLinearSpeed(v_left);
    motors->getDynamixel(1)->setLinearSpeed(v_right);
  }
}

/**
 * Check if the emergency stop is active and
 * show other statuses
 */
void PWMBoardTest::check_EM_STOP()
{

  motors->getDynamixel(0)->getStatus();
  int status_0 = motors->getDynamixel(0)->presentStatus();
  ROS_DEBUG("Motor 0 status %s", motors->getDynamixel(0)->translateErrorCode(status_0));

  int status_1 = motors->getDynamixel(1)->presentStatus();
  motors->getDynamixel(1)->getStatus();
  ROS_DEBUG("Motor 1 status %s", motors->getDynamixel(1)->translateErrorCode(status_1));

  if (status_0 == M3XL_STATUS_EM_STOP_ERROR)
  {
    ROS_WARN("Emergency Button Pressed");
  }
  else if (status_1 == M3XL_STATUS_EM_STOP_ERROR)
  {
    ROS_WARN("Emergency Button Pressed");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PWMBoardTest");

  char *maxpwm = NULL;

  //Read in speed value
  if (argc == 2)
  {
    maxpwm = argv[1];
    PWMBoardTest boardTest;

    boardTest.init(maxpwm);
    //Run
    boardTest.spin();
  }
  else
  {
    ROS_ERROR("No speed value set, exiting");
  }

  return 0;
}
