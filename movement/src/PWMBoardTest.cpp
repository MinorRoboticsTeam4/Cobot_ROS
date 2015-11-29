#include <ros/ros.h>
#include <threemxl/C3mxlROS.h>
#include "movement/PWMBoardTest.h"

double maxPWM;
//TODO remove this errorcount code, unnessarcy
//TODO add comments
int timeStop = 100;   //time error is detected

void DxlROSExample::init(char *maxpwm, char *timestop)
{
  CDxlConfig *config = new CDxlConfig();

  if (maxpwm)
  {
    maxPWM = boost::lexical_cast<double>(maxpwm);
    std::cout << "Set PWM to: " << maxPWM << std::endl;
  }
  if (timestop)
  {
    timeStop = boost::lexical_cast<int>(timestop);
    std::cout << "Set stop time for EM message to: " << timeStop << std::endl;
  }
  ROS_INFO("Using direct connection");
  //  motor_ = new C3mxl();

  config->mDxlTypeStr = "3MXL";
  motors_ = new CDxlGroup();

  serial_port_.port_open("/dev/ttyUSB0", LxSerial::RS485_FTDI);
  serial_port_.set_speed(LxSerial::S921600);
  motors_->setSerialPort(&serial_port_);

  motors_->addNewDynamixel(config->setID(107));
  motors_->addNewDynamixel(config->setID(106));
  motors_->init();
  motors_->getDynamixel(0)->set3MxlMode(PWM_MODE);
  motors_->getDynamixel(1)->set3MxlMode(PWM_MODE);
//  motors_->getDynamixel(0)->set3MxlMode(SPEED_MODE);
//  motors_->getDynamixel(1)->set3MxlMode(SPEED_MODE);
  delete config;
}

bool double_equals(double a, double b, double epsilon = 0.001)
{
  return std::abs(a - b) < epsilon;
}

void DxlROSExample::spin()
{
  double rate = 5.0d;
  ros::Rate loop_rate(rate); //herz;

  double pwmSpeed = 0.0d;
  double stepSize = 0.2;

  double modStepRate = 0.2;
  double cntRate = 0.0d;

  int errorcnt = 0;

  if (maxPWM > 0)
  {

    bool toMax = true;

    motors_->getDynamixel(0)->setPWM(0);
    motors_->getDynamixel(1)->setPWM(0);
//    motors_->getDynamixel(0)->setLinearSpeed(0);
//    motors_->getDynamixel(1)->setLinearSpeed(0);

    while (ros::ok())
    {
      motors_->getDynamixel(0)->setPWM(pwmSpeed);
      motors_->getDynamixel(1)->setPWM(pwmSpeed);
      std::cout << "Current PWM value " << pwmSpeed << std::endl;
//      motors_->getDynamixel(0)->setLinearSpeed(pwmSpeed);
//      motors_->getDynamixel(1)->setLinearSpeed(pwmSpeed);

      motors_->getDynamixel(0)->getStatus();

      int status_0 = motors_->getDynamixel(0)->presentStatus();

      std::cout << "Motor 0 status" << motors_->getDynamixel(0)->translateErrorCode(status_0) << std::endl;

      int status_1 = motors_->getDynamixel(1)->presentStatus();

      motors_->getDynamixel(1)->getStatus();
      std::cout << "Motor 1 status" << motors_->getDynamixel(0)->translateErrorCode(status_1) << std::endl;

      std::cout << "Current PWM value " << pwmSpeed << std::endl;

      if (status_1 == M3XL_STATUS_EM_STOP_ERROR)
      {

        if (errorcnt == timeStop)
        {
          toMax = false; //Go back to 0
        }
        else
        {
          errorcnt += 1;
        }
      }
      else if (status_0 == M3XL_STATUS_EM_STOP_ERROR)
      {
        if (errorcnt == timeStop)
        {
          toMax = false; //Go back to 0
        }
        else
        {
          errorcnt += 1;
        }
      }

      if (double_equals(cntRate, rate))
      {
        if (!(double_equals(pwmSpeed, maxPWM)) && toMax){
        pwmSpeed += stepSize;
        printf("Stepping up pwmSpeed, new value: %f till %f \n: ", pwmSpeed, maxPWM);
      }
      else
      {
        toMax = false;

        if (!double_equals(pwmSpeed, 0.0))
        {
          motors_->getDynamixel(0)->setPWM(pwmSpeed);
          motors_->getDynamixel(1)->setPWM(pwmSpeed);

//          motors_->getDynamixel(0)->setLinearSpeed(pwmSpeed);
//          motors_->getDynamixel(1)->setLinearSpeed(pwmSpeed);

          printf("Stepping down pwmSpeed from here: %f %f \n", pwmSpeed, 0.0);

          pwmSpeed -= stepSize;
        }
        else
        {
          printf("Done \n");
          break;
        }

      }

      cntRate = 0.0d;
      std::cout << "cntrate " << cntRate << std::endl;
    }

    else
    {
      cntRate += modStepRate;
      std::cout << "cntrate " << cntRate << std::endl;
    }

      loop_rate.sleep();
    }

  }
  else
  {
    bool toMin = true;

    motors_->getDynamixel(0)->setPWM(0);
    motors_->getDynamixel(1)->setPWM(0);
//    motors_->getDynamixel(0)->setLinearSpeed(0);
//    motors_->getDynamixel(1)->setLinearSpeed(0);

    while (ros::ok())
    {
      motors_->getDynamixel(0)->setPWM(pwmSpeed);
      motors_->getDynamixel(1)->setPWM(pwmSpeed);
//      motors_->getDynamixel(0)->setLinearSpeed(pwmSpeed);
//      motors_->getDynamixel(1)->setLinearSpeed(pwmSpeed);

      motors_->getDynamixel(0)->getStatus();
      int status_0 = motors_->getDynamixel(0)->presentStatus();

      std::cout << "Motor 0 status" << motors_->getDynamixel(0)->translateErrorCode(status_0) << std::endl;

      int status_1 = motors_->getDynamixel(1)->presentStatus();

      motors_->getDynamixel(1)->getStatus();
      std::cout << "Motor 1 status" << motors_->getDynamixel(0)->translateErrorCode(status_1) << std::endl;

      std::cout << "Current PWM value " << pwmSpeed << std::endl;

      if (status_1 == M3XL_STATUS_EM_STOP_ERROR)
      {
        if (errorcnt == timeStop)
        {
          toMin = false; //Go back to 0
        }
        else
        {
          errorcnt += 1;
        }
      }
      else if (status_0 == M3XL_STATUS_EM_STOP_ERROR)
      {

        if (errorcnt == timeStop)
        {
          toMin = false; //Go back to 0
        }
        else
        {
          errorcnt += 1;
        }
      }

      if (double_equals(cntRate, rate))
      {
        if (!(double_equals(pwmSpeed, maxPWM)) && toMin){
        pwmSpeed -= stepSize;
        printf("Stepping down pwmSpeed, new value: %f till %f \n: ", pwmSpeed, maxPWM);
      }
      else
      {
        toMin = false;

        if (!double_equals(pwmSpeed, 0.0))
        {
          motors_->getDynamixel(0)->setPWM(pwmSpeed);
          motors_->getDynamixel(1)->setPWM(pwmSpeed);

//          motors_->getDynamixel(0)->setLinearSpeed(pwmSpeed);
//          motors_->getDynamixel(1)->setLinearSpeed(pwmSpeed);

          printf("Stepping up pwmSpeed from here: %f %f \n", pwmSpeed, 0.0);

          pwmSpeed += stepSize;
        }
        else
        {
          printf("Done \n");
          break;
        }

      }

      cntRate = 0.0d;
      std::cout << "cntrate " << cntRate << std::endl;
    }

    else
    {
      cntRate += modStepRate;
      std::cout << "cntrate " << cntRate << std::endl;
    }

      loop_rate.sleep();
    }

  }
//  motor_->setSpeed(0);
//  motor_->set3MxlMode(STOP_MODE);

  motors_->getDynamixel(0)->setPWM(0.0);
  motors_->getDynamixel(1)->setPWM(0.0);
//  motors_->getDynamixel(0)->setLinearSpeed(0);
//  motors_->getDynamixel(1)->setLinearSpeed(0);

//  motors_->getDynamixel(0)->set3MxlMode(STOP_MODE);
//  motors_->getDynamixel(1)->set3MxlMode(STOP_MODE);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dxl_ros_example");

  char *maxpwm = NULL;
  char *timeStop = NULL;
  if (argc == 2)
    maxpwm = argv[1];

  if (argc == 3)
  {
    maxpwm = argv[1];
    timeStop = argv[2];
  }
  DxlROSExample dxl_ros_example;

  dxl_ros_example.init(maxpwm, timeStop);
  dxl_ros_example.spin();

  return 0;
}
