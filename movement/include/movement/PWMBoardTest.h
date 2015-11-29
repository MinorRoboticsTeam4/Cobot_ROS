#ifndef MOVEMENT_INCLUDE_MOVEMENT_TEST3MXLBOARD_H
#define MOVEMENT_INCLUDE_mOVEMENT_TEST3MXLBOARD_H

#include <ros/ros.h>
#include <threemxl/platform/hardware/dynamixel/CDxlGeneric.h>
#include <threemxl/platform/hardware/dynamixel/CDxlGroup.h>

/// Usage example for CDynamixel and CDynamixelROS
class DxlROSExample
{
  protected:
    ros::NodeHandle nh_;   ///< ROS node handle
    CDxlGeneric *motor_;   ///< Motor interface
    CDxlGroup *motors_;    ///< Multiple motor interface
    LxSerial serial_port_; ///< Serial port interface

  public:
    /// Constructor
    DxlROSExample() : nh_("~"), motor_(NULL),motors_() { }

    /// Destructor
    /** Delete motor interface, close serial port, and shut down node handle */
    ~DxlROSExample()
    {
     // if (motor_)
      //  delete motor_;
      if (serial_port_.is_port_open())
        serial_port_.port_close();

      nh_.shutdown();
    }

    /// Initialize node
    void init(char *maxpwm,char *timeStop);

    /// Spin
    /** Alternatively drives the motor clockwise and counterclockwise */
    void spin();
};

#endif
