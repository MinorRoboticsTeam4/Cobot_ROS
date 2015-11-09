/*
 * gamepad_controller.h
 *
 *      Author: aclangerak
 */


/**
 * This class contains the needed Nodehandler and related code.
 * Also it contains a callback function to convert joy -> twist messages.
 * Lastly, it holds the linear, angular and the scaling factors of
 * the gamepad.
 */
class GamePadTeleop
{
public:
	/**
	 * Default Consctructor
	 */
	GamePadTeleop();

private:
	/**
	 * Callback function to convert joy -> twist messages.
	 */
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  /**
   * NodeHandler for ROS usage
   */
  ros::NodeHandle nh;

  /**
   * holds linear and angular values
   */
  int linear, angular;

  /**
   * scaling factors
   */
  double linear_scale, angular_scale;

  /**
   * Publisher published messages.
   */
  ros::Publisher vel_pub;

  /**
   * Subscriber listens to messages and calls a callback messages.
   */
  ros::Subscriber joy_sub;

};

