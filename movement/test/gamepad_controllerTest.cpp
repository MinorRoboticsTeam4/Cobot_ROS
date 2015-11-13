#include <gtest/gtest.h>
#include <limits>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include "movement/gamepad_controller.h"

/*
 * Test class
 */
class TwistSubscriber
{
public:
  bool received;
  geometry_msgs::Twist message;

  TwistSubscriber() :
      received(false)
  {
  }

  void callback(const geometry_msgs::Twist &incoming_message)
  {
    this->received = true;
    this->message = incoming_message;
  }

};

TEST(GamepadTest, LoadParam)
{
  movement::GamePad_controller gamepad;

  ros::NodeHandle test_nh;
  int linearAxis;
  test_nh.getParam("gamepad_controller/linear_axis", linearAxis);
  EXPECT_EQ(linearAxis,gamepad.getLinearAxis());

  int angularAxis;
  test_nh.getParam("gamepad_controller/angular_axis", angularAxis);
  EXPECT_EQ(angularAxis,gamepad.getAngularAxis());

  int linearScale;
  test_nh.getParam("gamepad_controller/linear_scale", linearScale);
  EXPECT_EQ(linearScale,gamepad.getLinearScale());

  int angularScale;
  test_nh.getParam("gamepad_controller/angular_scale", angularScale);
  EXPECT_EQ(angularScale,gamepad.getAngularScale());

  std::string pubName;
  test_nh.getParam("gamepad_controller/pub_topic", pubName);
  EXPECT_EQ(pubName,gamepad.getPublishTopic());

}


TEST(GamepadTest, CallbackFunc)
{
  movement::GamePad_controller gamepad;

  ros::NodeHandle test_nh;
  TwistSubscriber subscriber;

  ros::Subscriber rosSubscriber = test_nh.subscribe("test_vel", 100, &TwistSubscriber::callback, &subscriber);

  ASSERT_EQ(1, rosSubscriber.getNumPublishers());

  ros::Publisher rosPublisher = test_nh.advertise<sensor_msgs::Joy>("joy", 100);

  sensor_msgs::Joy joy_msg;
  joy_msg.axes.resize(4);
  joy_msg.axes[gamepad.getLinearAxis()] = 2e+16;
  joy_msg.axes[gamepad.getAngularAxis()] = 1e+16;

  //Prepare ROS for publishing
  rosPublisher.publish(joy_msg);
  sleep(1);
  ros::spinOnce();

  //"Actual" publishing
  rosPublisher.publish(joy_msg);
  sleep(1);
  ros::spinOnce();

  //Test message received and value
  ASSERT_TRUE(subscriber.received);
  EXPECT_FLOAT_EQ(4e+16, subscriber.message.linear.x);
  EXPECT_FLOAT_EQ(2e+16, subscriber.message.angular.z);

}

int main(int argc, char** argv)
{
  //Initialize ros node
  ros::init(argc, argv, "gamepad_teleop_Test");
  ros::NodeHandle test_nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

