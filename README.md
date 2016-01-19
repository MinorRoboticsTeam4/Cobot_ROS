[![Stories in Ready](https://badge.waffle.io/MinorRoboticsTeam4/Cobot_ROS.svg?label=ready&title=Ready)](http://waffle.io/MinorRoboticsTeam4/Cobot_ROS)
[![Build Status](https://travis-ci.org/MinorRoboticsTeam4/Cobot_ROS.svg)](https://travis-ci.org/MinorRoboticsTeam4/Cobot_ROS)


For more details about this project, team members and the robot, please visit the [Wiki pages](https://github.com/MinorRoboticsTeam4/CoffeeBot/wiki) 

See the  [Starting Guide for development](https://github.com/MinorRoboticsTeam4/CoffeeBot/wiki)  information. 


## Quick usage guide

### Moving around
Something about what launch file(s) to use

<bash>roslaunch cobot_launch cobot.launch</bash>

gamepad:
<bash>roslaunch cobot_teleop gamepad.launch</bash>

(semi) autonomous driving
<bash>roslaunch cobot_teleop test_driver.launch commands:=[<command_1>,<command_2>...,<command_n>]</bash>
Available commands:
LINE,SPEED LINE, SPIN, SPEED SPIN, LINE AND SPIN, TRIANGLE, SQUARE, PLUS
Full description of the commands is found in ???

### Creating a map of the environment
Something with Kinect placement
Something about what launch file(s) to use

### Creating list of navigation locations
Something about what launch file(s) to use





