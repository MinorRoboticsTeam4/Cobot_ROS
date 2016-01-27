[![Stories in Ready](https://badge.waffle.io/MinorRoboticsTeam4/Cobot_ROS.svg?label=ready&title=Ready)](http://waffle.io/MinorRoboticsTeam4/Cobot_ROS)
[![Build Status](https://travis-ci.org/MinorRoboticsTeam4/Cobot_ROS.svg)](https://travis-ci.org/MinorRoboticsTeam4/Cobot_ROS)


For more details about this project, team members and the robot, please visit the [Wiki pages](https://github.com/MinorRoboticsTeam4/CoffeeBot/wiki) 

See the  [Starting Guide for development](https://github.com/MinorRoboticsTeam4/CoffeeBot/wiki)  information. 


## Quick usage guide

### Moving around
In order to move cobot, you need to run the following commands:

```shell
  roslaunch cobot_launch cobot.launch type:='manual'
```

To move Cobot with a gamepad, run:

```shell
  roslaunch cobot_teleop gamepad.launch
```
In order for Cobot (semi) autonomous driving, run the following where <command> can be:

LINE,SPEED LINE, SPIN, SPEED SPIN, LINE AND SPIN, TRIANGLE, SQUARE, PLUS

```shell
  roslaunch cobot_teleop test_driver.launch commands:=[<command_1>,<command_2>...,<command_n>]
```

### Creating a map of the environment

For creating the map, first place the Kinect in the lower compartment of cobot.

Then execute the following commands for startup the mapping:
 
```shell
    roslaunch cobot_launch cobot.launch type:='mapping'
```

```shell
    roslaunch cobot_navigation cobot_gmapping.launch
```

```shell
  roslaunch cobot_teleop gamepad.launch
```

If you want to see how the map looks, run the following:

```shell
    roslaunch cobot_visualise visualise_mapping.launch
```


When you are done and want to save the current map, run (where mapname is the name of the map):
```shell
  rosrun map_server map_saver -f <mapname>
```

This will the map in the current directory. Move these files to the directory "cobot_navigation/maps" using the following command:

```shell
  mv my_map.pgm <location of cobot_navigation/maps>
  mv my_map.yaml <location of cobot_navigation/maps>
```

### Navigation and Ordering coffee

#### Creating list of navigation locations
For creating the map, first attach the Kinect to the table.

Then run the following:

```shell
    roslaunch cobot_launch cobot.launch type:='navigation'
```

mapname is the name of the map to use.

```shell
    roslaunch cobot_navigation cobot_automonous.launch map_file:=<mapname>
```

Start the following for visualising and interacting with navigation, run:

```shell
  roslaunch cobot_visualise visualise_navigation.launch
```

In this screen, select "2d pose estimate" and drag an arrow from the current cobot position to the orientation it faces.

If you want to add a navigation goal, run first:

```shell
  rostopic echo /move_base_simple/goal
```

This will output the location and orientation of the given goal.
You can create a new navigation goal by selecting "2d nav goal" and drag an arrow on the position to travel to. 
The direction is the orientation of cobot.

#### Running the connection between the application and cobot

The navigation goals that are made above must be added to the cobot server.

After that, run the following:

```shell
  roslaunch cobot_api_client cobot_serverConnection.launch 
```

This starts the connection between the application and cobot.

#### Order a coffee

Open up the application and you can order a coffee.







