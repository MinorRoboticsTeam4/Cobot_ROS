[![Stories in Ready](https://badge.waffle.io/MinorRoboticsTeam4/Cobot_ROS.svg?label=ready&title=Ready)](http://waffle.io/MinorRoboticsTeam4/Cobot_ROS)

For more details about this project, team members and the robot, please visit the [Wiki pages](https://github.com/MinorRoboticsTeam4/CoffeeBot/wiki) 


## Quick usage guide
Coming soon...



## Project Setup using Eclipse

### Project creation

Execute following command on top level directory


```shell
catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles"
```


Next run the following command to pass the current shell environment into the make process in Eclipse.

```shell
awk -f $(rospack find mk)/eclipse.awk build/.project > build/.project_with_env && mv build/.project_with_env build/.project
```

For adding debugging execute the following line

```shell
cmake ../src -DCMAKE_BUILD_TYPE=Debug
```

### Importing

Import the project into Eclipse by selecting "import existing project" 
and navigate to the "build" folder of the catkin workspace.

After importing, right click on the project and select properties -> C/C++ general -> Preprocessor Include Paths, Macros etc. Click the tab "Providers" and check the box next to "CDT GCC Built-in Compiler Settings [ Shared ]".

After that, right click the project, select Index -> Rebuild. This will rebuild your index. Once this is done, all includes wont give errors(hopefully)

### Building

To build the project, first update the CMakelists.txt of the package you are working on. For more information see ...
After that, right click on the project and select "build project". 

### Debugging (and running)

For running the Nodes, it is adviced to create a ros launch file (see [wiki ros](wiki.ros.org/roslaunch) and set the nodes that are needed
to run. 



Information was taken from [ros wiki](wiki.ros.org/IDEs) 
