cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME movement)
project(${PROJECT_NAME})


####################################################
## Installation of Google Test and Mock framework ##
####################################################

# We need thread support
#find_package(Threads REQUIRED)

# Enable ExternalProject CMake module
#include(ExternalProject)

#set(GTEST_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/gtest")

# Download and install GoogleTest
#ExternalProject_Add(GTestExternal 
    # Download version 1.7.0
#    URL https://github.com/google/googletest/archive/release-1.7.0.zip
    # Disable install step
#    INSTALL_COMMAND "")


#Set handy variables
#set(LIBPREFIX "${CMAKE_STATIC_LIBRARY_PREFIX}")
#set(LIBSUFFIX "${CMAKE_STATIC_LIBRARY_SUFFIX}")
#set(GTEST_LOCATION "${GTEST_PREFIX}/src/GTestExternal-build")
#set(GTEST_INCLUDES "${GTEST_PREFIX}/src/GTestExternal/include")
#set(GTEST_LIBRARY  "${GTEST_LOCATION}/${LIBPREFIX}gtest${LIBSUFFIX}")
#set(GTEST_MAINLIB  "${GTEST_LOCATION}/${LIBPREFIX}gtest_main${LIBSUFFIX}")

# Create a libgtest target to be used as a dependency by test programs
#add_library(GTest IMPORTED STATIC GLOBAL)
#add_dependencies(GTest GTestExternal)

# Set gtest properties
#set_target_properties(GTest PROPERTIES
#    IMPORTED_LOCATION                 "${GTEST_LIBRARY}"
#    INTERFACE_INCLUDE_DIRECTORIES     "${GTEST_INCLUDES}"
#    IMPORTED_LINK_INTERFACE_LIBRARIES "${CMAKE_THREAD_LIBS_INIT}")

# Set gtest properties with main included
# Dont know how to use this with catkin though

#add_library(GTestMain IMPORTED STATIC GLOBAL)
#set_target_properties(GTestMain PROPERTIES
#    IMPORTED_LOCATION 		       "${GTEST_MAINLIB}"
#    IMPORTED_LINK_INTERFACE_LIBRARIES  "${GTEST_LIBRARY};${CMAKE_THREAD_LIBS_INIT}")


################################################
## End Google Installation		      ##
################################################





## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED)

find_package(catkin COMPONENTS
  joy
  roscpp
  turtlesim
  geometry_msgs
  REQUIRED
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
 

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES movement
  CATKIN_DEPENDS joy roscpp turtlesim geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(SYSTEM 	 
							${catkin_INCLUDE_DIRS}
)

include_directories(include)
link_directories(${catkin_LIBRARY_DIRS})

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/gamepad_controller.cpp
)
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(movement ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



## Declare a C++ executable
# add_executable(movement_node src/movement_node.cpp)
add_executable(gamepad_controller 
	src/gamepad_controller.cpp
)


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(movement_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(gamepad_controller 
	${catkin_EXPORTED_TARGETS}
)


## Specify libraries to link a library or executable target against
# target_link_libraries(movement_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(gamepad_controller 
	${catkin_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS movement movement_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_movement.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
if(CATKIN_ENABLE_TESTING)

  find_package(rostest)

  #Create gamepad test file
  catkin_add_gtest(gamepad-test 
	test/gamepad_controllerTest.cpp)  
  target_link_libraries(gamepad-test 
	${catkin_LIBRARIES} 
	${PROJECT_NAME}
  )
 

endif()


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
