cmake_minimum_required(VERSION 2.8.3)
project(bt_sim_helpers)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  message_generation
)

find_package(roslint REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Door.msg
  Doors.msg
  SimObject.msg
  SimObjects.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetTarget.srv
  InteractObject.srv
  OpenDoor.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_bt_py
  CATKIN_DEPENDS rospy message_generation geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/get_target_node.py
  scripts/sim_doors_node.py
  scripts/sim_pickups_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY etc/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/etc
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


#############
## Testing ##
#############

# Always lint
roslint_python()

if (CATKIN_ENABLE_TESTING)
#  roslint_add_test()
  ## Add gtest based cpp test target and link libraries
  # catkin_add_gtest(${PROJECT_NAME}-test test/test_ros_bt_py.cpp)
  # if(TARGET ${PROJECT_NAME}-test)
  #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  # endif()

  ## Add folders to be run by python nosetests
  # nosetest automatically collects test methods, no need for main
  # methods in your test files!
  # catkin_add_nosetests(test/unittest)

  # find_package(rostest REQUIRED)
  # add_rostest(test/rostest/async_service.test)
  # add_rostest(test/rostest/ros_leaves.test)
  # add_rostest(test/rostest/shovable.test)
endif()
