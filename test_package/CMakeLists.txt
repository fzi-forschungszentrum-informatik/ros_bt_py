cmake_minimum_required(VERSION 2.8.3)
project(test_package)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

find_package(roslint REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

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
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   )


## Mark other files for installation (e.g. launch and bag files, etc.)
add_library(something src/test_package/something.cpp)
target_link_libraries(something ${catkin_LIBRARIES})

#############
## Testing ##
#############

# Always lint
roslint_python()

if (CATKIN_ENABLE_TESTING)
  ## Add folders to be run by python nosetests
  # nosetest automatically collects test methods, no need for main
  # methods in your test files!
  catkin_add_nosetests(test/unittest)

  catkin_add_gtest(something_test test/gtest/something_test.cpp)
  target_link_libraries(something_test ${catkin_LIBRARIES} something)
endif()
