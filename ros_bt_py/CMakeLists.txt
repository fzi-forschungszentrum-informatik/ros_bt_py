cmake_minimum_required(VERSION 2.8.3)
project(ros_bt_py)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
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
  scripts/tree_node.py
  scripts/load_tree.py
  scripts/find_best_executor_node.py
  scripts/remote_tree_slot_node.py
  scripts/print_editor_url.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY etc/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
install(DIRECTORY html/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/html/
  )


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

  find_package(rostest REQUIRED)
  add_rostest(test/rostest/async_service.test)
  add_rostest(test/rostest/parallel_if_remote.test)
  add_rostest(test/rostest/ros_leaf_utility.test)
  add_rostest(test/rostest/ros_leaves.test)
  add_rostest(test/rostest/shovable.test)
endif()
