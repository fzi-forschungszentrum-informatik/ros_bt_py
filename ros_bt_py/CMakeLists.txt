cmake_minimum_required(VERSION 3.0)
project(ros_bt_py)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++17)

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

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

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
catkin_install_python(PROGRAMS
  scripts/diagnostics_node.py
  scripts/find_best_executor_node.py
  scripts/load_tree.py
  scripts/print_editor_url.py
  scripts/remote_tree_slot_node.py
  scripts/tree_node.py
  scripts/mission_control_node.py
  scripts/capability_repository_node.py
  scripts/assignment_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY etc/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/etc/
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
  # remove generated files
  file(REMOVE_RECURSE test/testdata/save_data/generated_files)
  file(MAKE_DIRECTORY test/testdata/save_data/generated_files)
  ## Add folders to be run by python nosetests
  # nosetest automatically collects test methods, no need for main
  # methods in your test files!
  catkin_add_nosetests(test/unittest)

  find_package(rostest REQUIRED)
  add_rostest(test/rostest/async_service.test ARGS coverage:=ENABLE_COVERAGE_TESTING)
  add_rostest(test/rostest/parallel_if_remote.test ARGS coverage:=ENABLE_COVERAGE_TESTING)
  add_rostest(test/rostest/package_manager.test ARGS coverage:=ENABLE_COVERAGE_TESTING)
  add_rostest(test/rostest/ros_header.test ARGS coverage:=ENABLE_COVERAGE_TESTING)
  add_rostest(test/rostest/ros_leaf_utility.test ARGS coverage:=ENABLE_COVERAGE_TESTING)
  add_rostest(test/rostest/ros_leaves.test ARGS coverage:=ENABLE_COVERAGE_TESTING)
  add_rostest(test/rostest/ros_param.test ARGS coverage:=ENABLE_COVERAGE_TESTING)
  add_rostest(test/rostest/shovable.test ARGS coverage:=ENABLE_COVERAGE_TESTING)
  add_rostest(test/rostest/shove_tree.test ARGS coverage:=ENABLE_COVERAGE_TESTING)
  add_rostest(test/rostest/websocket_interface.test ARGS coverage:=ENABLE_COVERAGE_TESTING)

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*,*/${PROJECT_NAME}/html*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage_report
      DEPENDENCIES tests
    )
  endif()
endif()
