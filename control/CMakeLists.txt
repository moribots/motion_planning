cmake_minimum_required(VERSION 3.0.2)
project(control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-Wall -Wextra -pipe -Wno-psabi)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  global_planner
  map	
  message_generation
  message_runtime
  rigid2d
  roscpp
  rospy
  rostest
  nuslam
  visualization_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(osqp REQUIRED)

find_package(OsqpEigen REQUIRED)

set(osqp_INCLUDE_DIR /usr/local/include/osqp)
set(OsqpEigen_INCLUDE_DIR /usr/local/include/OsqpEigen)

message(STATUS "OsqpEigen_INCLUDE_DIR : " ${OsqpEigen_INCLUDE_DIR})
message(STATUS "osqp_INCLUDE_DIR : " ${osqp_INCLUDE_DIR})

# message(STATUS "QP : " ${OsqpEigen_FOUND})
# message(STATUS "QP lib: " ${OsqpEigen_LIBRARIES})
# message(STATUS "QP include: " ${OsqpEigen_INCLUDE_DIRS})

# if (OsqpEigenFOUND)
#   message(STATUS "Found OsqpEigen")
# else ()
#   message(ERROR " OsqpEigen not found")
# endif ()


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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
add_message_files(
  FILES
  Waypoint.msg
)

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
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME} global_planner map rigid2d nuslam
CATKIN_DEPENDS global_planner map message_generation message_runtime rigid2d roscpp nuslam visualization_msgs std_msgs
#  DEPENDS system_lib OsqpEigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIR}
  ${osqp_INCLUDE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/rk4.cpp
  src/${PROJECT_NAME}/TrajMPC.cpp
  src/${PROJECT_NAME}/Models.cpp
  src/${PROJECT_NAME}/utilities.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(osqp_example src/osqp_example.cpp)
add_executable(osqp_eigen_example src/osqp_eigen_example.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(osqp_example PROPERTIES OUTPUT_NAME osqp_example PREFIX "")
set_target_properties(osqp_eigen_example PROPERTIES OUTPUT_NAME osqp_eigen_example PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(osqp_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${Eigen3_EXPORTED_TARGETS} ${rigid2d_EXPORTED_TARGETS} ${nuslam_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(osqp_eigen_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${Eigen3_EXPORTED_TARGETS} ${rigid2d_EXPORTED_TARGETS} ${nuslam_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(
osqp_example # this is my node that will use below libraries
${rigid2d_LIBRARIES}
${nuslam_LIBRARIES}
Eigen3::Eigen
osqp::osqp
${PROJECT_NAME}
${catkin_LIBRARIES}
)

target_link_libraries(
osqp_eigen_example # this is my node that will use below libraries
${rigid2d_LIBRARIES}
${nuslam_LIBRARIES}
Eigen3::Eigen
osqp::osqp
OsqpEigen::OsqpEigen
${PROJECT_NAME}
${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  src/mppi
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS osqp_example osqp_eigen_example
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
