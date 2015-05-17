cmake_minimum_required(VERSION 2.8.3)
project(motion)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
 roscpp
 # sensor_msgs
 message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


#######################################
## Declare ROS messages and services ##
#######################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  float32List.msg
  # Locomotion_Control
  velocity.msg
  position.msg
  moveConfiguration.msg
  # Cartesian_Control
  path.msg
  # Motion_Task
  task.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  # General
  enable.srv
  isEnabled.srv
  # Stiffness_Control
  stiffnessInterp.srv
  setStiffness.srv
  getStiffness.srv
  # Joint_Control
  angleInterp.srv
  angleInterpSpeed.srv
  setAngles.srv
  changeAngles.srv
  getAngles.srv
  useHand.srv
  # Locomotion_control
  move.srv
  moveTo.srv
  moveToward.srv
  getMoveConfig.srv
  getRobotPosition.srv
  getNextRobotPosition.srv
  getRobotVelocity.srv
  getWalkArmsEnabled.srv
  setWalkArmsEnabled.srv
  # Cartesian_Control
  positionInterpolation.srv
  positionInterpolations.srv
  setPosition.srv
  changePosition.srv
  getPosition.srv
  transformInterpolation.srv
  transformInterpolations.srv
  setTransform.srv
  changeTransform.srv
  getTransform.srv
  # Body_Balancer
  footState.srv
  enableBalanceConstraint.srv
  goToBalance.srv
  enableEffector.srv
  setEffectorControl.srv
  # Motion_Task
  getTaskList.srv
  areResourcesAvailable.srv
  taskResource.srv
  # Robot_Posture
  getPostureList.srv
  setPosture.srv
  getPostureFamily.srv
  setMaxTryNumber.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  # sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES motion
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(motion
#   src/${PROJECT_NAME}/motion.cpp
# )

## Declare a cpp executable
add_executable(motion
  src/motion.cpp
  src/stiffness_control.cpp
  src/joint_control.cpp
  src/locomotion_control.cpp
  src/cartesian_control.cpp
  src/body_balancer.cpp
  src/fall_manager.cpp
  src/motion_task.cpp
  src/robot_posture.cpp)

add_executable(srv_test
  src/srv_test.cpp)

add_executable(joyRemote
 src/joyRemote.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(motion motion_generate_messages_cpp)
add_dependencies(srv_test motion_generate_messages_cpp)
add_dependencies(joyRemote motion_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(motion
  ${catkin_LIBRARIES}
  alcommon
  alerror
  alproxies
  alvalue
)

target_link_libraries(srv_test
  ${catkin_LIBRARIES}
  alcommon
  alerror
  alproxies
  alvalue
)

target_link_libraries(joyRemote
  ${catkin_LIBRARIES}
  alcommon
  alerror
  alproxies
  alvalue
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS motion motion_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_motion.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
