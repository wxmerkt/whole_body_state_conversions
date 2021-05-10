cmake_minimum_required(VERSION 3.1)
project(whole_body_state_conversions)

find_package(catkin REQUIRED COMPONENTS
  whole_body_state_msgs
)

# Ensure version of whole_body_state_msgs is at least 1.0.0
if(NOT ${whole_body_state_msgs_VERSION} VERSION_GREATER "0.9.99")
  message(FATAL_ERROR "whole_body_state_msgs version 1.0.0 or greater required.")
endif()

find_package(pinocchio REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include ${pinocchio_INCLUDE_DIRS}
  CATKIN_DEPENDS whole_body_state_msgs
  DEPENDS pinocchio
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
