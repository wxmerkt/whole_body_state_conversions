cmake_minimum_required(VERSION 3.1)
project(whole_body_state_conversions)

find_package(catkin REQUIRED COMPONENTS
  whole_body_state_msgs
)

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
