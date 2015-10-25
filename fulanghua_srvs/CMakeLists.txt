cmake_minimum_required(VERSION 2.8.3)
project(fulanghua_srvs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    Pose.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

