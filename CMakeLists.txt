cmake_minimum_required(VERSION 3.0.2)
project(nav40_driver)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
SET(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
  sensor_msgs
  # sleipnir_msgs
  message_generation
)

add_message_files(
  FILES
  sensorgps.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/nav40_ros_driver.cpp)


 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
