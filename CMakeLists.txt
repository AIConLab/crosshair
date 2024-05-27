cmake_minimum_required(VERSION 3.0.2)
project(crosshair)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  roslib
)

find_package(OpenCV 4 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs cv_bridge image_transport
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    include
)

add_library(crosshair_lib src/crosshair.cpp)
add_executable(crosshair src/crosshair_node.cpp)

# Correct the linking for OpenCV and add the crosshair_lib
target_link_libraries(crosshair_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})  
target_link_libraries(crosshair crosshair_lib)  