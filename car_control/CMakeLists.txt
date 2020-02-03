cmake_minimum_required(VERSION 3.0.2)
project(car_control)


find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
geometry_msgs
)



catkin_package(

)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(car_control_node src/car_control_node.cpp)
target_link_libraries(car_control_node ${catkin_LIBRARIES})
#add_dependencies(car_control_node car_control_node_generate_messages_cpp)

FIND_LIBRARY(EXT_LIBRARY pigpio ~/ck_lee/src/pigpio-master)

target_link_libraries(
  car_control_node
  ${catkin_LIBRARIES} ${EXT_LIBRARY}
)