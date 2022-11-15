cmake_minimum_required(VERSION 3.0.2)
project(road_crossing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  compass_msgs
  behaviortree_cpp_v3
  message_generation
)

generate_messages(DEPENDENCIES compass_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(get_azimuth src/get_azimuth.cpp)
target_link_libraries(get_azimuth ${catkin_LIBRARIES})
add_dependencies(get_azimuth road_crossing_generate_messages_cpp)
