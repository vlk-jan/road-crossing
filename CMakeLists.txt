cmake_minimum_required(VERSION 3.0.2)
project(road_crossing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  compass_msgs
  behaviortree_cpp_v3
  message_generation
)

generate_messages(DEPENDENCIES compass_msgs)

catkin_package()

# TODO: fix compile errors -> undefined references

include_directories(include ${catkin_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

set(SOURCES src/get_azimuth.cpp include/road_crossing/get_azimuth.h)
add_library(azimuth_lib ${SOURCES} ${catkin_LIBRARIES})

set(SOURCES src/road_cross_tree.cpp include/road_crossing/road_cross_tree.h)
add_library(road_cross_tree ${SOURCES} ${catkin_LIBRARIES})

add_executable(get_azimuth src/get_azimuth.cpp)
target_link_libraries(get_azimuth ${catkin_LIBRARIES})
add_dependencies(get_azimuth road_crossing_generate_messages_cpp)

add_executable(run_tree src/main.cpp)
target_link_libraries(run_tree ${catkin_LIBRARIES} road_cross_tree azimuth_lib)
add_dependencies(run_tree get_azimuth)
