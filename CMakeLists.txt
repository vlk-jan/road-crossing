cmake_minimum_required(VERSION 3.0.2)
project(road_crossing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  compass_msgs
  geometry_msgs
  std_msgs
  behaviortree_cpp_v3
  message_generation
)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

add_message_files(
  FILES
  injector_msgs.msg
  place_data.msg
  UTM_data.msg
)

add_service_files(
  FILES
  get_road_segment.srv
)

generate_messages(
  DEPENDENCIES 
  compass_msgs 
  geometry_msgs
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/test_injector.py src/road_cost/road_cost.py src/road_cost/get_segment_srv.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(SOURCES src/misc.cpp include/road_crossing/misc.h)
add_library(misc_lib ${catkin_LIBRARIES} ${SOURCES})
target_link_libraries(misc_lib ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})

set(SOURCES src/get_azimuth.cpp include/road_crossing/get_azimuth.h)
add_library(azimuth_lib ${catkin_LIBRARIES} ${SOURCES})
target_link_libraries(azimuth_lib ${catkin_LIBRARIES} misc_lib)

set(SOURCES src/road_cross_tree.cpp include/road_crossing/road_cross_tree.h)
add_library(road_cross_tree_lib ${SOURCES} ${catkin_LIBRARIES})
target_link_libraries(road_cross_tree_lib ${catkin_LIBRARIES} azimuth_lib)

set(SOURCES src/movement.cpp include/road_crossing/movement.h)
add_library(movement_lib ${catkin_LIBRARIES} ${SOURCES})
target_link_libraries(movement_lib ${catkin_LIBRARIES} misc_lib)

set(SOURCES src/get_gps.cpp include/road_crossing/get_gps.h)
add_library(gps_lib ${catkin_LIBRARIES} ${SOURCES})
target_link_libraries(gps_lib ${catkin_LIBRARIES})

add_executable(run_tree src/main.cpp)
target_link_libraries(run_tree ${catkin_LIBRARIES} road_cross_tree_lib azimuth_lib movement_lib gps_lib)

add_executable(vehicles src/vehicles.cpp)
target_link_libraries(vehicles ${catkin_LIBRARIES})

add_executable(test_file src/test.cpp)
target_link_libraries(test_file ${catkin_LIBRARIES} misc_lib gps_lib azimuth_lib)
