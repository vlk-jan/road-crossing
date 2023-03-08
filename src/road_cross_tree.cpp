#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "compass_msgs/Azimuth.h"
#include "road_crossing/get_azimuth.h"
#include "road_crossing/road_cross_tree.h"
#include "road_crossing/movement.h"
#include "road_crossing/get_gps.h"

Road_cross_tree::Road_cross_tree(std::string filename)
{
    this->factory.registerNodeType<get_required_azimuth>("get_required_azimuth");
    this->factory.registerNodeType<get_current_azimuth>("get_current_azimuth");
    this->factory.registerNodeType<equal_azimuths>("robot_perpendicular");
    this->factory.registerNodeType<rotate_robot>("rotate_robot");
    this->factory.registerNodeType<road_heading>("road_heading");
    this->factory.registerNodeType<compute_heading>("compute_heading");
    this->factory.registerNodeType<cross_road>("cross_road");
    this->factory.registerNodeType<get_position>("get_position");

    this->tree = this->factory.createTreeFromFile(filename);
}

Road_cross_tree::~Road_cross_tree()
{}

int Road_cross_tree::run_tree()
{
    this->tree.tickRoot();
    return EXIT_SUCCESS;
}
