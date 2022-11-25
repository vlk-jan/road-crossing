#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "compass_msgs/Azimuth.h"
#include "road_crossing/get_azimuth.h"
#include "road_crossing/road_cross_tree.h"
#include "road_crossing/movement.h"

Road_cross_tree::Road_cross_tree(char *filename)
{
    this->factory.registerNodeType<get_required_azimuth>("get_required_azimuth");
    this->factory.registerNodeType<get_current_azimuth>("get_current_azimuth");
    this->factory.registerNodeType<equal_azimuths>("equal_azimuths");
    this->factory.registerNodeType<rotate_robot>("rotate_robot");

    this->tree = this->factory.createTreeFromFile(filename);
}

Road_cross_tree::~Road_cross_tree()
{}

int Road_cross_tree::run_tree()
{
    this->tree.tickRoot();
    return EXIT_SUCCESS;
}
