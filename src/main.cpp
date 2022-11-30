#include <pthread.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "compass_msgs/Azimuth.h"
#include "road_crossing/road_cross_tree.h"

int main(int argc, char **argv)
{
    std::string tree_file;
    ros::param::get("tree_file", tree_file);
    Road_cross_tree BT_tree(tree_file);

    BT_tree.run_tree();

    return EXIT_SUCCESS;
}
