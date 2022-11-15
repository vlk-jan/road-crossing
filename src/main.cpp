#include <pthread.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "compass_msgs/Azimuth.h"
#include "road_crossing/get_azimuth.h"
#include "road_crossing/road_cross_tree.h"

int main(int argc, char **argv)
{
    Road_cross_tree BT_tree("test_tree.xml");

    BT_tree.tree.tickRoot();

    return EXIT_SUCCESS;
}
