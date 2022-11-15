#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "compass_msgs/Azimuth.h"

int main(int argc, char **argv);

void get_azimuth_callback(const compass_msgs::Azimuth::ConstPtr& msg);
