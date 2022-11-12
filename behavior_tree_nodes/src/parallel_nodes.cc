#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "compass_msgs/Azimuth.h"
#include "parallel_nodes.hh"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GetAzimuth");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("compass/true/enu/rad", 1000, ReadAzimuth);
    ros::spin();
}

void ReadAzimuth(const compass_msgs::Azimuth::ConstPtr& msg)
{
    double azimuth = msg->azimuth;
    ROS_INFO("Current azimuth: [%d]", azimuth);
}
