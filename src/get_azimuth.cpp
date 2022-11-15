#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "compass_msgs/Azimuth.h"
#include "road_crossing/get_azimuth.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GetAzimuth");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("compass/true/enu/rad", 1000, get_azimuth_callback);
    ros::spin();
}

void get_azimuth_callback(const compass_msgs::Azimuth::ConstPtr& msg)
{
    double azimuth = msg->azimuth;
    ROS_INFO("Current azimuth: [%f]", azimuth);
}

