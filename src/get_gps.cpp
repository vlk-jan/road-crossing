/*
* Name: misc.cpp
* Author: Jan Vlk
* Date: 13.2.2023
* Description: This file contains miscellaneous functions and classes, or functions and classes that do not have a specific place yet.
* Last modified: 5.3.2023
*/

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

#include "road_crossing/get_gps.h"


double lat, lon;

void callback_gps(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    lat = msg->latitude;
    lon = msg->longitude;
    ROS_INFO("Current lat, lon: [%f], [%f]", lat, lon);
}

BT::NodeStatus get_position::tick()
{
    setOutput("lattitude", lat);
    setOutput("longitude", lon);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList get_position::providedPorts()
{
    return {BT::OutputPort<double>("lattitude"), BT::OutputPort<double>("longitude")};
}
