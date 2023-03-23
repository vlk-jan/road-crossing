/*
* Name: misc.cpp
* Author: Jan Vlk
* Date: 13.2.2023
* Description: This file contains miscellaneous functions and classes, or functions and classes that do not have a specific place yet.
* Last modified: 23.3.2023
*/

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "road_crossing/place_data.h"
#include "road_crossing/UTM_data.h"

#include "road_crossing/get_gps.h"
#include "road_crossing/misc.h"


void GPS_nodes::init_publishers(ros::NodeHandle& nh)
{
    this->pubUTM = nh.advertise<road_crossing::UTM_data>("UTM_data", 1000);
}

void GPS_nodes::callback_gps(GPS_nodes* node, const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    double lat = msg->latitude;
    double lon = msg->longitude;
    ROS_INFO("Current lat, lon: [%f], [%f]", lat, lon);
    double x, y;
    gps_to_utm(lat, lon, x, y);
    node->easting = x;
    node->northing = y;
    ROS_INFO("Current easting, northing: [%f], [%f]", node->easting, node->northing);
}

void GPS_nodes::place_suitability()
{
    road_crossing::UTM_data msg;

    while (ros::ok()){
        if (this->new_place){
            msg.easting = this->easting;
            msg.northing = this->northing;
            msg.context_score = 0; //TODO: implement context score

            this->pubUTM.publish(msg);
            this->new_place = false;
        }

        ros::spinOnce();
    }
}

void GPS_nodes::callback_cost(GPS_nodes* node, const road_crossing::place_data::ConstPtr& msg)
{
    node->is_valid = msg->is_valid;
    node->suitable = msg->suitable;
    node->better_easting = msg->better_easting;
    node->better_northing = msg->better_northing;
    ROS_INFO("Place valid, suitable: [%d], [%d]", node->is_valid, node->suitable);
    if (node->suitable)
        ROS_INFO("Better place easting, northing: [%f], [%f]", node->better_easting, node->better_northing);
}

BT::NodeStatus GPS_nodes::cross_road::tick()
{
    if (is_valid)
        return BT::NodeStatus::SUCCESS;
    else
        return BT::NodeStatus::FAILURE;
}

BT::PortsList GPS_nodes::cross_road::providedPorts()
{
    return {};
}

BT::NodeStatus GPS_nodes::place_suitable::tick()
{
    if (suitable)
        return BT::NodeStatus::SUCCESS;
    else
        return BT::NodeStatus::FAILURE;
}

BT::PortsList GPS_nodes::place_suitable::providedPorts()
{
    return {};
}

BT::NodeStatus GPS_nodes::better_place::tick()
{
    if (better_easting != 0 && better_northing != 0){
        setOutput("better_easting", better_easting);
        setOutput("better_northing", better_northing);
        return BT::NodeStatus::SUCCESS;
    } else
        return BT::NodeStatus::FAILURE;
}

BT::PortsList GPS_nodes::better_place::providedPorts()
{
    return {BT::OutputPort<double>("better_easting"), BT::OutputPort<double>("better_northing")};
}

BT::NodeStatus GPS_nodes::get_position::tick()
{
    setOutput("easting", easting);
    setOutput("northing", northing);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList GPS_nodes::get_position::providedPorts()
{
    return {BT::OutputPort<double>("easting"), BT::OutputPort<double>("northing")};
}
