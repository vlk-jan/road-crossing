/*
* Name: misc.cpp
* Author: Jan Vlk
* Date: 13.2.2023
* Description: This file contains miscellaneous functions and classes, or functions and classes that do not have a specific place yet.
* Last modified: 22.4.2023
*/

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

#include "road_crossing/get_gps.h"
#include "road_crossing/misc.h"
#include "road_crossing/get_suitability.h"


void GPS_nodes::init_service(ros::NodeHandle& nh)
{
    std::string service_name = "get_suitability";
    ros::service::waitForService(service_name);
    GPS_nodes::place_suitability_client = nh.serviceClient<road_crossing::get_suitability>(service_name);
}

void GPS_nodes::callback_gps(GPS_nodes* node, const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    double lat = msg->latitude;
    double lon = msg->longitude;
    //ROS_INFO("Current lat, lon: [%f], [%f]", lat, lon);
    double x, y;
    gps_to_utm(lat, lon, x, y);
    node->easting = x;
    node->northing = y;
    //ROS_INFO("Current easting, northing: [%f], [%f]", node->easting, node->northing);
}

int GPS_nodes::place_suitability()
{
    if (GPS_nodes::easting == 0 && GPS_nodes::northing == 0){
        ROS_WARN("GPS was not received -> no position set");
        return EXIT_FAILURE;
    }
    road_crossing::get_suitability srv;
    srv.request.easting = GPS_nodes::easting;
    srv.request.northing = GPS_nodes::northing;
    srv.request.context_score = calculate_context_score(GPS_nodes::easting, GPS_nodes::northing);

    if (GPS_nodes::place_suitability_client.call(srv)){
        GPS_nodes::suitable = srv.response.suitable;
        GPS_nodes::is_valid = srv.response.valid;
    } else {
        ROS_ERROR("Failed to call service get_suitability");
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

void GPS_nodes::req_position(double& easting, double& northing)
{
    easting = GPS_nodes::easting;
    northing = GPS_nodes::northing;
}

BT::NodeStatus GPS_nodes::cross_road::tick()
{
    if (GPS_nodes::is_valid)
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
    if (GPS_nodes::suitable)
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
    if (GPS_nodes::new_place){
        return BT::NodeStatus::SUCCESS;
    } else
        return BT::NodeStatus::FAILURE;
}

BT::PortsList GPS_nodes::better_place::providedPorts()
{
    return {};
}

BT::NodeStatus GPS_nodes::get_position::tick()
{
    setOutput("easting", GPS_nodes::easting);
    setOutput("northing", GPS_nodes::northing);
    if (!GPS_nodes::place_suitability())
        return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::FAILURE;
}

BT::PortsList GPS_nodes::get_position::providedPorts()
{
    return {BT::OutputPort<double>("easting"), BT::OutputPort<double>("northing")};
}

BT::NodeStatus GPS_nodes::get_better_place::tick()
{
    // TODO: implement service for obtaining better place
    // until than -- FAILURE -> continue in current place
    setOutput("better_easting", 0.0);
    setOutput("better_northing", 0.0);
    setOutput("new_place", false);
    return BT::NodeStatus::FAILURE;
}

BT::PortsList GPS_nodes::get_better_place::providedPorts()
{
    return {BT::OutputPort<double>("better_easting"), BT::OutputPort<double>("better_northing"),
            BT::OutputPort<bool>("new_place")};
}
