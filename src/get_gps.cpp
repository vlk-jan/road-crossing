/*
* Name: misc.cpp
* Author: Jan Vlk
* Date: 13.2.2023
* Description: This file contains miscellaneous functions and classes, or functions and classes that do not have a specific place yet.
* Last modified: 7.3.2023
*/

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "road_crossing/place_data.h"
#include "road_crossing/UTM_data.h"

#include "road_crossing/get_gps.h"

#include <GeographicLib/UTMUPS.hpp>


double easting, northing;
bool is_valid, suitable, new_place = false;

void callback_gps(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    double lat = msg->latitude;
    double lon = msg->longitude;
    ROS_INFO("Current lat, lon: [%f], [%f]", lat, lon);
    double x, y;
    gps_to_utm(lat, lon, x, y);
    easting = x;
    northing = y;
    ROS_INFO("Current easting, northing: [%f], [%f]", easting, northing);
}

void gps_to_utm(double lat, double lon, double &x, double &y)
{
    int zone;
    bool northp;
    GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);
}

void place_suitability()
{
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("place_cost", 5, callback_cost);
    ros::Publisher pub = n.advertise<road_crossing::UTM_data>("UTM_data", 5);
 
    road_crossing::UTM_data msg;

    while (ros::ok()){
        if (new_place){
            msg.easting = easting;
            msg.northing = northing;
            msg.context_score = 0; //TODO: implement context score

            pub.publish(msg);
            new_place = false;
        }

        ros::spinOnce();
    }
}

void callback_cost(const road_crossing::place_data::ConstPtr& msg)
{
    is_valid = msg->is_valid;
    suitable = msg->suitable;
    ROS_INFO("Place valid, suitable: [%d], [%d]", is_valid, suitable);
}

BT::NodeStatus get_position::tick()
{
    setOutput("easting", easting);
    setOutput("northing", northing);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList get_position::providedPorts()
{
    return {BT::OutputPort<double>("easting"), BT::OutputPort<double>("northing")};
}
