/*
* Name: misc.cpp
* Author: Jan Vlk
* Date: 13.2.2023
* Description: This file contains miscellaneous functions and classes, or functions and classes that do not have a specific place yet.
* Last modified: 19.5.2023
*/

#include "road_crossing/misc.h"

#include <cmath>

#include "ros/ros.h"

#include <GeographicLib/UTMUPS.hpp>

#include "road_crossing/get_road_info.h"


double gps_points_heading(double lat1, double lon1, double lat2, double lon2)
{
    // https://stackoverflow.com/a/18738281
    double dLon = lon2 - lon1;
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    double brng = atan2(y, x);
    brng = std::fmod((brng + 2*M_PI), 2*M_PI);
    brng = ned_to_enu(brng);
    return brng;
}

double angle_diff(double angle1, double angle2)
{
    // https://stackoverflow.com/a/28037434
    double diff = std::fmod(angle2 - angle1 + M_PI, 2*M_PI) - M_PI;
    return diff < -M_PI ? diff + 2*M_PI : diff;
}

double ned_to_enu(double azimuth)
{
    if (azimuth > 0 && azimuth < M_PI/2)
        return M_PI/2 - azimuth;
    else
        return M_PI*5/2 - azimuth;
}

double deg_to_rad(double angle)
{
    return angle * M_PI / 180;
}

double comp_heading(double rob_heading, double road_heading)
{
    double heading1 = road_heading + M_PI/2;
    heading1 = heading1 < 0 ? 2*M_PI + heading1 : heading1;
    double heading2 = road_heading - M_PI/2;
    heading2 = heading2 < 0 ? 2*M_PI + heading2 : heading2;
    double diff1 = angle_diff(rob_heading, heading1);
    double diff2 = angle_diff(rob_heading, heading2);
    if (std::abs(diff1) < std::abs(diff2))
        return heading1;
    else
        return heading2;
}

void gps_to_utm(double lat, double lon, double &x, double &y)
{
    int zone;
    bool northp;
    GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);
}

int calculate_context_score(float easting, float northing)
{
    int score = 0;
    road_crossing::get_road_info srv;
    srv.request.easting = easting;
    srv.request.northing = northing;

    if (!road_info_client.call(srv)){
        ROS_ERROR("Failed to call service get_road_info");
        return score;
    }

    double road_width = srv.response.road_width;
    if (road_width <= 3.5)
        score += 4;
    else if (road_width <= 4.5)
        score += 3;
    else if (road_width <= 5.5)
        score += 2;
    else if (road_width <= 6.5)
        score += 1;

    double max_velocity = srv.response.maximal_velocity;
    if (max_velocity <= 30)
        score += 3;
    else if (max_velocity <= 50)
        score += 2;
    else if (max_velocity <= 80)
        score += 1;

    std::string road_type = srv.response.road_type;
    if (road_type == "motorway")
        score -= 50;
    else if (road_type == "trunk")
        score -= 50;
    else if (road_type == "primary")
        score += 1;
    else if (road_type == "secondary")
        score += 2;
    else if (road_type == "tertiary")
        score += 3;

    int num_lanes = srv.response.num_lanes;
    if (num_lanes == 1)
        score += 5;
    else if (num_lanes == 2)
        score += 4;
    else if (num_lanes == 3)
        score += 2;
    else if (num_lanes == 4)
        score += 1;

    bool pedestrian_crossing = srv.response.pedestrian_crossing;
    if (pedestrian_crossing)
        score += 100;

    return score;
}

void init_service(ros::NodeHandle &nh)
{
    std::string service_name = "get_road_info";
    ros::service::waitForService(service_name);
    road_info_client = nh.serviceClient<road_crossing::get_road_info>(service_name);
}
