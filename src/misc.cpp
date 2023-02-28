/*
* Name: misc.cpp
* Author: Jan Vlk
* Date: 13.2.2023
* Description: This file contains miscellaneous functions and classes, or functions and classes that do not have a specific place yet.
* Last modified: 28.2.2023
*/

#include "road_crossing/misc.h"

#include <cmath>

#include "ros/ros.h"


double gpsPointsHeading(double lat1, double lon1, double lat2, double lon2)
{
    double dLon = lon2 - lon1;
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    double brng = atan2(y, x);
    brng = brng * 180 / M_PI;
    brng = std::fmod((brng + 360), 360);
    return brng;
}

double angleDifference(double angle1, double angle2)
{
    double diff = std::fmod(angle2 - angle1 + M_PI, 2*M_PI) - M_PI;
    return diff < -M_PI ? diff + 2*M_PI : diff;
}

double ned2enu(double angle)
{
    if (angle > 0 && angle < M_PI/2)
        return M_PI/2 - angle;
    else
        return M_PI*5/2 - angle;
}

double deg2rad(double angle)
{
    return angle * M_PI / 180;
}
