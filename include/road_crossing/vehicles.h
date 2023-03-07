#ifndef VEHICLES_H
#define VEHICLES_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"

struct collision_info
{
    int car_id;
    double v_front;
    double v_back;
};

struct vehicle_info
{
    int id;
    double pos_x;
    double pos_y;
    double x_dot;
    double y_dot;
    double length;
    double width;
};

/**
     * @brief Calculate the collision data and determine if with the current speeds the collision will occur.
     *
     * @param vehicle    struct with data about the vehicle.
     * @param robot    struct with data about the robot.
     * @param collision    struct in which the output collision data will be stored.
     */
bool vehicle_collision(vehicle_info vehicle, vehicle_info robot, collision_info &collision);

#endif