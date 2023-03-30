/*
* Name: movement.cpp
* Author: Jan Vlk
* Date: 25.11.2022
* Description: This file contains functions for moving the robot.
* Last modified: 30.3.2023
*/

#include <cmath>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "road_crossing/movement.h"
#include "road_crossing/get_azimuth.h"
#include "road_crossing/misc.h"


void MOV_nodes::init_publishers(ros::NodeHandle& nh)
{
    this->pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    //this->pub_map = nh.advertise<??>("weight_map", 5);
}

BT::NodeStatus MOV_nodes::rotate_robot::tick()
{
    BT::Optional<double> req_azimuth = getInput<double>("req_azimuth");
    BT::Optional<double> cur_azimuth = getInput<double>("cur_azimuth");

    if (!req_azimuth)
        throw BT::RuntimeError("missing required input req_azimuth: ", req_azimuth.error());
    if (!cur_azimuth)
        throw BT::RuntimeError("missing required input cur_azimuth: ", cur_azimuth.error());

    const double speed_const = MAX_ROT_SPEED/M_PI;
    double speed;

    // ensure that robot rotates maximum of half a circle
    double diff = angle_diff(cur_azimuth.value(), req_azimuth.value());

    if (diff < 0){  // rotation to the right
        speed = diff*speed_const;
    } else {  // rotation to the left
        speed = -1 * diff*speed_const;
    }

    geometry_msgs::Twist msg = geometry_msgs::Twist();
    msg.angular.z = speed;

    MOV_nodes::pub_cmd.publish(msg);
    ros::spin();

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList MOV_nodes::rotate_robot::providedPorts()
{
    return {BT::InputPort<double>("req_azimuth"), BT::InputPort<double>("cur_azimuth"),
            BT::InputPort<ros::NodeHandle>("node_handle")};
}

BT::NodeStatus MOV_nodes::move_to_place::tick()
{
    BT::Optional<double> req_eas = getInput<double>("better_easting");
    BT::Optional<double> req_nor = getInput<double>("better_northing");

    if (!req_eas)
        throw BT::RuntimeError("missing required input better_easting: ", req_eas.error());
    if (!req_nor)
        throw BT::RuntimeError("missing required input better_northing: ", req_nor.error());

    // TODO: implement weight map creation & publishing

    return BT::NodeStatus::FAILURE;
}

BT::PortsList MOV_nodes::move_to_place::providedPorts()
{
    return {BT::InputPort<double>("better_easting"), BT::InputPort<double>("better_northing")};
}
