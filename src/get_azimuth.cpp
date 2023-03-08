/*
* Name: get_azimuth.cpp
* Author: Jan Vlk
* Date: 16.11.2022
* Description: This file contains functions for operations dealing with compass and azimuth.
* Last modified: 8.3.2023
*/

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "compass_msgs/Azimuth.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "road_crossing/get_azimuth.h"
#include "tf/transform_datatypes.h"
#include "road_crossing/misc.h"


double azimuth;
compass_indices indices;

std::string get_topic()
{
    std::string param = "~publish_";
    std::string reference_str[] = {"utm", "true", "mag"};
    int ref_len = 3;
    std::string orientation_str[] = {"ned", "enu"};
    int ori_len = 2;
    std::string data_type_str[] = {"rad", "deg", "quat", "imu", "pose"};
    int data_len = 5;

    bool published = false;
    ROS_INFO("Searching for topic...");

    for (int i=0; i<ref_len; ++i){
        for (int j=0; j<ori_len; ++j){
            for (int k=0; k<data_len; ++k){
                param += reference_str[i] + "_azimuth_" + orientation_str[j] + "_" + data_type_str[k];
                ros::param::get(param, published);
                if (published){
                    indices.reference_i = i;
                    indices.orientation_i = j;
                    indices.data_type_i = k;
                    return (reference_str[i] + "/" +  orientation_str[j] + "/" + data_type_str[k]);
                }
                param = "~publish_";
            }
        }
    }
    return "";
}

void callback_compass(const compass_msgs::Azimuth::ConstPtr& msg)
{
    // TODO: Is there a better way than global variable?
    if (indices.data_type_i == 0){
        if (indices.orientation_i == 0)
           azimuth = ned2enu(msg->azimuth);
        else
            azimuth = msg->azimuth;
    } else{
        if (indices.orientation_i == 0)
            azimuth = ned2enu(deg2rad(msg->azimuth));
        else
            azimuth = deg2rad(msg->azimuth);
    }
    azimuth = msg->azimuth;
    ROS_INFO("Current azimuth: [%f]", azimuth);
}

void callback_quat(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    if (indices.orientation_i == 0)
        azimuth = ned2enu(tf::getYaw(msg->quaternion));
    else
        azimuth = tf::getYaw(msg->quaternion);
}

void callback_imu(const sensor_msgs::Imu::ConstPtr& msg)
{
    if (indices.orientation_i == 0)
        azimuth = ned2enu(tf::getYaw(msg->orientation));
    else
        azimuth = tf::getYaw(msg->orientation);
}

void callback_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (indices.orientation_i == 0)
        azimuth = ned2enu(tf::getYaw(msg->pose.orientation));
    else
        azimuth = tf::getYaw(msg->pose.orientation);
}

BT::NodeStatus get_required_azimuth::tick()
{
    setOutput("req_azimuth", 3.f);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList get_required_azimuth::providedPorts()
{
    return {BT::OutputPort<double>("req_azimuth")};
}

BT::NodeStatus get_current_azimuth::tick()
{
    setOutput("cur_azimuth", azimuth);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList get_current_azimuth::providedPorts()
{
    return {BT::OutputPort<double>("cur_azimuth")};
}

BT::NodeStatus equal_azimuths::tick()
{
    BT::Optional<double> req_azimuth = getInput<double>("req_azimuth");
    BT::Optional<double> cur_azimuth = getInput<double>("cur_azimuth");
    
    if (!req_azimuth)
        throw BT::RuntimeError("missing required input req_azimuth: ", req_azimuth.error());
    if (!cur_azimuth)
        throw BT::RuntimeError("missing required input cur_azimuth: ", cur_azimuth.error());
    
    if (abs(req_azimuth.value() - cur_azimuth.value()) < 0.3f)
        return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::FAILURE;
}

BT::PortsList equal_azimuths::providedPorts()
{
    return {BT::InputPort<double>("req_azimuth"), BT::InputPort<double>("cur_azimuth")};
}

BT::NodeStatus road_heading::tick()
{
    BT::Optional<double> easting1 = getInput<double>("easting1");
    BT::Optional<double> northing1 = getInput<double>("northing1");
    BT::Optional<double> easting2 = getInput<double>("easting2");
    BT::Optional<double> northing2 = getInput<double>("northing2");

    if (!easting1)
        throw BT::RuntimeError("missing required input easting1: ", easting1.error());
    if (!northing1)
        throw BT::RuntimeError("missing required input northing1: ", northing1.error());
    if (!easting2)
        throw BT::RuntimeError("missing required input easting2: ", easting2.error());
    if (!northing2)
        throw BT::RuntimeError("missing required input northing2: ", northing2.error());

    double heading = gpsPointsHeading(easting1.value(), northing1.value(), easting2.value(), northing2.value());
    setOutput("road_heading", heading);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList road_heading::providedPorts()
{
    return {BT::InputPort<double>("easting1"), BT::InputPort<double>("northing1"), BT::InputPort<double>("easting2"),
            BT::InputPort<double>("northing2"), BT::OutputPort<double>("road_heading")};
}

BT::NodeStatus compute_heading::tick()
{
    BT::Optional<double> rob_heading = getInput<double>("cur_heading");
    BT::Optional<double> road_heading = getInput<double>("road_heading");

    if (!rob_heading)
        throw BT::RuntimeError("missing required input cur_heading: ", rob_heading.error());
    if (!road_heading)
        throw BT::RuntimeError("missing required input road_heading: ", road_heading.error());
    
    double heading = comp_heading(rob_heading.value(), road_heading.value());
    setOutput("req_azimuth", heading);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList compute_heading::providedPorts()
{
    return {BT::InputPort<double>("cur_heading"), BT::InputPort<double>("road_heading"), BT::OutputPort<double>("req_azimuth")};
}
