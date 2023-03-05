/*
* Name: get_azimuth.cpp
* Author: Jan Vlk
* Date: 16.11.2022
* Description: This file contains functions for operations dealing with compass and azimuth.
* Last modified: 28.2.2023
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

/*void get_topic(compass_indices *indices)
{
    std::string param = "publish_";
    std::string reference_str[] = {"utm", "true", "mag"};
    std::string orientation_str[] = {"ned", "enu"};
    std::string data_type_str[] = {"rad", "deg", "quat", "imu", "pose"};

    bool published = false;

    for (int i=0; i<reference_str->length(); ++i){
        for (int j=0; j<orientation_str->length(); ++j){
            for (int k=0; k<data_type_str->length(); ++k){
                param += reference_str[i] + "_azimuth_" + orientation_str[j] + "_" + data_type_str[k];
                ros::param::get(param, published);
                if (published){
                    indices->reference_i = i;
                    indices->orientation_i = j;
                    indices->data_type_i = k;
                    return;
                }
            }
        }
    }
    return;
}*/

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
    // TODO: dynamic function, currently for testing purpose const value
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
