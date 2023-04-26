/*
* Name: get_azimuth.cpp
* Author: Jan Vlk
* Date: 16.11.2022
* Description: This file contains functions for operations dealing with compass and azimuth.
* Last modified: 26.4.2023
*/

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "compass_msgs/Azimuth.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"

#include "road_crossing/get_azimuth.h"
#include "road_crossing/misc.h"
#include "road_crossing/get_road_segment.h"


std::string AZI_nodes::get_topic()
{
    std::string param = "/magnetometer_compass/publish_";
    std::string reference_str[] = {"utm", "true", "mag"};
    int ref_len = 3;
    std::string orientation_str[] = {"enu", "ned"};
    int ori_len = 2;
    std::string data_type_str[] = {"rad", "deg", "quat", "imu", "pose"};
    int data_len = 5;

    for (int i=0; i<ref_len; ++i){
        for (int j=0; j<ori_len; ++j){
            for (int k=0; k<data_len; ++k){
                param += reference_str[i] + "_azimuth_" + orientation_str[j] + "_" + data_type_str[k];
                if (ros::param::has(param)){
                    AZI_nodes::indices.reference_i = i;
                    AZI_nodes::indices.orientation_i = j;
                    AZI_nodes::indices.data_type_i = k;
                    std::string ret = ("/compass/" + reference_str[i] + "/" +  orientation_str[j] + "/" + data_type_str[k]);
                    return ret;
                }
                param = "/magnetometer_compass/publish_";
            }
        }
    }
    return "";
}

void AZI_nodes::init_service(ros::NodeHandle& nh)
{
    std::string service_name = "get_road_segment";
    ros::service::waitForService(service_name);
    AZI_nodes::client = nh.serviceClient<road_crossing::get_road_segment>(service_name);
}

void AZI_nodes::callback_compass(const compass_msgs::Azimuth::ConstPtr& msg)
{
    if (AZI_nodes::indices.data_type_i == 0){
        if (AZI_nodes::indices.orientation_i == 0)
           AZI_nodes::azimuth = ned_to_enu(msg->azimuth);
        else
            AZI_nodes::azimuth = msg->azimuth;
    } else{
        if (AZI_nodes::indices.orientation_i == 0)
            AZI_nodes::azimuth = ned_to_enu(deg_to_rad(msg->azimuth));
        else
            AZI_nodes::azimuth = deg_to_rad(msg->azimuth);
    }
}

void AZI_nodes::callback_quat(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    if (indices.orientation_i == 0)
        AZI_nodes::azimuth = ned_to_enu(tf::getYaw(msg->quaternion));
    else
        AZI_nodes::azimuth = tf::getYaw(msg->quaternion);
}

void AZI_nodes::callback_imu(const sensor_msgs::Imu::ConstPtr& msg)
{
    if (indices.orientation_i == 0)
        AZI_nodes::azimuth = ned_to_enu(tf::getYaw(msg->orientation));
    else
        AZI_nodes::azimuth = tf::getYaw(msg->orientation);
}

void AZI_nodes::callback_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (indices.orientation_i == 0)
        AZI_nodes::azimuth = ned_to_enu(tf::getYaw(msg->pose.orientation));
    else
        AZI_nodes::azimuth = tf::getYaw(msg->pose.orientation);
}

BT::NodeStatus AZI_nodes::get_required_azimuth::tick()
{
    setOutput("req_azimuth", 3.f);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList AZI_nodes::get_required_azimuth::providedPorts()
{
    return {BT::OutputPort<double>("req_azimuth")};
}

BT::NodeStatus AZI_nodes::get_azimuth::tick()
{
    setOutput("azimuth", AZI_nodes::azimuth);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList AZI_nodes::get_azimuth::providedPorts()
{
    return {BT::OutputPort<double>("azimuth")};
}

BT::NodeStatus AZI_nodes::robot_perpendicular::tick()
{
    BT::Optional<double> req_azimuth = getInput<double>("azimuth");
    BT::Optional<double> cur_azimuth = getInput<double>("heading");
    
    if (!req_azimuth)
        throw BT::RuntimeError("missing required input azimuth: ", req_azimuth.error());
    if (!cur_azimuth)
        throw BT::RuntimeError("missing required input heading: ", cur_azimuth.error());
    
    if (abs(req_azimuth.value() - cur_azimuth.value()) < EQUAL_AZI_LIMIT)
        return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::FAILURE;
}

BT::PortsList AZI_nodes::robot_perpendicular::providedPorts()
{
    return {BT::InputPort<double>("azimuth"), BT::InputPort<double>("heading")};
}

BT::NodeStatus AZI_nodes::road_heading::tick()
{
    BT::Optional<double> easting = getInput<double>("easting");
    BT::Optional<double> northing = getInput<double>("northing");

    if (!easting)
        throw BT::RuntimeError("missing required input easting1: ", easting.error());
    if (!northing)
        throw BT::RuntimeError("missing required input northing1: ", northing.error());

    road_crossing::get_road_segment srv;
    srv.request.easting = easting.value();
    srv.request.northing = northing.value();

    if (AZI_nodes::client.call(srv)){
        double heading = gps_points_heading(srv.response.easting_1, srv.response.northing_1,
                                          srv.response.easting_2, srv.response.northing_2);
        setOutput("road_heading", heading);
        return BT::NodeStatus::SUCCESS;
    } else
        return BT::NodeStatus::FAILURE;
}

BT::PortsList AZI_nodes::road_heading::providedPorts()
{
    return {BT::InputPort<double>("easting"), BT::InputPort<double>("northing"), BT::OutputPort<double>("road_heading")};
}

BT::NodeStatus AZI_nodes::compute_heading::tick()
{
    BT::Optional<double> rob_heading = getInput<double>("rob_azimuth");
    BT::Optional<double> road_heading = getInput<double>("road_heading");

    if (!rob_heading)
        throw BT::RuntimeError("missing required input cur_heading: ", rob_heading.error());
    if (!road_heading)
        throw BT::RuntimeError("missing required input road_heading: ", road_heading.error());
    
    double heading = comp_heading(rob_heading.value(), road_heading.value());
    setOutput("req_azimuth", rob_heading.value()-0.1); //Simulation purposes
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList AZI_nodes::compute_heading::providedPorts()
{
    return {BT::InputPort<double>("rob_azimuth"), BT::InputPort<double>("road_heading"), BT::OutputPort<double>("req_azimuth")};
}
