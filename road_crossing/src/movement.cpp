/*
* Name: movement.cpp
* Author: Jan Vlk
* Date: 25.11.2022
* Description: This file contains functions for moving the robot.
* Last modified: 11.5.2023
*/

#include <cmath>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "road_crossing/movement.h"
#include "road_crossing/get_azimuth.h"
#include "road_crossing/misc.h"
#include "road_crossing/get_gps.h"
#include "road_crossing/get_finish.h"
#include "road_crossing_msgs/start_msgs.h"
#include "road_crossing/vehicles.h"


void MOV_nodes::init_publishers(ros::NodeHandle& nh)
{
    MOV_nodes::pub_cmd = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    //MOV_nodes::pub_map = nh.advertise<??>("weight_map", 5);
    MOV_nodes::pub_finish = nh.advertise<road_crossing_msgs::start_msgs>("/road_crossing/start", 5);
    std::string service_name = "get_finish";
    ros::service::waitForService(service_name);
    MOV_nodes::get_finish_client = nh.serviceClient<road_crossing::get_finish>(service_name);
}

BT::NodeStatus MOV_nodes::rotate_robot::tick()
{
    BT::Optional<double> cur_azimuth = getInput<double>("azimuth");
    BT::Optional<double> req_azimuth = getInput<double>("heading");

    if (!req_azimuth)
        throw BT::RuntimeError("missing required input req_azimuth: ", req_azimuth.error());
    if (!cur_azimuth)
        throw BT::RuntimeError("missing required input cur_azimuth: ", cur_azimuth.error());

    const double speed_const = VEL_info::get_max_rot_vel()/M_PI;
    double speed;

    // ensure that robot rotates maximum of half a circle
    double diff = angle_diff(cur_azimuth.value(), req_azimuth.value());

    if (diff > 0){  // rotation to the right
        speed = diff*speed_const;
    } else {  // rotation to the left
        speed = -1 * diff*speed_const;
    }

    geometry_msgs::Twist msg = geometry_msgs::Twist();
    msg.angular.z = speed;
    //ROS_INFO("Cur azimuth: %f, req azimuth: %f", cur_azimuth.value(), req_azimuth.value());
    //ROS_INFO("Angle diff: %f, speed: %f", diff, speed);

    MOV_nodes::pub_cmd.publish(msg);
    ros::spinOnce();

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList MOV_nodes::rotate_robot::providedPorts()
{
    return {BT::InputPort<double>("heading"), BT::InputPort<double>("azimuth")};
}

BT::NodeStatus MOV_nodes::move_to_place::tick()
{
    BT::Optional<double> req_eas = getInput<double>("better_easting");
    BT::Optional<double> req_nor = getInput<double>("better_northing");

    if (!req_eas)
        throw BT::RuntimeError("missing required input better_easting: ", req_eas.error());
    if (!req_nor)
        throw BT::RuntimeError("missing required input better_northing: ", req_nor.error());

    return BT::NodeStatus::FAILURE;
}

BT::PortsList MOV_nodes::move_to_place::providedPorts()
{
    return {BT::InputPort<double>("better_easting"), BT::InputPort<double>("better_northing")};
}

BT::NodeStatus MOV_nodes::step_from_road::tick()
{
    BT::Optional<double> rob_azi = getInput<double>("azimuth");
    BT::Optional<double> road_azi = getInput<double>("road_heading");

    if (!rob_azi)
        throw BT::RuntimeError("missing required input azimuth: ", rob_azi.error());
    if (!road_azi)
        throw BT::RuntimeError("missing required input road_heading: ", road_azi.error());

    double diff = angle_diff(rob_azi.value(), road_azi.value());
    if (diff < 0.3){
        ROS_WARN("Unable to safely step backwards");
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::Twist msg = geometry_msgs::Twist();
    msg.linear.x = VEL_info::get_max_lin_vel()/2;

    MOV_nodes::pub_cmd.publish(msg);
    ros::spinOnce();    

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList MOV_nodes::step_from_road::providedPorts()
{
    return {BT::InputPort<double>("azimuth"), BT::InputPort<double>("road_heading")};
}

BT::NodeStatus MOV_nodes::not_started::tick()
{
    if (!MOV_nodes::is_moving)
        return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::FAILURE;
}

BT::PortsList MOV_nodes::not_started::providedPorts()
{
    return {};
}

BT::NodeStatus MOV_nodes::start_movement::tick()
{
    MOV_nodes::is_moving = true;
    MOV_nodes::lin_speed = VEL_info::get_max_lin_vel();
    ROS_INFO("Movement started.");
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList MOV_nodes::start_movement::providedPorts()
{
    return {};
}

BT::NodeStatus MOV_nodes::move_fwd_full::tick()
{
    geometry_msgs::Twist msg = geometry_msgs::Twist();
    msg.linear.x = VEL_info::get_max_lin_vel();
    VEH_nodes::set_robot_vel(msg.linear.x, msg.linear.y);
    MOV_nodes::lin_speed = msg.linear.x;

    MOV_nodes::pub_cmd.publish(msg);
    ros::spinOnce();

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList MOV_nodes::move_fwd_full::providedPorts()
{
    return {};
}

BT::NodeStatus MOV_nodes::move_fwd::tick()
{
    BT::Optional<double> max_vel_fwd = getInput<double>("max_vel_fwd");
    BT::Optional<double> min_vel_fwd = getInput<double>("min_vel_fwd");

    if (!max_vel_fwd)
        throw BT::RuntimeError("missing required input max_vel_fwd: ", max_vel_fwd.error());
    if (!min_vel_fwd)
        throw BT::RuntimeError("missing required input min_vel_fwd: ", min_vel_fwd.error());

    if (max_vel_fwd.value() - min_vel_fwd.value() > 2*VEL_info::get_vel_margin()){
        MOV_nodes::lin_speed = max_vel_fwd.value() - VEL_info::get_vel_margin();
    } else {  // Should not happen
        MOV_nodes::lin_speed = 0;
        ROS_WARN("Movement forward requested, but no speed is possible.");
    }

    geometry_msgs::Twist msg = geometry_msgs::Twist();
    msg.linear.x = MOV_nodes::lin_speed;
    ROS_INFO("Moving forward with speed %f.", MOV_nodes::lin_speed);
    VEH_nodes::set_robot_vel(msg.linear.x, msg.linear.y);

    MOV_nodes::pub_cmd.publish(msg);
    ros::spinOnce();

    if (MOV_nodes::lin_speed == 0)
        return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList MOV_nodes::move_fwd::providedPorts()
{
    return {BT::InputPort<double>("max_vel_fwd"), BT::InputPort<double>("min_vel_fwd")};
}

BT::NodeStatus MOV_nodes::move_bwd::tick()
{
    BT::Optional<double> max_vel_bwd = getInput<double>("max_vel_bwd");
    BT::Optional<double> min_vel_bwd = getInput<double>("min_vel_bwd");

    if (!max_vel_bwd)
        throw BT::RuntimeError("missing required input max_vel_bwd: ", max_vel_bwd.error());
    if (!min_vel_bwd)
        throw BT::RuntimeError("missing required input min_vel_bwd: ", min_vel_bwd.error());

    if (max_vel_bwd.value() - min_vel_bwd.value() > 2*VEL_info::get_vel_margin()){
        MOV_nodes::lin_speed = max_vel_bwd.value() + VEL_info::get_vel_margin();
    } else {  // Should not happen
        MOV_nodes::lin_speed = 0;
        ROS_WARN("Movement backward requested, but no speed is possible.");
    }

    geometry_msgs::Twist msg = geometry_msgs::Twist();
    msg.linear.x = MOV_nodes::lin_speed;
    ROS_INFO("Moving backward with speed %f", MOV_nodes::lin_speed);
    VEH_nodes::set_robot_vel(msg.linear.x, msg.linear.y);

    MOV_nodes::pub_cmd.publish(msg);
    ros::spinOnce();

    if (MOV_nodes::lin_speed == 0)
        return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList MOV_nodes::move_bwd::providedPorts()
{
    return {BT::InputPort<double>("max_vel_bwd"), BT::InputPort<double>("min_vel_bwd")};
}

BT::NodeStatus MOV_nodes::stop_movement::tick()
{
    MOV_nodes::lin_speed = 0;
    ROS_INFO("Movement stopped.");

    geometry_msgs::Twist msg = geometry_msgs::Twist();
    msg.linear.x = MOV_nodes::lin_speed;
    MOV_nodes::pub_cmd.publish(msg);
    ros::spinOnce();
    VEH_nodes::set_robot_vel(msg.linear.x, msg.linear.y);

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList MOV_nodes::stop_movement::providedPorts()
{
    return {};
}

BT::NodeStatus MOV_nodes::crossing_finished_gps::tick()
{
    double easting, northing;
    GPS_nodes::req_position(easting, northing);
    road_crossing::get_finish finish_srv;

    finish_srv.request.easting = easting;
    finish_srv.request.northing = northing;

    if (MOV_nodes::get_finish_client.call(finish_srv)){
        if (finish_srv.response.finish){
            ROS_INFO("Crossing finished.");
            road_crossing_msgs::start_msgs msg;
            msg.start = false;
            msg.valid = true;
            MOV_nodes::pub_finish.publish(msg);
            return BT::NodeStatus::SUCCESS;
        }
    }

    return BT::NodeStatus::FAILURE;
}

BT::PortsList MOV_nodes::crossing_finished_gps::providedPorts()
{
    return {};
}
