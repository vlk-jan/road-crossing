/*
* Name: start_srv.cpp
* Author: Jan Vlk
* Date: 27.3.2023
* Description: This file contains functions for starting the BT algorithm
* Last modified: 13.4.2023
*/

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"

#include "road_crossing/start_srv.h"
#include "road_crossing/start_algorithm.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "start_srv");
    ros::NodeHandle nh;

    Start_service cond_nodes;

    const boost::function<bool(road_crossing::start_algorithm::Request&, const road_crossing::start_algorithm::Response&)> start_service =
          boost::bind(&Start_service::start_algorithm, &cond_nodes, _1, _2);

    ros::ServiceServer start_srv = nh.advertiseService("start_algorithm", start_service);

    ros::spin();

    return EXIT_SUCCESS;
}

bool Start_service::start_algorithm(Start_service* node, road_crossing::start_algorithm::Request &req, const road_crossing::start_algorithm::Response &res)
{
    if (req.start && req.stop){
        ROS_WARN("Start and stop algorithm service called");
    } else if (req.start){
        ROS_INFO("Start algorithm service called");
        node->is_running = true;
    } else if (req.stop){
        ROS_INFO("Stop algorithm service called");
        node->is_running = false;
    } else {
        ROS_WARN("Start algorithm service called without any argument");
    }

    return node->is_running;
}

BT::NodeStatus Start_service::start_algorithm::tick()
{
    if (Start_service::is_running)
        return BT::NodeStatus::SUCCESS;
    else
        return BT::NodeStatus::FAILURE;
}

BT::PortsList Start_service::start_algorithm::providedPorts()
{
    return {};
}
