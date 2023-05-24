/*
* Name: start_srv.cpp
* Author: Jan Vlk
* Date: 27.3.2023
* Description: This file contains functions for starting the BT algorithm
* Last modified: 19.5.2023
*/

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"

#include "road_crossing/start_srv.h"
#include "road_crossing/start_algorithm.h"
#include "road_crossing_msgs/start_msgs.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "start_srv");
    ros::NodeHandle nh;

    Start_service::init_publishers(nh);

    const boost::function<bool(road_crossing::start_algorithm::Request&, road_crossing::start_algorithm::Response&)> start_service =
          boost::bind(&Start_service::start_algorithm_service, _1, _2);

    ros::ServiceServer start_srv = nh.advertiseService("start_algorithm", start_service);

    ros::spin();

    return EXIT_SUCCESS;
}

void Start_service::init_publishers(ros::NodeHandle& nh)
{
    Start_service::start_pub = nh.advertise<road_crossing_msgs::start_msgs>("/road_crossing/start", 5);
}

bool Start_service::start_algorithm_service(road_crossing::start_algorithm::Request &req, road_crossing::start_algorithm::Response &res)
{
    bool valid = false;
    bool running = false;
    if (req.start && req.stop){
        ROS_WARN("Start and stop of algorithm requested simultaneously");
    } else if (req.start){
        ROS_INFO("Start algorithm service called");
        running = true;
        valid = true;
    } else if (req.stop){
        ROS_INFO("Stop algorithm service called");
        valid = true;
    } else {
        ROS_WARN("Start algorithm service called without any argument");
    }

    if (valid){
        res.is_running = running;
        road_crossing_msgs::start_msgs msg;
        msg.valid = true;
        msg.start = running;
        Start_service::start_pub.publish(msg);
        return true;
    } else
        return false;
}

void Start_service::start_callback(const road_crossing_msgs::start_msgs::ConstPtr& msg)
{
    ROS_INFO("Start callback called");
    if (msg->valid){
        Start_service::is_running = msg->start;
    }
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
