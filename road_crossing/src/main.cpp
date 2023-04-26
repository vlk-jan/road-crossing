/*
* Name: main.cpp
* Author: Jan Vlk
* Date: 16.11.2022
* Description: Main file of our algorithm, creates ROS subscribers, BT tree and provides.
* Last modified: 26.4.2023
*/

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"

#include "ros/ros.h"
#include "compass_msgs/Azimuth.h"

#include "road_crossing/get_gps.h"
#include "road_crossing/get_azimuth.h"
#include "road_crossing/road_cross_tree.h"
#include "road_crossing/vehicles.h"
#include "road_crossing_msgs/injector_msgs.h"
#include "road_crossing_msgs/start_msgs.h"
#include "road_crossing/start_srv.h"
#include "road_crossing/movement.h"
#include "road_crossing/misc.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "road_crosser");
    std::string tree_file;
    if (ros::param::has("/road_crossing/tree_file")){
        ros::param::get("/road_crossing/tree_file", tree_file);
        ROS_INFO("Tree file loaded");
    } else {
        ROS_ERROR("Missing tree file");
        return EXIT_FAILURE;
    }
    
    Road_cross_tree BT_tree(tree_file);

    std::string compass;
    std::string gps = "/gps/fix";
    std::string vehicles = "/road_crossing/injector";
    std::string start = "/road_crossing/start";

    ros::NodeHandle nh;

    AZI_nodes azi_nodes;
    GPS_nodes gps_nodes;
    VEH_nodes veh_nodes;
    MOV_nodes mov_nodes;
    Start_service start_serv;

    compass = azi_nodes.get_topic();
    
    const boost::function<void(const compass_msgs::Azimuth::ConstPtr&)> cb_compass =
                            boost::bind(&AZI_nodes::callback_compass, _1);
    const boost::function<void(const sensor_msgs::NavSatFix::ConstPtr&)> cb_gps =
                            boost::bind(&GPS_nodes::callback_gps, _1);
    const boost::function<void(const road_crossing_msgs::injector_msgs::ConstPtr&)> cb_veh =
                            boost::bind(&VEH_nodes::callback_vehicle_injector, _1);
    const boost::function<void(const road_crossing_msgs::start_msgs::ConstPtr&)> cb_start = 
                            boost::bind(&Start_service::start_callback, _1);

    ros::Subscriber sub_compass = nh.subscribe(compass, 5, cb_compass);
    ros::Subscriber sub_gps = nh.subscribe(gps, 5, cb_gps);
    ros::Subscriber sub_veh = nh.subscribe(vehicles, 5, cb_veh);
    ros::Subscriber sub_start = nh.subscribe(start, 5, cb_start);

    gps_nodes.init_service(nh);
    azi_nodes.init_service(nh);
    mov_nodes.init_publishers(nh);
    start_serv.init_publishers(nh);
    init_service(nh);

    BT::FileLogger logger_file(BT_tree.tree, "./bt_trace.fbl");

    while (ros::ok()){
        BT_tree.run_tree();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
