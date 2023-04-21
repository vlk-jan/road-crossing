#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "compass_msgs/Azimuth.h"

#include "road_crossing/get_gps.h"
#include "road_crossing/get_azimuth.h"
#include "road_crossing/road_cross_tree.h"
#include "road_crossing/vehicles.h"
#include "road_crossing_msgs/injector_msgs.h"

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

    std::string compass = "/compass/true/enu/rad";
    std::string gps = "/gps/fix";

    ros::NodeHandle nh;

    AZI_nodes azi_nodes;
    GPS_nodes gps_nodes;
    VEH_nodes veh_nodes;
    
    const boost::function<void(const compass_msgs::Azimuth::ConstPtr&)> cb_compass = boost::bind(&AZI_nodes::callback_compass, &azi_nodes, _1);
    const boost::function<void(const sensor_msgs::NavSatFix::ConstPtr&)> cb_gps = boost::bind(&GPS_nodes::callback_gps, &gps_nodes, _1);
    const boost::function<void(const road_crossing_msgs::injector_msgs::ConstPtr&)> cb_veh = boost::bind(&VEH_nodes::callback_vehicle_injector, &veh_nodes, _1);

    ros::Subscriber sub_compass = nh.subscribe(compass, 5, cb_compass);
    ros::Subscriber sub_gps = nh.subscribe(gps, 5, cb_gps);
    ros::Subscriber sub_veh = nh.subscribe("/injector", 5, cb_veh);

    BT_tree.run_tree();

    ros::spin();

    return EXIT_SUCCESS;
}
