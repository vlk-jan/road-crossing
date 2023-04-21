#include "ros/ros.h"

#include "sensor_msgs/NavSatFix.h"
#include "compass_msgs/Azimuth.h"

#include "road_crossing/misc.h"
#include "road_crossing/get_gps.h"
#include "road_crossing/get_azimuth.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "road_crossing");
    ros::NodeHandle nh;
    std::string compass = "compass/true/enu/rad/";
    std::string gps = "fix/";

    AZI_nodes azi_nodes;
    GPS_nodes gps_nodes;
    
    const boost::function<void(const compass_msgs::Azimuth::ConstPtr&)> cb_compass = boost::bind(&AZI_nodes::callback_compass, &azi_nodes, _1);
    const boost::function<void(const sensor_msgs::NavSatFix::ConstPtr&)> cb_gps = boost::bind(&GPS_nodes::callback_gps, &gps_nodes, _1);

    ros::Subscriber sub_compass = nh.subscribe(compass, 5, cb_compass);
    ros::Subscriber sub_gps = nh.subscribe(gps, 5, cb_gps);

    ros::spin();
}
