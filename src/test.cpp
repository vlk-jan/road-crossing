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

    ros::Subscriber sub_compass = nh.subscribe(compass, 5, callback_compass);
    ros::Subscriber sub_gps = nh.subscribe(gps, 5, callback_gps);

    ros::spin();
}
