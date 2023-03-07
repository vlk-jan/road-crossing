#ifndef GET_GPS
#define GET_GPS

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"


/**
 * @brief Subscriber for the gps data of the robot.
 * 
 * @param msg    read message from the gps topic.
*/
void callback_gps(const sensor_msgs::NavSatFix::ConstPtr& msg);

/**
 * @brief Converts gps coordinates to utm coordinates.
 * 
 * @param lat    latitude of the gps coordinates.
 * @param lon    longitude of the gps coordinates.
 * @param x      easting of the utm coordinates.
 * @param y      northing of the utm coordinates.
*/
void gps_to_utm(double lat, double lon, double &x, double &y);

class get_position : public BT::SyncActionNode
{
    public:
        get_position(const std::string& name, const BT::NodeConfiguration& config) :
            BT::SyncActionNode(name, config)
        {}

        virtual ~get_position(){}

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts();
};

#endif
