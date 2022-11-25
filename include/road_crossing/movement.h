#ifndef movement
#define movement

#include <cmath>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "road_crossing/movement.h"

#define MAX_ROT_SPEED 1
#define MAX_LIN_SPEED 1.2

class rotate_robot : public BT::SyncActionNode
{
    public:
        rotate_robot(const std::string& name, const BT::NodeConfiguration& config):
            BT::SyncActionNode(name, config)
        {}

        virtual ~rotate_robot(){}

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts();
};

// BT::NodeStatus rotate_robot(double cur_azimuth, double req_azimuth, bool ned, ros::NodeHandle nh);

#endif
