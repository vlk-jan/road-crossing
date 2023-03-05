#ifndef get_azimuth
#define get_azimuth

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "compass_msgs/Azimuth.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/PoseStamped.h"


struct compass_indices{
    int reference_i;
    int orientation_i;
    int data_type_i;
};

std::string get_topic();

void get_topic(compass_indices *indices);

void callback_compass(const compass_msgs::Azimuth::ConstPtr& msg);

void callback_quat(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

void callback_imu(const sensor_msgs::Imu::ConstPtr& msg);

void callback_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

class get_required_azimuth : public BT::SyncActionNode
{
    public:
        get_required_azimuth(const std::string& name, const BT::NodeConfiguration& config) :
            BT::SyncActionNode(name, config)
        {}

        virtual ~get_required_azimuth(){}

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts();
};

class get_current_azimuth : public BT::SyncActionNode
{
    public:
        get_current_azimuth(const std::string& name, const BT::NodeConfiguration& config):
            BT::SyncActionNode(name, config)
        {}

        virtual ~get_current_azimuth(){}

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts();
};

class equal_azimuths : public BT::ConditionNode
{
    public:
        equal_azimuths(const std::string& name, const BT::NodeConfiguration& config):
            BT::ConditionNode(name, config)
        {}

        virtual ~equal_azimuths(){}

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts();
};

#endif
