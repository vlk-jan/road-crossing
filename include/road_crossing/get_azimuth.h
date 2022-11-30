#ifndef get_azimuth
#define get_azimuth

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "compass_msgs/Azimuth.h"

struct compass_indices{
    int reference_i;
    int orientation_i;
    int data_type_i;
};

int main(int argc, char **argv);

std::string get_topic();

void get_topic(compass_indices *indices);

void get_azimuth_callback(const compass_msgs::Azimuth::ConstPtr& msg);

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
