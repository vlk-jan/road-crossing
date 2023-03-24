#ifndef MOVEMENT
#define MOVEMENT

#include <cmath>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "road_crossing/movement.h"

#define MAX_ROT_SPEED 1
#define MAX_LIN_SPEED 1.2

class MOV_nodes
{
    public:
        MOV_nodes(){}
        virtual ~MOV_nodes(){}

        /**
         * @brief Init the publishers for functions in this class.
         * 
         * @param nh ROS NodeHandle.
         */
        void init_publishers(ros::NodeHandle& nh);

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

    private:
        static ros::Publisher pub_cmd;
};

ros::Publisher MOV_nodes::pub_cmd;

#endif
