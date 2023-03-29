#ifndef MOVEMENT
#define MOVEMENT

#include <cmath>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "road_crossing/movement.h"

#define MAX_ROT_SPEED 1  // rad/s
#define MAX_LIN_SPEED 1.2  // m/s

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

        /**
         * @brief Node to rotate the robot. Main usage is to get robot perpendicular to the road.
         */
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

        /**
         * @brief Node to move the robot to a better place where it can cross the road.
         * The place is determined with other nodes.
         */
        class move_to_place : public BT::SyncActionNode
        {
            public:
                move_to_place(const std::string& name, const BT::NodeConfiguration& config):
                    BT::SyncActionNode(name, config)
                {}

                virtual ~move_to_place(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

        /**
         * @brief Node to move the robot way from the road. It is used when the robot is unable
         * to rotate. We probably need to move the robot away from the road and then rotate it.
         */
        class step_from_road : public BT::SyncActionNode
        {
            public:
                step_from_road(const std::string& name, const BT::NodeConfiguration& config):
                    BT::SyncActionNode(name, config)
                {}

                virtual ~step_from_road(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

    private:
        static ros::Publisher pub_cmd, pub_map;
};

ros::Publisher MOV_nodes::pub_cmd;
ros::Publisher MOV_nodes::pub_map;

#endif
