#ifndef MOVEMENT
#define MOVEMENT

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"


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
         * @brief Action node to move the robot to a better place where it can cross the road.
         * The place is determined with other nodes.
         * 
         * TODO: implement weight map (as point-cloud) creation & publishing
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
         * @brief Action node to move the robot way from the road. It is used when the robot is unable
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

        /**
         * @brief Condition node that indicates if the robot is not moving.
         */
        class not_started : public BT::ConditionNode
        {
            public:
                not_started(const std::string& name, const BT::NodeConfiguration& config) :
                    BT::ConditionNode(name, config)
                {}

                virtual ~not_started(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

        /**
         * @brief Action node initializing the movement of the robot.
         */
        class start_movement : public BT::SyncActionNode
        {
            public:
                start_movement(const std::string& name, const BT::NodeConfiguration& config):
                    BT::SyncActionNode(name, config)
                {}

                virtual ~start_movement(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

        /**
         * @brief Action node that moves the robot forward with full velocity.
         */
        class move_fwd_full : public BT::SyncActionNode
        {
            public:
               move_fwd_full(const std::string& name, const BT::NodeConfiguration& config):
                    BT::SyncActionNode(name, config)
                {}

                virtual ~move_fwd_full(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

        /**
         * @brief Movement forward. Possible change of velocity.
         */
        class move_fwd : public BT::SyncActionNode
        {
            public:
                move_fwd(const std::string& name, const BT::NodeConfiguration& config):
                    BT::SyncActionNode(name, config)
                {}

                virtual ~move_fwd(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

        /**
         * @brief Movement backward. Possible change of velocity.
         */
        class move_bwd : public BT::SyncActionNode
        {
            public:
                move_bwd(const std::string& name, const BT::NodeConfiguration& config):
                    BT::SyncActionNode(name, config)
                {}

                virtual ~move_bwd(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

        /**
         * @brief Stop the movement of the robot.
         */
        class stop_movement : public BT::SyncActionNode
        {
            public:
                stop_movement(const std::string& name, const BT::NodeConfiguration& config):
                    BT::SyncActionNode(name, config)
                {}

                virtual ~stop_movement(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

        /**
         * @brief Condition node checking if the robot has crossed the road.
         */
        class crossing_finished_gps : public BT::ConditionNode
        {
            public:
                crossing_finished_gps(const std::string& name, const BT::NodeConfiguration& config) :
                    BT::ConditionNode(name, config)
                {}

                virtual ~crossing_finished_gps(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

    private:
        static ros::Publisher pub_cmd, pub_map, pub_finish;
        static ros::ServiceClient get_finish_client;
        static double lin_speed, rot_speed;
        static bool is_moving;
};

ros::Publisher MOV_nodes::pub_cmd;
ros::Publisher MOV_nodes::pub_map;
ros::Publisher MOV_nodes::pub_finish;
ros::ServiceClient MOV_nodes::get_finish_client;
double MOV_nodes::lin_speed = 0;
double MOV_nodes::rot_speed = 0;
bool MOV_nodes::is_moving = false;

#endif
