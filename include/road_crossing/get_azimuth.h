#ifndef GET_AZIMUTH
#define GET_AZIMUTH

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "compass_msgs/Azimuth.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/PoseStamped.h"


#define EQUAL_AZI_LIMIT 0.1745 // 10 degrees


struct compass_indices{
    int reference_i;
    int orientation_i;
    int data_type_i;
};

class AZI_nodes
{
    public:
        AZI_nodes(){}
        virtual ~AZI_nodes(){}

        /**
         * @brief Find topic the compass data are being published to.
         */
        std::string get_topic();

        //void get_topic(compass_indices& indices);

        /**
         * @brief Initialize service clients for functions in this class.
         * 
         * @param nh ROS NodeHandle.
         */
        void init_service(ros::NodeHandle& nh);

        static void callback_compass(AZI_nodes* node, const compass_msgs::Azimuth::ConstPtr& msg);

        static void callback_quat(AZI_nodes* node, const geometry_msgs::QuaternionStamped::ConstPtr& msg);

        static void callback_imu(AZI_nodes* node, const sensor_msgs::Imu::ConstPtr& msg);

        static void callback_pose(AZI_nodes* node, const geometry_msgs::PoseStamped::ConstPtr& msg);

        /// @brief Testing only, not used in the final version
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

        class get_azimuth : public BT::SyncActionNode
        {
            public:
                get_azimuth(const std::string& name, const BT::NodeConfiguration& config):
                    BT::SyncActionNode(name, config)
                {}

                virtual ~get_azimuth(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

        class robot_perpendicular : public BT::ConditionNode
        {
            public:
                robot_perpendicular(const std::string& name, const BT::NodeConfiguration& config):
                    BT::ConditionNode(name, config)
                {}

                virtual ~robot_perpendicular(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };


        class road_heading : public BT::SyncActionNode
        {
            public:
                road_heading(const std::string& name, const BT::NodeConfiguration& config):
                    BT::SyncActionNode(name, config)
                {}

                virtual ~road_heading(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

        class compute_heading : public BT::SyncActionNode
        {
            public:
                compute_heading(const std::string& name, const BT::NodeConfiguration& config):
                    BT::SyncActionNode(name, config)
                {}

                virtual ~compute_heading(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

    private:
        static double azimuth;
        static compass_indices indices;
        static ros::ServiceClient client;
};

double AZI_nodes::azimuth = 0;
compass_indices AZI_nodes::indices;
ros::ServiceClient AZI_nodes::client;

#endif
