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


/// @brief Indices describing compass data type.
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

        /**
         * @brief Callback function for compass topic subscriber. This is the prefered format.
         * 
         * @param msg Compass data in Azimuth format.
         */
        static void callback_compass(const compass_msgs::Azimuth::ConstPtr& msg);

        /**
         * @brief Callback function for compass topic subscriber.
         * 
         * @param msg Compass data in Quaternion format.
         */
        static void callback_quat(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

        /**
         * @brief Callback function for compass topic subscriber.
         * 
         * @param msg Compass data in Imu format.
         */
        static void callback_imu(const sensor_msgs::Imu::ConstPtr& msg);

        /**
         * @brief Callback function for compass topic subscriber.
         * 
         * @param msg Compass data in Pose format.
         */
        static void callback_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

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

        /**
         * @brief Action node that saves the current robot's azimuth to the blackboard.
         */
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

        /**
         * @brief Condition node checking if the robot is perpendicular to the road. It compares current azimuth with azimuth calculated from the road heading. 
         */
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

        /**
         * @brief Action node that calculates the heading of the road closest to the robot.
         */
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

        /**
         * @brief Action node that calculates the azimuth for robot to be perpendicular to the road.
         */
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
