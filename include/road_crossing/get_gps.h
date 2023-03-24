#ifndef GET_GPS
#define GET_GPS

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "road_crossing/place_data.h"


class GPS_nodes
{
    public:
        GPS_nodes(){}
        virtual ~GPS_nodes(){}

        /**
         * @brief Init the publishers for funcions in this class.
         * 
         * @param nh ROS NodeHandle.
         */
        void init_publishers(ros::NodeHandle& nh);

        /**
         * @brief Subscriber for the gps data of the robot.
         * 
         * @param node GPS_nodes object, necessary for static variables.
         * @param msg Read message from the gps topic.
         */
        static void callback_gps(GPS_nodes* node, const sensor_msgs::NavSatFix::ConstPtr& msg);

        /**
         * @brief Subscriber for the cost data, calculated by road cost.
         * 
         * @param node GPS_nodes object, necessary for static variables.
         * @param msg Read message from the cost topic.
         */
        static void callback_cost(GPS_nodes* node, const road_crossing::place_data::ConstPtr& msg);

        /**
         * @brief Request information from road cost node, if the current robot's position is suitable for road crossing.
         * 
         * TODO: rewrite as service
         */
        void place_suitability();

        class cross_road : public BT::ConditionNode
        {
            public:
                cross_road(const std::string& name, const BT::NodeConfiguration& config) :
                    BT::ConditionNode(name, config)
                {}

                virtual ~cross_road(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

        class place_suitable : public BT::ConditionNode
        {
            public:
                place_suitable(const std::string& name, const BT::NodeConfiguration& config) :
                    BT::ConditionNode(name, config)
                {}

                virtual ~place_suitable(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

        class better_place : public BT::ConditionNode
        {
            public:
                better_place(const std::string& name, const BT::NodeConfiguration& config) :
                    BT::ConditionNode(name, config)
                {}

                virtual ~better_place(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

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
    
    private:
        static ros::Publisher pubUTM;
        static double easting, northing;
        static double better_easting, better_northing;
        static bool is_valid, suitable, new_place;
};

ros::Publisher GPS_nodes::pubUTM;
double GPS_nodes::easting = 0;
double GPS_nodes::northing = 0;
double GPS_nodes::better_easting = 0;
double GPS_nodes::better_northing = 0;
bool GPS_nodes::is_valid = false;
bool GPS_nodes::suitable;
bool GPS_nodes::new_place = false;

#endif
