#ifndef VEHICLES
#define VEHICLES

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"

#include "road_crossing_msgs/injector_msgs.h"


struct collision_info
{
    int car_id;
    double v_front;
    double v_back;
    bool collide;
    bool collide_stop;
};

struct vehicle_info
{
    int id;
    double pos_x;
    double pos_y;
    double x_dot;
    double y_dot;
    double x_ddot;
    double y_ddot;
    double length;
    double width;
};

struct vehicles_data
{
    int num_vehicles;
    std::vector<vehicle_info> data;
};

struct collisions_data
{
    int num_collisions;
    std::vector<collision_info> data;
};

class VEH_nodes
{
    public:
        VEH_nodes(){}
        ~VEH_nodes(){}

        /**
         * @brief Calculate the collision data and determine if with the current speeds the collision will occur.
         *
         * @param vehicle Struct with data about the vehicle.
         * @param robot Struct with data about the robot.
         * @param collision Struct in which the output collision data will be stored.
         * 
         * TODO: add acceleration and vehicle width to the calculation
         */
        static void vehicle_collision(vehicle_info vehicle, vehicle_info robot, collision_info &collision);

        /**
         * @brief Callback function for the vehicle subscriber.
         *
         * @param msg Message with the vehicle data.
         * 
         * TODO: add message type & implement
         */
        static void callback_vehicle(); 

        /**
         * @brief Callback function for the vehicle injector subscriber.
         * 
         * @param node VEH_nodes object, necessary for static variables.
         * @param msg Message with the vehicle data from the injector.
         */
        static void callback_vehicle_injector(VEH_nodes* node, const road_crossing_msgs::injector_msgs::ConstPtr& msg);

        class get_cars : public BT::SyncActionNode
        {
            public:
                get_cars(const std::string& name, const BT::NodeConfiguration& config)
                    : BT::SyncActionNode(name, config)
                {}

                virtual ~get_cars(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

        class get_cars_injector : public BT::SyncActionNode
        {
            public:
                get_cars_injector(const std::string& name, const BT::NodeConfiguration& config)
                    : BT::SyncActionNode(name, config)
                {}

                virtual ~get_cars_injector(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

        class cars_in_trajectory : public BT::ConditionNode
        {
            public:
                cars_in_trajectory(const std::string& name, const BT::NodeConfiguration& config)
                    : BT::ConditionNode(name, config)
                {}

                virtual ~cars_in_trajectory(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

        class calculate_collision : public BT::SyncActionNode
        {
            public:
                calculate_collision(const std::string& name, const BT::NodeConfiguration& config)
                    : BT::SyncActionNode(name, config)
                {}

                virtual ~calculate_collision(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

        class collision_imminent : public BT::ConditionNode
        {
            public:
                collision_imminent(const std::string& name, const BT::NodeConfiguration& config)
                    : BT::ConditionNode(name, config)
                {}

                virtual ~collision_imminent(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

        class collision_fwd_move : public BT::ConditionNode
        {
            public:
                collision_fwd_move(const std::string& name, const BT::NodeConfiguration& config)
                    : BT::ConditionNode(name, config)
                {}

                virtual ~collision_fwd_move(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

        class collision_bwd_move : public BT::ConditionNode
        {
            public:
                collision_bwd_move(const std::string& name, const BT::NodeConfiguration& config)
                    : BT::ConditionNode(name, config)
                {}

                virtual ~collision_bwd_move(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

        class collision_on_stop : public BT::ConditionNode
        {
            public:
                collision_on_stop(const std::string& name, const BT::NodeConfiguration& config)
                    : BT::ConditionNode(name, config)
                {}

                virtual ~collision_on_stop(){}

                BT::NodeStatus tick() override;

                static BT::PortsList providedPorts();
        };

    private:
        static vehicle_info robot;
        static vehicles_data vehicles;
        static collisions_data collisions;
};

vehicle_info VEH_nodes::robot;
vehicles_data VEH_nodes::vehicles;
collisions_data VEH_nodes::collisions;

#endif
