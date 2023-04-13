#ifndef START_SRV
#define START_SRV

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "road_crossing/start_algorithm.h"


class Start_service
{
    public:
        Start_service() {}
        virtual ~Start_service() {}
        
        /**
         * @brief Callback for service server for starting the algorithm.
         * 
         * @param node COND_nodes object, necessary for static variables.
         * @param req Request from the service client.
         * @param res Response to the service client.
         */
        static bool start_algorithm_service(Start_service* node, road_crossing::start_algorithm::Request& req, const road_crossing::start_algorithm::Response& res);

        class start_algorithm : public BT::ConditionNode
        {
            public:
                start_algorithm(const std::string& name, const BT::NodeConfiguration& config) :
                    BT::ConditionNode(name, config)
                {}
                
                virtual ~start_algorithm(){}
                
                BT::NodeStatus tick() override;
                
                static BT::PortsList providedPorts();
        };

    private:
        static bool is_running;
};

bool Start_service::is_running = false;

#endif
