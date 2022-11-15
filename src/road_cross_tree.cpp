#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "compass_msgs/Azimuth.h"
#include "road_crossing/get_azimuth.h"
#include "road_crossing/road_cross_tree.h"

class Cross_road_tree
{
    public:
        Cross_road_tree(char *filename)
        {
            this->tree = this->factory.createTreeFromFile(filename);
        }

        BT::Tree tree;

        void set_azimuth(double azimuth)
        {
            this->azimuth = azimuth;
            ROS_INFO("Updated azimuth for tree to value: [%f]", this->azimuth);
        }

    private:
        double azimuth;
        BT::BehaviorTreeFactory factory;

};
