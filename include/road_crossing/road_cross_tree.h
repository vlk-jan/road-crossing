#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "compass_msgs/Azimuth.h"
#include "road_crossing/get_azimuth.h"

class Road_cross_tree
{
    public:
        Road_cross_tree(char *filename);

        BT::Tree tree;

        void set_azimuth(double azimuth);

    private:
        double azimuth;

        BT::BehaviorTreeFactory factory;
};
