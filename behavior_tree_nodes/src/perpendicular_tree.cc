#include <iostream>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "perpendicular_nodes.hh"


int main(int argc, char *argv[])
{
    BT::BehaviorTreeFactory factory;

    factory.registerSimpleAction("GetAzimuth", std::bind(GetAzimuth));

    auto tree = factory.createTreeFromFile("./main_tree.xml");

    return EXIT_SUCCESS;
}
