/*
* Name: road_cross_tree.cpp
* Author: Jan Vlk
* Date: 16.11.2022
* Description: This file contains the class and functions for the BT itself.
* Last modified: 28.4.2023
*/

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "compass_msgs/Azimuth.h"

#include "road_crossing/get_azimuth.h"
#include "road_crossing/road_cross_tree.h"
#include "road_crossing/movement.h"
#include "road_crossing/get_gps.h"
#include "road_crossing/start_srv.h"
#include "road_crossing/vehicles.h"

Road_cross_tree::Road_cross_tree(std::string filename)
{
    // Main-BT
    this->factory.registerNodeType<Start_service::start_algorithm>("start_algorithm");

    // Init-BT
    this->factory.registerNodeType<GPS_nodes::get_position>("get_position");
    this->factory.registerNodeType<GPS_nodes::cross_road>("cross_road");
    this->factory.registerNodeType<GPS_nodes::place_suitable>("place_suitable");
    this->factory.registerNodeType<GPS_nodes::better_place>("better_place_found");
    this->factory.registerNodeType<MOV_nodes::move_to_place>("move_to_place");
    this->factory.registerNodeType<GPS_nodes::get_better_place>("get_better_place");

    // Perpendicular-BT
    this->factory.registerNodeType<AZI_nodes::get_azimuth>("get_azimuth");
    this->factory.registerNodeType<AZI_nodes::road_heading>("road_heading");
    this->factory.registerNodeType<AZI_nodes::compute_heading>("compute_heading");
    this->factory.registerNodeType<AZI_nodes::robot_perpendicular>("robot_perpendicular");
    this->factory.registerNodeType<MOV_nodes::rotate_robot>("rotate_robot");
    this->factory.registerNodeType<MOV_nodes::step_from_road>("step_from_road");

    // Crossing-BT
    this->factory.registerNodeType<VEH_nodes::get_cars_injector>("get_cars");
    this->factory.registerNodeType<VEH_nodes::cars_in_trajectory>("cars_in_trajectory");
    this->factory.registerNodeType<MOV_nodes::not_started>("not_started");
    this->factory.registerNodeType<MOV_nodes::start_movement>("start_movement");
    this->factory.registerNodeType<MOV_nodes::move_fwd_full>("move_fwd_full");
    this->factory.registerNodeType<VEH_nodes::calculate_collision>("calculate_collision");
    this->factory.registerNodeType<VEH_nodes::collision_imminent>("collision_imminent");
    this->factory.registerNodeType<MOV_nodes::move_fwd>("move_fwd");
    this->factory.registerNodeType<VEH_nodes::collision_fwd_move>("collision_fwd_move");
    this->factory.registerNodeType<MOV_nodes::move_bwd>("move_bwd");
    this->factory.registerNodeType<VEH_nodes::collision_bwd_move>("collision_bwd_move");
    this->factory.registerNodeType<MOV_nodes::stop_movement>("stop_movement");
    this->factory.registerNodeType<VEH_nodes::collision_on_stop>("collision_on_stop");
    this->factory.registerNodeType<MOV_nodes::crossing_finished_gps>("crossing_finished");

    this->tree = this->factory.createTreeFromFile(filename);
}

int Road_cross_tree::run_tree()
{
    this->tree.tickRoot();
    this->tree.sleep(std::chrono::milliseconds(10));
    return EXIT_SUCCESS;
}
