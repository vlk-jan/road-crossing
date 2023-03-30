/*
* Name: vehicles.cpp
* Author: Jan Vlk
* Date: 2.3.2022
* Description: This file contains functions for operations dealing with vehicle detection and collisions.
* Last modified: 30.3.2023
*/

#include <cmath>
#include <limits>

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "road_crossing/vehicles.h"
#include "road_crossing/movement.h"
#include "road_crossing/misc.h"


void VEH_nodes::vehicle_collision(vehicle_info vehicle, vehicle_info robot, collision_info &collision)
{
    // Calculate the intersection point of the robot's and vehicle's paths
    double beta = (robot.x_dot*(vehicle.pos_y-robot.pos_y) - robot.y_dot*(vehicle.pos_x-robot.pos_x))/
                  (vehicle.x_dot*robot.y_dot - vehicle.y_dot*robot.x_dot);

    double pos_x = vehicle.pos_x + beta*vehicle.x_dot;
    double pos_y = vehicle.pos_y + beta*vehicle.y_dot;
    ROS_INFO("Intersection point: (%f, %f)", pos_x, pos_y);

    // Calculate the time of the vehicle till the intersection point
    double vehicle_speed = sqrt(pow(vehicle.x_dot, 2) + pow(vehicle.y_dot, 2));
    double vehicle_phi = atan2(vehicle.y_dot, vehicle.x_dot);
    //ROS_INFO("Vehicle speed: %f, phi: %f", vehicle_speed, vehicle_phi);

    double vehicle_length_x = vehicle.length*cos(vehicle_phi)/2;
    double vehicle_length_y = vehicle.length*sin(vehicle_phi)/2;
    double vehicle_front_x = vehicle.pos_x + vehicle_length_x;
    double vehicle_front_y = vehicle.pos_y + vehicle_length_y;
    double vehicle_back_x = vehicle.pos_x - vehicle_length_x;
    double vehicle_back_y = vehicle.pos_y - vehicle_length_y;

    double vehicle_travel_front = sqrt(pow(pos_x - vehicle_front_x, 2) + pow(pos_y - vehicle_front_y, 2));
    double vehicle_travel_back = sqrt(pow(pos_x - vehicle_back_x, 2) + pow(pos_y - vehicle_back_y, 2));
    //ROS_INFO("Vehicle travel: %f, %f", vehicle_travel_front, vehicle_travel_back);

    double t_front = vehicle_travel_front/vehicle_speed;
    double t_back = vehicle_travel_back/vehicle_speed;
    ROS_INFO("Time for vehicle till intersection: %f, %f", t_front, t_back);

    // Calculate the velocity for robot to collide with the vehicle
    double robot_speed = sqrt(pow(robot.x_dot, 2) + pow(robot.y_dot, 2));
    double robot_phi = atan2(robot.y_dot, robot.x_dot);
    //ROS_INFO("Robot speed: %f, phi: %f", robot_speed, robot_phi);

    double robot_length_x = robot.length*cos(vehicle_phi)/2;
    double robot_length_y = robot.length*sin(vehicle_phi)/2;
    double robot_front_x = robot.pos_x + robot_length_x;
    double robot_front_y = robot.pos_y + robot_length_y;
    double robot_back_x = robot.pos_x - robot_length_x;
    double robot_back_y = robot.pos_y - robot_length_y;

    double robot_travel_front = sqrt(pow(pos_x - robot_front_x, 2) + pow(pos_y - robot_front_y, 2));
    double robot_travel_back = sqrt(pow(pos_x - robot_back_x, 2) + pow(pos_y - robot_back_y, 2));
    //ROS_INFO("Robot travel: %f, %f", robot_travel_front, robot_travel_back);

    double v_front = robot_travel_front/t_front;
    double v_back = robot_travel_front/t_back;

    // Check if the collision is in the direction of travel of the robot
    double rob_t = robot_travel_front/robot_speed;
    bool collision_front = ((robot.pos_x + rob_t*robot.x_dot) == pos_x) &&
                           ((robot.pos_y + rob_t*robot.y_dot) == pos_y);
    
    v_front = collision_front ? v_front : -v_front;
    v_back = collision_front ? v_back : -v_back;
    ROS_INFO("Velocity for collision: %f, %f", v_front, v_back);

    rob_t = collision_front ? rob_t : -rob_t;
    ROS_INFO("Time for robot till intersection: %f", rob_t);

    collision.car_id = vehicle.id;
    collision.v_front = v_front;
    collision.v_back = v_back;

    // Check if the robot will collide with the vehicle
    // if they both travel at the provided velocities
    if (rob_t >= t_front && rob_t <= t_back)
        collision.collide = true;
    else
        collision.collide = false;
}

BT::NodeStatus VEH_nodes::cars_in_trajectory::tick()
{
    if (VEH_nodes::vehicles.num_vehicles == 0) // No vehicles detected
        return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList VEH_nodes::cars_in_trajectory::providedPorts()
{
    return {};
}

BT::NodeStatus VEH_nodes::calculate_collision::tick()
{
    VEH_nodes::collisions.num_collisions = VEH_nodes::vehicles.num_vehicles;
    VEH_nodes::collisions.data = {};

    double max_vel_fwd = 0;
    double max_vel_bwd = 0;
    double min_vel_fwd = std::numeric_limits<double>::max();
    double min_vel_bwd = std::numeric_limits<double>::min();

    for (int i=0; i<VEH_nodes::vehicles.num_vehicles; ++i){
        collision_info collision;
        VEH_nodes::vehicle_collision(VEH_nodes::vehicles.data[i], VEH_nodes::robot, collision);
        VEH_nodes::collisions.data.push_back(collision);

        // Determine the velocities with regard to the direction of travel
        double higher_velocity, lower_velocity;
        if (collision.v_front >= 0){
            double higher_velocity = collision.v_front >= collision.v_back ? collision.v_front : collision.v_back;
            double lower_velocity = collision.v_front <= collision.v_back ? collision.v_front : collision.v_back;
        } else {
            double higher_velocity = collision.v_front <= collision.v_back ? collision.v_front : collision.v_back;
            double lower_velocity = collision.v_front >= collision.v_back ? collision.v_front : collision.v_back;
        }

        // Determine the maximum and minimum velocities with regard to the direction of travel
        if (collision.v_front >= 0){
            max_vel_fwd = higher_velocity > max_vel_fwd ? higher_velocity : max_vel_fwd;
            min_vel_fwd = lower_velocity < min_vel_fwd ? lower_velocity : min_vel_fwd;
        } else {
            max_vel_bwd = higher_velocity < max_vel_bwd ? higher_velocity : max_vel_bwd;
            min_vel_bwd = lower_velocity > min_vel_bwd ? lower_velocity : min_vel_bwd;
        }
    }

    setOutput<double>("max_vel_fwd", max_vel_fwd);
    setOutput<double>("min_vel_fwd", min_vel_fwd);
    setOutput<double>("max_vel_bwd", max_vel_bwd);
    setOutput<double>("min_vel_bwd", min_vel_bwd);

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList VEH_nodes::calculate_collision::providedPorts()
{
    return {BT::OutputPort<double>("max_vel_fwd"), BT::OutputPort<double>("min_vel_fwd"),
            BT::OutputPort<double>("max_vel_bwd"), BT::OutputPort<double>("min_vel_bwd")};
}

BT::NodeStatus VEH_nodes::collision_imminent::tick()
{
    for (int i=0; i<VEH_nodes::collisions.num_collisions; ++i){
        if (VEH_nodes::collisions.data[i].collide)
            return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::PortsList VEH_nodes::collision_imminent::providedPorts()
{
    return {};
}

BT::NodeStatus VEH_nodes::collision_fwd_move::tick()
{
    BT::Optional<double> max_vel_fwd = getInput<double>("max_vel_fwd");
    BT::Optional<double> min_vel_fwd = getInput<double>("min_vel_fwd");

    if (!max_vel_fwd)
        throw BT::RuntimeError("missing required input max_vel_fwd: ", max_vel_fwd.error());
    if (!min_vel_fwd)
        throw BT::RuntimeError("missing required input min_vel_fwd: ", min_vel_fwd.error());
    
    if (max_vel_fwd.value() >= MAX_LIN_SPEED && min_vel_fwd.value() <= MIN_LIN_SPEED)
        return BT::NodeStatus::SUCCESS;

    return BT::NodeStatus::FAILURE;
}

BT::PortsList VEH_nodes::collision_fwd_move::providedPorts()
{
    return {BT::InputPort<double>("max_vel_fwd"), BT::InputPort<double>("min_vel_fwd")};
}

BT::NodeStatus VEH_nodes::collision_bwd_move::tick()
{
    BT::Optional<double> max_vel_bwd = getInput<double>("max_vel_bwd");
    BT::Optional<double> min_vel_bwd = getInput<double>("min_vel_bwd");

    if (!max_vel_bwd)
        throw BT::RuntimeError("missing required input max_vel_bwd: ", max_vel_bwd.error());
    if (!min_vel_bwd)
        throw BT::RuntimeError("missing required input min_vel_bwd: ", min_vel_bwd.error());
    
    if (max_vel_bwd.value() <= -MAX_LIN_SPEED && min_vel_bwd.value() >= -MIN_LIN_SPEED)
        return BT::NodeStatus::SUCCESS;

    return BT::NodeStatus::FAILURE;
}

BT::PortsList VEH_nodes::collision_bwd_move::providedPorts()
{
    return {BT::InputPort<double>("max_vel_bwd"), BT::InputPort<double>("min_vel_bwd")};
}

/// @brief For testing purposes
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Vehicles");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    vehicle_info vehicle;
    vehicle.id = 1;
    vehicle.pos_x = -30;
    vehicle.pos_y = -5;
    vehicle.x_dot = 8.33;
    vehicle.y_dot = 0;
    vehicle.length = 4.7;
    vehicle.width = 1.8;

    vehicle_info robot;
    robot.id = 0;
    robot.pos_x = 0;
    robot.pos_y = 0;
    robot.x_dot = 0;
    robot.y_dot = 1.3;
    robot.length = 1.1;
    robot.width = 0.5;

    collision_info collision;

    VEH_nodes veh_nodes;

    while (ros::ok())
    {
        veh_nodes.vehicle_collision(vehicle, robot, collision);
        ROS_INFO("Collision info: %d, %f, %f", collision.car_id, collision.v_front, collision.v_back);
        if (collision.collide)
            ROS_INFO("Collision with vehicle %d", collision.car_id);
        else
            ROS_INFO("No collision");

        loop_rate.sleep();
    }
}
