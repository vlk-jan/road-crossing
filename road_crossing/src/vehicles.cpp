/*
* Name: vehicles.cpp
* Author: Jan Vlk
* Date: 2.3.2022
* Description: This file contains functions for operations dealing with vehicle detection and collisions.
* Last modified: 1.5.2023
*/

#include <cmath>
#include <limits>

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "road_crossing/vehicles.h"
#include "road_crossing/movement.h"
#include "road_crossing/misc.h"
#include "road_crossing_msgs/injector_msgs.h"
#include "road_crossing/get_gps.h"


void VEH_nodes::vehicle_collision(vehicle_info vehicle, vehicle_info robot, collision_info &collision)
{
    collision.car_id = vehicle.id;

    // Calculate the robot start position with respect to its length and vehicles width
    double robot_len_x = (robot.length + vehicle.width)/2;
    double robot_front_x = robot_len_x;
    double robot_front_y = 0;
    double robot_back_x = -robot_len_x;
    double robot_back_y = 0;

    // Calculate the vehicle start position with respect to its length and robots width
    double vehicle_phi = atan2(vehicle.y_dot, vehicle.x_dot);
    double vehicle_len_x = (vehicle.length + robot.width)*cos(vehicle_phi)/2;
    double vehicle_len_y = (vehicle.length + robot.width)*sin(vehicle_phi)/2;
    double vehicle_front_x = vehicle.pos_x + vehicle_len_x;
    double vehicle_front_y = vehicle.pos_y + vehicle_len_y;
    double vehicle_back_x = vehicle.pos_x - vehicle_len_x;
    double vehicle_back_y = vehicle.pos_y - vehicle_len_y;

    // Calculate the time till intersection point
    double time1 = 0, time2 = 0;
    if (vehicle.y_ddot == 0){
        time1 = -(vehicle_front_y)/(vehicle.y_dot);
        time2 = -(vehicle_back_y)/(vehicle.y_dot);
    } else {
        double a = (vehicle.y_ddot)/2;
        double b = vehicle.y_dot;
        double c1 = vehicle_front_y;
        double d1 = pow(b, 2) - 4*a*c1;
        //ROS_INFO("d1: %f", d1);
        if (d1 >= 0){
            double t1 = (-b + sqrt(d1))/(2*a);
            double t2 = (-b - sqrt(d1))/(2*a);
            //ROS_INFO("d1, t1: %f, t2: %f", t1, t2);
            if (t1 >= 0 && t2 >= 0)
                time1 = (t1 <= t2) ? t1 : t2;
            else if (t1 < 0 && t2 < 0)
                time1 = (t1 >= t2) ? t1 : t2;
            else
                time1 = (t1 >= 0) ? t1 : t2;
        }
        double c2 = vehicle_back_y;
        double d2 = pow(b, 2) - 4*a*c2;
        //ROS_INFO("d2: %f", d2);
        if (d2 >= 0){
            double t1 = (-b + sqrt(d2))/(2*a);
            double t2 = (-b - sqrt(d2))/(2*a);
            //ROS_INFO("d2, t1: %f, t2: %f", t1, t2);
            if (t1 >= 0 && t2 >= 0)
                time2 = (t1 <= t2) ? t1 : t2;
            else if (t1 < 0 && t2 < 0)
                time2 = (t1 >= t2) ? t1 : t2;
            else
                time2 = (t1 >= 0) ? t1 : t2;
        }
    }

    // Calculate the velocity for the robot to collide with the vehicle
    double vel1 = 0, vel2 = 0;
    if (time1){
        ROS_INFO("Time till intersection point front: %f", time1);
        double x1 = vehicle_front_x + vehicle.x_dot*time1 + vehicle.x_ddot*pow(time1, 2)/2;
        if (x1 <= robot_front_x && x1 >= robot_back_x)
            vel1 = 0;
        else if (time1 != 0)
            vel1 = (x1 - robot_back_x)/time1;
    }
    if (time2){
        ROS_INFO("Time till intersection point back: %f", time2);
        double x2 = vehicle_back_x + vehicle.x_dot*time2 + vehicle.x_ddot*pow(time2, 2)/2;
        if (x2 <= robot_front_x && x2 >= robot_back_x)
            vel2 = 0;
        else if (time2 != 0)
            vel2 = (x2 - robot_front_x)/time2;
    }

    // Set results to collision info
    if (vel1 && vel2){
        collision.v_front = vel1;
        collision.v_back = vel2;
        bool collide = (robot.x_dot <= vel1) && (robot.x_dot >= vel2);
        collision.collide = collide;
    } else {
        collision.v_front = (vel1) ? vel1 : 0;
        collision.v_back = (vel2) ? vel2 : 0;
        collision.collide = false;
        bool collide = (time1 != 0 && vel1 == 0) || (time2 != 0 && vel2 == 0);
        collision.collide_stop = collide;
    }

    ROS_INFO("Calculation, v_front: %f, v_back: %f", collision.v_front, collision.v_back);
}

void VEH_nodes::callback_vehicle_injector(const road_crossing_msgs::injector_msgs::ConstPtr& msg)
{
    double easting, northing;
    GPS_nodes::req_position(easting, northing);
    for (int i = 0; i < VEH_nodes::vehicles.data.size(); ++i){
        if (msg->veh_id == vehicles.data[i].id){
            VEH_nodes::vehicles.data[i].pos_x = msg->easting - easting;
            VEH_nodes::vehicles.data[i].pos_y = msg->northing - northing;
            VEH_nodes::vehicles.data[i].x_dot = msg->x_dot;
            VEH_nodes::vehicles.data[i].y_dot = msg->y_dot;
            VEH_nodes::vehicles.data[i].x_ddot = msg->x_ddot;
            VEH_nodes::vehicles.data[i].y_ddot = msg->y_ddot;
            VEH_nodes::vehicles.data[i].length = msg->length;
            VEH_nodes::vehicles.data[i].width = msg->width;

            ROS_INFO("vehicle pos_x: %f, pos_y: %f", VEH_nodes::vehicles.data[i].pos_x, VEH_nodes::vehicles.data[i].pos_y);

            return;
        }
    }

    vehicle_info vehicle;
    vehicle.id = msg->veh_id;
    vehicle.pos_x = msg->easting - easting;
    vehicle.pos_y = msg->northing - northing;
    vehicle.x_dot = msg->x_dot;
    vehicle.y_dot = msg->y_dot;
    vehicle.x_ddot = msg->x_ddot;
    vehicle.y_ddot = msg->y_ddot;
    vehicle.length = msg->length;
    vehicle.width = msg->width;

    VEH_nodes::vehicles.data.push_back(vehicle);
    ++VEH_nodes::vehicles.num_vehicles;
}

BT::NodeStatus VEH_nodes::get_cars_injector::tick()
{
    setOutput("vehicles", VEH_nodes::vehicles);

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList VEH_nodes::get_cars_injector::providedPorts()
{
    return {BT::OutputPort<vehicle_info>("vehicles")};
}

BT::NodeStatus VEH_nodes::cars_in_trajectory::tick()
{
    if (VEH_nodes::vehicles.num_vehicles == 0)  // No vehicles detected
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
    double min_vel_bwd = -std::numeric_limits<double>::max();

    VEH_nodes::robot.x_dot = MOV_nodes::get_lin_vel();

    for (int i=0; i<VEH_nodes::vehicles.num_vehicles; ++i){
        collision_info collision;
        VEH_nodes::vehicle_collision(VEH_nodes::vehicles.data[i], VEH_nodes::robot, collision);
        VEH_nodes::collisions.data.push_back(collision);
        ROS_INFO("v_front: %f, v_back: %f", collision.v_front, collision.v_back);

        // Determine the velocities with regard to the direction of travel
        double higher_velocity, lower_velocity;
        if (collision.v_front >= 0){
            higher_velocity = collision.v_front >= collision.v_back ? collision.v_front : collision.v_back;
            lower_velocity = collision.v_front <= collision.v_back ? collision.v_front : collision.v_back;
        } else {
            higher_velocity = collision.v_front <= collision.v_back ? collision.v_front : collision.v_back;
            lower_velocity = collision.v_front >= collision.v_back ? collision.v_front : collision.v_back;
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
    
    if (max_vel_fwd.value() + 0.1 >= MAX_LIN_SPEED && min_vel_fwd.value() - 0.1 <= MIN_LIN_SPEED)
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
    
    if (max_vel_bwd.value() - 0.1 <= -MAX_LIN_SPEED && min_vel_bwd.value() + 0.1 >= -MIN_LIN_SPEED)
        return BT::NodeStatus::SUCCESS;

    return BT::NodeStatus::FAILURE;
}

BT::PortsList VEH_nodes::collision_bwd_move::providedPorts()
{
    return {BT::InputPort<double>("max_vel_bwd"), BT::InputPort<double>("min_vel_bwd")};
}

BT::NodeStatus VEH_nodes::collision_on_stop::tick()
{
    for (int i=0; i<VEH_nodes::collisions.num_collisions; ++i){
        if (VEH_nodes::collisions.data[i].collide_stop)
            return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::PortsList VEH_nodes::collision_on_stop::providedPorts()
{
    return {};
}

/// @brief For testing purposes
/*int main(int argc, char **argv)
{
    ros::init(argc, argv, "Vehicles");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    vehicle_info vehicle;
    vehicle.id = 1;
    vehicle.pos_x = 0;
    vehicle.pos_y = 10;
    vehicle.x_dot = 0;
    vehicle.y_dot = -1;
    vehicle.x_ddot = 0;
    vehicle.y_ddot = 0.05;
    vehicle.length = 4.7;
    vehicle.width = 1.8;

    vehicle_info robot;
    robot.id = 0;
    robot.pos_x = 0;
    robot.pos_y = 0;
    robot.x_dot = 1.5;
    robot.y_dot = 0;
    robot.x_ddot = 0;
    robot.y_ddot = 0;
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
        else if (collision.collide_stop)
            ROS_INFO("Collision with vehicle %d, stop", collision.car_id);
        else
            ROS_INFO("No collision");

        loop_rate.sleep();
    }
}*/
