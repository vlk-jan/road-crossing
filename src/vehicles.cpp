/*
* Name: vehicles.cpp
* Author: Jan Vlk
* Date: 2.3.2022
* Description: This file contains functions for operations dealing with compass and azimuth.
* Last modified: 5.3.2023
*/

#include "road_crossing/vehicles.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include <cmath>

#include "road_crossing/misc.h"

bool vehicle_collision(vehicle_info vehicle, vehicle_info robot, collision_info &collision)
{
    // Calculate the intersection point of the robot's and vehicle's paths
    double beta = (robot.x_dot*(vehicle.pos_y-robot.pos_y) - robot.y_dot*(vehicle.pos_x-robot.pos_x))/(vehicle.x_dot*robot.y_dot - vehicle.y_dot*robot.x_dot);

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
    //ROS_INFO("Robot speed: %f, phi: %f", vehicle_speed, vehicle_phi);

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

    collision.car_id = vehicle.id;
    collision.v_front = v_front;
    collision.v_back = v_back;

    // Check if the robot will collide with the vehicle
    double rob_t = robot_travel_front/robot_speed;
    ROS_INFO("Time for robot till intersection: %f", rob_t);
    if (rob_t >= t_front && rob_t <= t_back)
        return true;
    else
        return false;
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
    vehicle.pos_y = 5;
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

    while (ros::ok())
    {
        bool ret = vehicle_collision(vehicle, robot, collision);
        ROS_INFO("Collision info: %d, %f, %f", collision.car_id, collision.v_front, collision.v_back);
        if (ret)
            ROS_INFO("Collision with vehicle %d", collision.car_id);
        else
            ROS_INFO("No collision");

        loop_rate.sleep();
    }
}
