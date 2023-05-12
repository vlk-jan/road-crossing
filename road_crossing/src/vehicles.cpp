/*
* Name: vehicles.cpp
* Author: Jan Vlk
* Date: 2.3.2022
* Description: This file contains functions for operations dealing with vehicle detection and collisions.
* Last modified: 12.5.2023
*/

#include <cmath>
#include <limits>

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "road_crossing/vehicles.h"
#include "road_crossing/misc.h"
#include "road_crossing_msgs/injector_msgs.h"
#include "road_crossing/get_gps.h"


VEL_info::VEL_info()
{
    ros::param::param<double>("~max_rot_vel", VEL_info::max_rot_vel, 1.0);
    ros::param::param<double>("~min_rot_vel", VEL_info::min_rot_vel, 0.2);
    ros::param::param<double>("~max_lin_vel", VEL_info::max_lin_vel, 1.0);
    ros::param::param<double>("~min_lin_vel", VEL_info::min_lin_vel, 0.2);
    ros::param::param<double>("~vel_margin", VEL_info::vel_margin, 0.15);
}

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
    double vehicle_phi;
    if (vehicle.y_dot == 0 && vehicle.x_dot)
        vehicle_phi = atan2(vehicle.y_dot, vehicle.x_dot);
    else
        vehicle_phi = vehicle.phi;
    double vehicle_len_x = (vehicle.length + robot.width)*cos(vehicle_phi)/2;
    double vehicle_len_y = (vehicle.length + robot.width)*sin(vehicle_phi)/2;
    double vehicle_front_x = vehicle.pos_x + vehicle_len_x;
    double vehicle_front_y = vehicle.pos_y + vehicle_len_y;
    double vehicle_back_x = vehicle.pos_x - vehicle_len_x;
    double vehicle_back_y = vehicle.pos_y - vehicle_len_y;

    // Calculate the time till intersection point
    double time1 = 0, time2 = 0;
    if (vehicle.y_ddot == 0 && vehicle.y_dot != 0){  // If the vehicle is moving with constant velocity
        time1 = -(vehicle_front_y)/(vehicle.y_dot);
        time2 = -(vehicle_back_y)/(vehicle.y_dot);
    } else if (vehicle.y_ddot != 0){  // If the vehicle is accelerating/decelerating
        double a = (vehicle.y_ddot)/2;
        double b = vehicle.y_dot;
        double c1 = vehicle_front_y;
        double d1 = pow(b, 2) - 4*a*c1;
        ROS_INFO("d1: %f", d1);
        if (d1 >= 0){
            double t1 = (-b + sqrt(d1))/(2*a);
            double t2 = (-b - sqrt(d1))/(2*a);
            ROS_INFO("d1, t1: %f, t2: %f", t1, t2);
            if (t1 >= 0 && t2 >= 0)
                time1 = (t1 <= t2) ? t1 : t2;
            else if (t1 < 0 && t2 < 0)
                time1 = (t1 >= t2) ? t1 : t2;
            else
                time1 = (t1 >= 0) ? t1 : t2;
        }
        double c2 = vehicle_back_y;
        double d2 = pow(b, 2) - 4*a*c2;
        ROS_INFO("d2: %f", d2);
        if (d2 >= 0){
            double t1 = (-b + sqrt(d2))/(2*a);
            double t2 = (-b - sqrt(d2))/(2*a);
            ROS_INFO("d2, t1: %f, t2: %f", t1, t2);
            if (t1 >= 0 && t2 >= 0)
                time2 = (t1 <= t2) ? t1 : t2;
            else if (t1 < 0 && t2 < 0)
                time2 = (t1 >= t2) ? t1 : t2;
            else
                time2 = (t1 >= 0) ? t1 : t2;
        }
    } else {  // If the vehicle is stationary
        // Check if the vehicle is in front of the robot
        ROS_INFO("Veh y position: %f, %f", vehicle_front_y, vehicle_back_y);
        if ((vehicle_front_y <= 0 && vehicle_back_y >= 0) || (vehicle_front_y >= 0 && vehicle_back_y <= 0)){
            collision.v_front = VEL_info::get_max_lin_vel();
            collision.v_back = VEL_info::get_min_lin_vel();
            collision.collide = true;
            collision.collide_stop = false;
            return;
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
        vel1 = (time1 < 0 && vel1 > 0) ? -vel1 : vel1;
    }
    if (time2){
        ROS_INFO("Time till intersection point back: %f", time2);
        double x2 = vehicle_back_x + vehicle.x_dot*time2 + vehicle.x_ddot*pow(time2, 2)/2;
        if (x2 <= robot_front_x && x2 >= robot_back_x)
            vel2 = 0;
        else if (time2 != 0)
            vel2 = (x2 - robot_front_x)/time2;
        vel2 = (time2 < 0 && vel2 > 0) ? -vel2 : vel2;
    }

    // Set results to collision info
    if (vel1 && vel2){
        collision.v_front = vel1;
        collision.v_back = vel2;
        bool collide = (robot.x_dot <= vel1) && (robot.x_dot >= vel2);
        collision.collide = collide;
        collision.collide_stop = false;
    } else {
        collision.v_front = (vel1) ? vel1 : 0;
        collision.v_back = (vel2) ? vel2 : 0;
        collision.collide = false;
        bool collide = (time1 != 0 && vel1 == 0) || (time2 != 0 && vel2 == 0);
        collision.collide_stop = collide;
    }
}

void VEH_nodes::callback_vehicle_injector(const road_crossing_msgs::injector_msgs::ConstPtr& msg)
{
    if (msg->clear){
        VEH_nodes::clear_vehicles_data();
        return;
    }
    double easting, northing;
    GPS_nodes::req_position(easting, northing);
    for (int i = 0; i < VEH_nodes::vehicles.data.size(); ++i){
        if (msg->veh_id == vehicles.data[i].id){
            VEH_nodes::vehicles.data[i].pos_y = msg->easting - easting;
            VEH_nodes::vehicles.data[i].pos_x = msg->northing - northing;
            VEH_nodes::vehicles.data[i].x_dot = msg->x_dot;
            VEH_nodes::vehicles.data[i].y_dot = msg->y_dot;
            VEH_nodes::vehicles.data[i].x_ddot = msg->x_ddot;
            VEH_nodes::vehicles.data[i].y_ddot = msg->y_ddot;
            VEH_nodes::vehicles.data[i].length = msg->length;
            VEH_nodes::vehicles.data[i].width = msg->width;
            VEH_nodes::vehicles.data[i].phi = msg->phi;

            ROS_INFO("vehicle: %ld, pos_x: %f, pos_y: %f", msg->veh_id, VEH_nodes::vehicles.data[i].pos_x, VEH_nodes::vehicles.data[i].pos_y);

            return;
        }
    }

    vehicle_info vehicle;
    vehicle.id = msg->veh_id;
    vehicle.pos_y = msg->easting - easting;
    vehicle.pos_x = msg->northing - northing;
    vehicle.x_dot = msg->x_dot;
    vehicle.y_dot = msg->y_dot;
    vehicle.x_ddot = msg->x_ddot;
    vehicle.y_ddot = msg->y_ddot;
    vehicle.length = msg->length;
    vehicle.width = msg->width;
    vehicle.phi = msg->phi;

    VEH_nodes::vehicles.data.push_back(vehicle);
    ++VEH_nodes::vehicles.num_vehicles;

    int i = VEH_nodes::vehicles.num_vehicles-1;
    ROS_INFO("vehicle: %ld, pos_x: %f, pos_y: %f", msg->veh_id, VEH_nodes::vehicles.data[i].pos_x, VEH_nodes::vehicles.data[i].pos_y);
}

void VEH_nodes::clear_vehicles_data()
{
    VEH_nodes::vehicles.data.clear();
    VEH_nodes::vehicles.num_vehicles = 0;
}

void VEH_nodes::update_intervals(collision_info collision, std::vector<d_interval>& interval_move)
{
    d_interval interval;
    interval.min = collision.v_back < collision.v_front ? collision.v_back : collision.v_front;
    interval.max = collision.v_front > collision.v_back ? collision.v_front : collision.v_back;
    for (int i=0; i<interval_move.size(); ++i){
        if (interval.max < interval_move[i].min || interval.min > interval_move[i].max)
            continue;
        else if (interval.min <= interval_move[i].min && interval.max >= interval_move[i].max){
            interval_move.erase(interval_move.begin()+i);
            --i;
            if (interval_move.size() == 0)
                break;
        } else if (interval.min <= interval_move[i].min && interval.max < interval_move[i].max){
            interval_move[i].min = interval.max;
        } else if (interval.min > interval_move[i].min && interval.max >= interval_move[i].max){
            interval_move[i].max = interval.min;
        } else if (interval.min > interval_move[i].min && interval.max < interval_move[i].max){
            d_interval new_interval;
            new_interval.min = interval_move[i].min;
            new_interval.max = interval.min;
            interval_move[i].min = interval.max;
            interval_move.push_back(new_interval);
        }
    }
}

void VEH_nodes::set_intervals(std::vector<d_interval> interval_fwd, std::vector<d_interval> interval_bwd,
                              double& max_vel_fwd, double& min_vel_fwd, double& max_vel_bwd, double& min_vel_bwd)
{
    for (int i = 0; i < interval_fwd.size(); ++i){
        if (interval_fwd[i].max - interval_fwd[i].min > 2*VEL_info::get_vel_margin())
            continue;
        interval_fwd.erase(interval_fwd.begin()+i);
        --i;
    }
    for (int i = 0; i < interval_bwd.size(); ++i){
        if (interval_bwd[i].max - interval_bwd[i].min > 2*VEL_info::get_vel_margin())
            continue;
        interval_bwd.erase(interval_bwd.begin()+i);
        --i;
    }
    if (interval_fwd.size() > 0){
        for (int i = 0; i < interval_fwd.size(); ++i){
            if (interval_fwd[i].max > max_vel_fwd && interval_fwd[i].max - VEL_info::get_vel_margin() > VEL_info::get_min_lin_vel()){
                max_vel_fwd = interval_fwd[i].max;
                min_vel_fwd = interval_fwd[i].min;
            }
        }
    } else {
        max_vel_fwd = 0;
        min_vel_fwd = 0;
    }
    if (interval_bwd.size() > 0){
        for (int i = 0; i < interval_bwd.size(); ++i){
            if (interval_bwd[i].min < max_vel_bwd && interval_bwd[i].min + VEL_info::get_vel_margin() < -VEL_info::get_min_lin_vel()){
                max_vel_bwd = interval_bwd[i].min;
                min_vel_bwd = interval_bwd[i].max;
            }
        }
    } else {
        max_vel_bwd = 0;
        min_vel_bwd = 0;
    }
}

void VEH_nodes::set_robot_vel(double x_dot, double y_dot)
{
    VEH_nodes::robot.x_dot = x_dot;
    VEH_nodes::robot.y_dot = y_dot;
}

BT::NodeStatus VEH_nodes::get_cars_injector::tick()
{
    setOutput("vehicles", VEH_nodes::vehicles);

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList VEH_nodes::get_cars_injector::providedPorts()
{
    return {BT::OutputPort<vehicles_data>("vehicles")};
}

BT::NodeStatus VEH_nodes::cars_in_trajectory::tick()
{
    BT::Optional<vehicles_data> veh_data = getInput<vehicles_data>("vehicles");
    if (!veh_data)
        throw BT::RuntimeError("missing required input vehicles: ", veh_data.error());
    
    if (veh_data.value().num_vehicles == 0)  // No vehicles detected
        return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList VEH_nodes::cars_in_trajectory::providedPorts()
{
    return {BT::InputPort<vehicles_data>("vehicles")};
}

BT::NodeStatus VEH_nodes::calculate_collision::tick()
{
    BT::Optional<vehicles_data> veh_data = getInput<vehicles_data>("vehicles");
    if (!veh_data)
        throw BT::RuntimeError("missing required input vehicles: ", veh_data.error());

    VEH_nodes::collisions.num_collisions = veh_data.value().num_vehicles;
    VEH_nodes::collisions.data.clear();

    std::vector<d_interval> interval_fwd, interval_bwd;
    d_interval base_fwd, base_bwd;

    base_fwd.min = VEL_info::get_min_lin_vel()-VEL_info::get_vel_margin();
    base_fwd.max = VEL_info::get_max_lin_vel()+VEL_info::get_vel_margin();
    interval_fwd.push_back(base_fwd);

    base_bwd.min = -VEL_info::get_max_lin_vel()+VEL_info::get_vel_margin();
    base_bwd.max = -VEL_info::get_min_lin_vel()-VEL_info::get_vel_margin();
    interval_bwd.push_back(base_bwd);

    for (int i=0; i<veh_data.value().num_vehicles; ++i){
        collision_info collision;
        VEH_nodes::vehicle_collision(veh_data.value().data[i], VEH_nodes::robot, collision);
        VEH_nodes::collisions.data.push_back(collision);
        ROS_INFO("veh_id: %d, v_front: %f, v_back: %f", collision.car_id, collision.v_front, collision.v_back);

        // Determine the maximum and minimum velocities with regard to the direction of travel
        if (collision.v_front >= 0 && collision.v_back >= 0){  // Forward movement, both velocities
            if (collision.v_front >= base_fwd.max && collision.v_back >= base_fwd.max)
                continue;
            else if (collision.v_front <= base_fwd.min && collision.v_back <= base_fwd.min)
                continue;
            else if (collision.v_front >= base_fwd.max && collision.v_back <= base_fwd.min){
                d_interval interval;
                interval.min = 0;
                interval.max = 0;
                interval_fwd.clear();
                interval_fwd.push_back(interval);
            }
            VEH_nodes::update_intervals(collision, interval_fwd);
        } else if (collision.v_front <= 0 && collision.v_back <= 0){  // Backward movement, both velocities
            if (collision.v_front <= base_bwd.min && collision.v_back <= base_bwd.min)
                continue;
            else if (collision.v_front >= base_bwd.max && collision.v_back >= base_bwd.max)
                continue;
            else if (collision.v_front <= base_bwd.min && collision.v_back >= base_bwd.max){
                d_interval interval;
                interval.min = 0;
                interval.max = 0;
                interval_bwd.clear();
                interval_bwd.push_back(interval);
            }
            VEH_nodes::update_intervals(collision, interval_bwd);
        } else if (collision.v_front <= 0 && collision.v_back >= 0){  // Velocity front backward, velocity back forward
            if (collision.v_back <= VEL_info::get_min_lin_vel()){
                d_interval interval;
                interval.min = 0;
                interval.max = 0;
                interval_fwd.clear();
                interval_fwd.push_back(interval);
            } else if (collision.v_back <= VEL_info::get_max_lin_vel()){
                d_interval interval;
                interval.min = VEL_info::get_min_lin_vel();
                interval.max = collision.v_back;
                interval_fwd.clear();
                interval_fwd.push_back(interval);
            } else
                continue;
        } else {
            ROS_WARN("calculate_collision: Error in velocity calculation");
        }
    }

    for (int i = 0; i < interval_fwd.size(); ++i){
        ROS_INFO("Interval fwd: %f - %f", interval_fwd[i].min, interval_fwd[i].max);
    }
    for (int i = 0; i < interval_bwd.size(); ++i){
        ROS_INFO("Interval bwd: %f - %f", interval_bwd[i].min, interval_bwd[i].max);
    }

    // Set highest intervals with enough margin
    double max_vel_fwd = VEL_info::get_min_lin_vel(), min_vel_fwd = 0;
    double max_vel_bwd = -VEL_info::get_min_lin_vel(), min_vel_bwd = 0;
    VEH_nodes::set_intervals(interval_fwd, interval_bwd, max_vel_fwd, min_vel_fwd, max_vel_bwd, min_vel_bwd);

    setOutput<double>("max_vel_fwd", max_vel_fwd);
    setOutput<double>("min_vel_fwd", min_vel_fwd);
    setOutput<double>("max_vel_bwd", max_vel_bwd);
    setOutput<double>("min_vel_bwd", min_vel_bwd);

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList VEH_nodes::calculate_collision::providedPorts()
{
    return {BT::InputPort<vehicles_data>("vehicles"),
            BT::OutputPort<double>("max_vel_fwd"), BT::OutputPort<double>("min_vel_fwd"),
            BT::OutputPort<double>("max_vel_bwd"), BT::OutputPort<double>("min_vel_bwd")};
}

BT::NodeStatus VEH_nodes::collision_fwd_move::tick()
{
    BT::Optional<double> max_vel_fwd = getInput<double>("max_vel_fwd");
    BT::Optional<double> min_vel_fwd = getInput<double>("min_vel_fwd");

    if (!max_vel_fwd)
        throw BT::RuntimeError("missing required input max_vel_fwd: ", max_vel_fwd.error());
    if (!min_vel_fwd)
        throw BT::RuntimeError("missing required input min_vel_fwd: ", min_vel_fwd.error());
    
    if (max_vel_fwd.value() - min_vel_fwd.value() <=  2*VEL_info::get_vel_margin()){
        ROS_INFO("Collision fwd move");
        return BT::NodeStatus::SUCCESS;
    }

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
    
    if (max_vel_bwd.value() - min_vel_bwd.value() <= 2*VEL_info::get_vel_margin()){
        ROS_INFO("Collision bwd move");
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

BT::PortsList VEH_nodes::collision_bwd_move::providedPorts()
{
    return {BT::InputPort<double>("max_vel_bwd"), BT::InputPort<double>("min_vel_bwd")};
}

BT::NodeStatus VEH_nodes::collision_on_stop::tick()
{
    for (int i=0; i<VEH_nodes::collisions.num_collisions; ++i){
        if (VEH_nodes::collisions.data[i].collide_stop){
            ROS_INFO("Collision on stop, veh_id: %d", VEH_nodes::collisions.data[i].car_id);
            return BT::NodeStatus::SUCCESS;
        }
    }
    return BT::NodeStatus::FAILURE;
}

BT::PortsList VEH_nodes::collision_on_stop::providedPorts()
{
    return {};
}
