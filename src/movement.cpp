#include <cmath>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "road_crossing/movement.h"
#include "road_crossing/get_azimuth.h"

BT::NodeStatus rotate_robot::tick()
{
    BT::Optional<double> req_azimuth = getInput<double>("req_azimuth");
    BT::Optional<double> cur_azimuth = getInput<double>("cur_azimuth");
    BT::Optional<ros::NodeHandle> nh = getInput<ros::NodeHandle>("node_handle");

    if (!req_azimuth)
        throw BT::RuntimeError("missing required input req_azimuth: ", req_azimuth.error());
    if (!cur_azimuth)
        throw BT::RuntimeError("missing required input cur_azimuth: ", cur_azimuth.error());
    if (!nh)
        throw BT::RuntimeError("missing required input node_handle: ", nh.error());

    compass_indices param_indices;
    get_topic(&param_indices);
    int direction_ned = 1 ? param_indices.orientation_i == 0 : -1; // index 0 means ned
    // TODO calculations for each value type
    double diff = cur_azimuth.value() - req_azimuth.value();
    const double speed_const = MAX_ROT_SPEED/M_PI;
    double speed;

    // ensure that robot rotates maximal of half a circle
    if (diff > M_PI)
        diff -= 2*M_PI;
    else if (diff < -M_PI)
        diff += 2*M_PI;

    if (diff < 0){  // rotation to the right
        speed = direction_ned * (diff*speed_const);
    } else {  // rotation to the left
        speed = -1 * direction_ned * (diff*speed_const);
    }

    geometry_msgs::Twist msg = geometry_msgs::Twist();
    msg.angular.z = speed;

    ros::Publisher pub = nh.value().advertise<geometry_msgs::Twist>("cmd_vel", 5);
    pub.publish(msg);
    ros::spin();

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList rotate_robot::providedPorts()
{
    return {BT::InputPort<double>("req_azimuth"), BT::InputPort<double>("cur_azimuth"),
            BT::InputPort<ros::NodeHandle>("node_handle")};
}
