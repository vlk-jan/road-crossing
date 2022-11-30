#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/ros.h"
#include "compass_msgs/Azimuth.h"
#include "road_crossing/get_azimuth.h"

double azimuth;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GetAzimuth");
    ros::NodeHandle n;
    std::string topic_prefix = "compass/";
    std::string topic = topic_prefix;
    std::string topic_suffix = get_topic();
    if (topic_suffix == ""){
        ROS_ERROR("No topic found");
        //return EXIT_FAILURE;
    } else
        ros::Subscriber sub = n.subscribe(topic, 1000, get_azimuth_callback);
    ros::spin();
}

std::string get_topic()
{
    std::string param = "publish_";
    std::string reference_str[] = {"utm", "true", "mag"};
    std::string orientation_str[] = {"ned", "enu"};
    std::string data_type_str[] = {"rad", "deg", "quat", "imu", "pose"};

    bool published = false;

    for (int i=0; i<reference_str->length(); ++i){
        for (int j=0; j<orientation_str->length(); ++j){
            for (int k=0; k<data_type_str->length(); ++k){
                param += reference_str[i] + "_azimuth_" + orientation_str[j] + "_" + data_type_str[k];
                ros::param::get(param, published);
                if (published){
                    return (reference_str[i] + "/" +  orientation_str[i] + "/" + data_type_str[k]);
                }
            }
        }
    }
    return "";
}

void get_topic(compass_indices *indices)
{
    std::string param = "publish_";
    std::string reference_str[] = {"utm", "true", "mag"};
    std::string orientation_str[] = {"ned", "enu"};
    std::string data_type_str[] = {"rad", "deg", "quat", "imu", "pose"};

    bool published = false;

    for (int i=0; i<reference_str->length(); ++i){
        for (int j=0; j<orientation_str->length(); ++j){
            for (int k=0; k<data_type_str->length(); ++k){
                param += reference_str[i] + "_azimuth_" + orientation_str[j] + "_" + data_type_str[k];
                ros::param::get(param, published);
                if (published){
                    indices->reference_i = i;
                    indices->orientation_i = j;
                    indices->data_type_i = k;
                    return;
                }
            }
        }
    }
    return;
}

void get_azimuth_callback(const compass_msgs::Azimuth::ConstPtr& msg)
{
    // TODO: Is there a better way than global variable?
    azimuth = msg->azimuth;
    ROS_INFO("Current azimuth: [%f]", azimuth);
}

BT::NodeStatus get_required_azimuth::tick()
{
    // TODO: dynamic function, currently for testing purpose const value
    setOutput("req_azimuth", 3.f);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList get_required_azimuth::providedPorts()
{
    return {BT::OutputPort<double>("req_azimuth")};
}

BT::NodeStatus get_current_azimuth::tick()
{
    setOutput("cur_azimuth", azimuth);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList get_current_azimuth::providedPorts()
{
    return {BT::OutputPort<double>("cur_azimuth")};
}

BT::NodeStatus equal_azimuths::tick()
{
    BT::Optional<double> req_azimuth = getInput<double>("req_azimuth");
    BT::Optional<double> cur_azimuth = getInput<double>("cur_azimuth");
    
    if (!req_azimuth)
        throw BT::RuntimeError("missing required input req_azimuth: ", req_azimuth.error());
    if (!cur_azimuth)
        throw BT::RuntimeError("missing required input cur_azimuth: ", cur_azimuth.error());
    
    if (abs(req_azimuth.value() - cur_azimuth.value()) < 0.3f)
        return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::FAILURE;
}

BT::PortsList equal_azimuths::providedPorts()
{
    return {BT::InputPort<double>("req_azimuth"), BT::InputPort<double>("cur_azimuth")};
}
