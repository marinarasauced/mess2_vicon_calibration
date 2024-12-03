
#include <cmath>
#include <cstdint>
#include <functional>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <random>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "mess2_msgs/action/vicon_calibrate.hpp"

using Action = mess2_msgs::action::VICONCalibrate;
using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

std::string get_executable_directory() {
    char buf[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (len != -1) {
        buf[len] = '\0';
        std::string path(buf);
        size_t pos = path.find_last_of('/');
        return (pos != std::string::npos) ? path.substr(0, pos) : ".";
    }
    return ".";
}

void create_directories(const std::string &path) {
    std::string cmd = "mkdir -p " + path;
    system(cmd.c_str()); 
}

namespace mess2_nodes
{
class VICONCalibrationClient : public rclcpp::Node
{
public:
    explicit VICONCalibrationClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("vicon_calibration_client", options)
    {
        this->declare_parameter("num_measurements", 1000);
        this->declare_parameter("agent_name", "agent");
        this->get_parameter("agent_name", agent_name_);

        this->_calibration_client = rclcpp_action::create_client<Action>(
            this,
            "calibrate_vicon"
        );

        this->_calibration_timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&VICONCalibrationClient::send_goal, this)
        );

    }

    void send_goal()
    {
        using namespace std::placeholders;

        this->_calibration_timer->cancel();

        if (!this->_calibration_client->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "action server not available after waiting");
        rclcpp::shutdown();
        }

        auto goal_msg = Action::Goal();
        this->get_parameter("num_measurements", goal_msg.num_measurements);

        RCLCPP_INFO(this->get_logger(), "sending goal");

        auto send_goal_options = rclcpp_action::Client<Action>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
        std::bind(&VICONCalibrationClient::_goal_response_callback, this, _1);
        
        send_goal_options.feedback_callback =
        std::bind(&VICONCalibrationClient::_feedback_callback, this, _1, _2);
        
        send_goal_options.result_callback =
        std::bind(&VICONCalibrationClient::_result_callback, this, _1);
        
        this->_calibration_client->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<Action>::SharedPtr _calibration_client;
    rclcpp::TimerBase::SharedPtr _calibration_timer;
    std::string agent_name_;

    void _goal_response_callback(const GoalHandle::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "goal was rejected by server");
        } else {
        RCLCPP_INFO(this->get_logger(), "goal accepted by server, waiting for result");
        }
    }

    void _feedback_callback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const Action::Feedback> feedback)
    {
        auto progress = feedback->progress;
        RCLCPP_INFO(this->get_logger(), "progress: %.2f", progress);
    }

    void _result_callback(const GoalHandle::WrappedResult &result) 
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "unknown result code");
                return;
        }

        YAML::Node yaml_node;
        yaml_node["x"] = result.result->quat_diff.x;
        yaml_node["y"] = result.result->quat_diff.y;
        yaml_node["z"] = result.result->quat_diff.z;
        yaml_node["w"] = result.result->quat_diff.w;

        std::string exe_dir = get_executable_directory();
        std::string output_dir = exe_dir + "/../../../calibration/" + agent_name_;
        create_directories(output_dir);

        std::string write_path = output_dir + "/calibration.yaml";
        std::ofstream fout(write_path);
        fout << yaml_node;
        fout.close();

        std::stringstream ss;
        ss << "quat_diff:\n";
        ss << "\tx: " << result.result->quat_diff.x << "\n";
        ss << "\ty: " << result.result->quat_diff.y << "\n";
        ss << "\tz: " << result.result->quat_diff.z << "\n";
        ss << "\tw: " << result.result->quat_diff.w << "\n";
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        rclcpp::shutdown();
    }
};  
}

RCLCPP_COMPONENTS_REGISTER_NODE(mess2_nodes::VICONCalibrationClient)