
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/bool.hpp"

#include "mess2_msgs/action/vicon_calibrate.hpp"
#include "mess2_plugins/calibration.hpp"
#include "mess2_plugins/utils.hpp"

using TransformStamped = geometry_msgs::msg::TransformStamped;
using Action = mess2_msgs::action::VICONCalibrate;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

using namespace mess2_plugins;
namespace mess2_nodes
{
class VICONCalibrationServer : public rclcpp::Node
{
public:
    explicit VICONCalibrationServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("vicon_calibration_server", options), ready_flag_(false)
    {
        using namespace std::placeholders;

        this->declare_parameter("agent_name", "agent");
        this->get_parameter("agent_name", agent_name_);
        _vicon_topic = "/vicon/" + agent_name_ + "/" + agent_name_;

        _vicon_subscription = this->create_subscription<TransformStamped>(
            _vicon_topic,
            10,
            std::bind(&VICONCalibrationServer::_vicon_callback, this, _1)
        );

        _ready_subscription = this->create_subscription<std_msgs::msg::Bool>(
            "ready",
            10,
            std::bind(&VICONCalibrationServer::_ready_callback, this, _1)
        );

        this->_calibration_server = rclcpp_action::create_server<Action>(
            this,
            "calibrate_vicon",
            std::bind(&VICONCalibrationServer::_handle_goal, this, _1, _2),
            std::bind(&VICONCalibrationServer::_handle_cancel, this, _1),
            std::bind(&VICONCalibrationServer::_handle_accepted, this, _1)
        );
    }

private:
    rclcpp_action::Server<Action>::SharedPtr _calibration_server;
    rclcpp::Subscription<TransformStamped>::SharedPtr _vicon_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _ready_subscription;
    TransformStamped global_;
    std::string _vicon_topic;
    std::string agent_name_;
    std::atomic<bool> ready_flag_;

    rclcpp_action::GoalResponse _handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Action::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "received goal request");
        (void)uuid;
        if (goal->num_measurements > 0)
        {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } else {
            return rclcpp_action::GoalResponse::REJECT; 
        }
        
    }

    rclcpp_action::CancelResponse _handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void _handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread{std::bind(&VICONCalibrationServer::_calibration_execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void _calibration_execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "executing goal");
        rclcpp::Rate rate(20);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Action::Feedback>();
        auto result = std::make_shared<Action::Result>();

        TransformStamped meas1;
        TransformStamped meas2;
        int64_t counter1 = 0;
        int64_t counter2 = 0;
        rclcpp::Time toc = global_.header.stamp;
        std::string namespace_ = this->get_namespace();

        std::string command1 = "move the hawk to pose one and then execute the following command in a separate terminal:\nros2 topic pub -1 " + namespace_ + "/ready std_msgs/msg/Bool '{data: true}'";
        RCLCPP_INFO(this->get_logger(), "%s", command1.c_str());
        while (!ready_flag_ && rclcpp::ok()) {
            rate.sleep();
        }
        ready_flag_ = false;
        RCLCPP_INFO(this->get_logger(), "collecting measurements at pose one");

        while (rclcpp::ok() && counter1 < goal->num_measurements)
        {
            if (goal_handle->is_canceling()) {
                result->success = 0;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "goal canceled");
                return;
            }
            rclcpp::Time tic = global_.header.stamp;
            if (tic > toc)
            {
                meas1.header.stamp = tic;
                meas1.transform = update_measurement(meas1.transform, global_.transform, counter1);
                counter1 = counter1 + 1;
                toc = tic;
                
                feedback->progress = 0.0 + (static_cast<double>(counter1) / goal->num_measurements) * 0.5;
                goal_handle->publish_feedback(feedback);
            }
            rate.sleep();
        }

        std::string command2 = "move the hawk to pose two and then execute the following command in a separate terminal:\nros2 topic pub -1 " + namespace_ + "/ready std_msgs/msg/Bool '{data: true}'";
        RCLCPP_INFO(this->get_logger(), "%s", command2.c_str());
        while (!ready_flag_ && rclcpp::ok()) {
            rate.sleep();
        }
        ready_flag_ = false;
        RCLCPP_INFO(this->get_logger(), "collecting measurements at pose two");

        while (rclcpp::ok() && counter2 < goal->num_measurements)
        {
            if (goal_handle->is_canceling()) {
                result->success = 0;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "goal canceled");
                return;
            }
            rclcpp::Time tic = global_.header.stamp;
            if (tic > toc)
            {
                meas2.header.stamp = tic;
                meas2.transform = update_measurement(meas2.transform, global_.transform, counter2);
                counter2 = counter2 + 1;
                toc = tic;
                
                feedback->progress = 0.5 + (static_cast<double>(counter2) / goal->num_measurements) * 0.5;
                goal_handle->publish_feedback(feedback);
            }
            rate.sleep();
        }

        auto quat_diff = get_vicon_calibration(meas2.transform, meas1.transform);
        if (rclcpp::ok()) {
            result->quat_diff = quat_diff;
            result->success = 1;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "goal succeeded");
        }
    }

    void _vicon_callback(const TransformStamped::SharedPtr msg)
    {
        global_.header = msg->header;
        global_.transform = msg->transform;
    }

    void _ready_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        ready_flag_ = msg->data;
    }
};  
}  

RCLCPP_COMPONENTS_REGISTER_NODE(mess2_nodes::VICONCalibrationServer)