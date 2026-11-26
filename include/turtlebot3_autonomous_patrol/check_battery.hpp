// CheckBattery reads Seggy’s normalized battery state and gates the patrol.

#pragma once

#include <atomic>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include "turtlebot3_autonomous_patrol/bt_logger.hpp"

class CheckBattery : public BT::ConditionNode
{
public:
  CheckBattery(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
    // BT nodes are NOT rclcpp::Nodes — pull the shared handle from the blackboard
    auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    BTLogger::init(node);

    constexpr auto parameter = "low_battery_threshold";
    const double threshold = node->has_parameter(parameter)
      ? node->get_parameter(parameter).as_double()
      : node->declare_parameter<double>(parameter, 0.20);
    if (!std::isfinite(threshold) || threshold < 0.0 || threshold > 1.0)
    {
      throw std::invalid_argument("low_battery_threshold must be a fraction between 0.0 and 1.0");
    }
    threshold_ = static_cast<float>(threshold);

    RCLCPP_INFO(node->get_logger(), "Low battery threshold: %f percent", threshold_ * 100.0f);

    battery_sub_ = node->create_subscription<sensor_msgs::msg::BatteryState>(
      "/battery/battery_state", 10,
      [this](const sensor_msgs::msg::BatteryState& msg) {
        if (msg.present && std::isfinite(msg.percentage) &&
            msg.percentage >= 0.0f && msg.percentage <= 1.0f)
        {
          battery_level_.store(msg.percentage);
        }
      });
  }

  BT::NodeStatus tick() override
  {
    const float level = battery_level_.load();
    if (std::isnan(level))
    {
      BTLogger::log("CheckBattery: waiting for a valid battery reading");
      return BT::NodeStatus::SUCCESS;
    }

    // we need to round values here because we rely on logger filtering duplicate messages
    const auto percentage = std::to_string(std::lround(level * 100.0f)) + "%";
    const auto percentage_t = std::to_string(std::lround(threshold_ * 100.0f)) + "%";

    if (level >= threshold_)
    {
      BTLogger::log("CheckBattery: SUCCESS (battery GOOD: " + percentage + "  — threshold is " + percentage_t + ")");
      return BT::NodeStatus::SUCCESS;
    }
    BTLogger::log("CheckBattery: FAILURE — battery LOW: " + percentage + "  — threshold is " + percentage_t);
    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  std::atomic<float> battery_level_{std::numeric_limits<float>::quiet_NaN()};
  float threshold_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
};
