// CheckBattery reads Seggy’s normalized battery state and gates the patrol.

#pragma once

#include <cmath>
#include <cstdint>
#include <mutex>
#include <stdexcept>

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include "turtlebot3_autonomous_patrol/bt_logger.hpp"

class CheckBattery : public BT::ConditionNode
{
  using BatteryState = sensor_msgs::msg::BatteryState;

public:
  CheckBattery(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
    // BT nodes are NOT rclcpp::Nodes — pull the shared handle from the blackboard
    auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    BTLogger::init(node);

    const auto threshold = [&node](const char* parameter, double default_value) {
      return node->has_parameter(parameter)
        ? node->get_parameter(parameter).as_double()
        : node->declare_parameter<double>(parameter, default_value);
    };
    const double low = threshold("low_battery_threshold", 0.20);
    const double full = threshold("full_battery_threshold", 1.0);
    if (!std::isfinite(low) || !std::isfinite(full) ||
        low < 0.0 || full > 1.0 || static_cast<float>(low) >= static_cast<float>(full))
    {
      throw std::invalid_argument(
        "Battery thresholds must satisfy 0 <= low_battery_threshold < full_battery_threshold <= 1");
    }
    low_threshold_ = static_cast<float>(low);
    full_threshold_ = static_cast<float>(full);

    RCLCPP_INFO(node->get_logger(), "Battery thresholds: low %.1f%%, full %.1f%%",
                low_threshold_ * 100.0f, full_threshold_ * 100.0f);

    battery_sub_ = node->create_subscription<BatteryState>(
      "/battery/battery_state", 10,
      [this](const BatteryState& msg) {
        if (!msg.present)
        {
          return;
        }

        const float level = msg.percentage;
        const bool valid_level = std::isfinite(level) && level >= 0.0f && level <= 1.0f;
        const bool reports_full =
          msg.power_supply_status == BatteryState::POWER_SUPPLY_STATUS_FULL;
        if (!valid_level && !reports_full)
        {
          return;
        }

        std::lock_guard<std::mutex> lock(battery_mutex_);
        battery_level_ = level;
        battery_status_ = msg.power_supply_status;
        received_battery_ = true;
        if (reports_full || level >= full_threshold_)
        {
          charging_required_ = false;
        }
        else if (level < low_threshold_)
        {
          charging_required_ = true;
        }
      });
  }

  BT::NodeStatus tick() override
  {
    float level;
    std::uint8_t status;
    bool charging_required;
    bool received_battery;
    {
      std::lock_guard<std::mutex> lock(battery_mutex_);
      level = battery_level_;
      status = battery_status_;
      charging_required = charging_required_;
      received_battery = received_battery_;
    }

    if (!std::isfinite(level) || level < 0.0f || level > 1.0f)
    {
      BTLogger::log(!received_battery
        ? "CheckBattery: waiting for a valid battery reading"
        : "CheckBattery: SUCCESS (battery status: Full)");
      return BT::NodeStatus::SUCCESS;
    }

    const auto percentage = std::to_string(std::lround(level * 100.0f)) + "%";
    const auto percentage_low = std::to_string(std::lround(low_threshold_ * 100.0f)) + "%";

    if (charging_required)
    {
      const auto percentage_full = std::to_string(std::lround(full_threshold_ * 100.0f)) + "%";
      BTLogger::log("CheckBattery: FAILURE — waiting for full charge (" + percentage +
                    ", status: " + status_label(status) + ", target " + percentage_full + " or FULL status)");
      return BT::NodeStatus::FAILURE;
    }
    BTLogger::log("CheckBattery: SUCCESS (battery " + percentage +
                  ", status: " + status_label(status) + "  — low threshold is " + percentage_low  + ")");
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  static const char* status_label(std::uint8_t status)
  {
    switch (status)
    {
      case BatteryState::POWER_SUPPLY_STATUS_CHARGING: return "Charging";
      case BatteryState::POWER_SUPPLY_STATUS_DISCHARGING: return "Discharging";
      case BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING: return "Not charging";
      case BatteryState::POWER_SUPPLY_STATUS_FULL: return "Full";
      default: return "Unknown";
    }
  }

  std::mutex battery_mutex_;
  float battery_level_{-1.0f};
  std::uint8_t battery_status_{BatteryState::POWER_SUPPLY_STATUS_UNKNOWN};
  bool received_battery_{false};
  bool charging_required_{false};
  float low_threshold_;
  float full_threshold_;
  rclcpp::Subscription<BatteryState>::SharedPtr battery_sub_;
};
