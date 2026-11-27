#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <nav2_msgs/action/dock_robot.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/undock_robot.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include "turtlebot3_autonomous_patrol/bt_logger.hpp"

class ChargeBattery : public BT::StatefulActionNode
{
  using Dock = nav2_msgs::action::DockRobot;
  using Navigate = nav2_msgs::action::NavigateToPose;
  using Undock = nav2_msgs::action::UndockRobot;
  using Battery = sensor_msgs::msg::BatteryState;
  using Clock = std::chrono::steady_clock;

  // Shared with callbacks so cancellation also covers a goal that is accepted after halt.
  template<class Action>
  struct Request
  {
    using Handle = rclcpp_action::ClientGoalHandle<Action>;
    std::mutex mutex;
    typename Handle::SharedPtr handle;
    typename Handle::WrappedResult result{};
    bool rejected = false;
    bool finished = false;
    bool canceled = false;
    std::uint16_t feedback = 0;
    std::uint16_t retries = 0;
  };

public:
  ChargeBattery(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
  {
    const auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    BTLogger::init(node);

    dock_id_ = parameter(node, "dock_id", std::string{});
    max_dock_retries_ = parameter(node, "max_dock_retries", std::int64_t{0});
    retry_pose_ = parameter(node, "dock_retry_pose", std::vector<double>{});
    if (max_dock_retries_ < 0)
    {
      throw std::invalid_argument("max_dock_retries must be nonnegative");
    }
    if (!retry_pose_.empty() && (retry_pose_.size() != 3 ||
        !std::isfinite(retry_pose_[0]) || !std::isfinite(retry_pose_[1]) ||
        !std::isfinite(retry_pose_[2])))
    {
      throw std::invalid_argument("dock_retry_pose must be empty or [x, y, yaw] in the map frame");
    }
    if (max_dock_retries_ > 0 && !retry_pose_.empty())
    {
      navigate_client_ = rclcpp_action::create_client<Navigate>(node, "/navigate_to_pose");
    }
    full_threshold_ = static_cast<float>(parameter(node, "full_battery_threshold", 1.0));
    if (!std::isfinite(full_threshold_) || full_threshold_ <= 0 || full_threshold_ > 1)
    {
      throw std::invalid_argument("full_battery_threshold must be in (0, 1]");
    }
    if (parameter(node, "simulate_dock_recharge", false))
    {
      if (!parameter(node, "use_sim_time", false))
      {
        throw std::invalid_argument("simulate_dock_recharge requires use_sim_time");
      }
      recharge_start_ = node->create_publisher<std_msgs::msg::Bool>("battery/recharge/start", 10);
      recharge_stop_ = node->create_publisher<std_msgs::msg::Bool>("battery/recharge/stop", 10);
    }
    dock_client_ = rclcpp_action::create_client<Dock>(node, "/dock_robot");
    undock_client_ = rclcpp_action::create_client<Undock>(node, "/undock_robot");
    battery_sub_ = node->create_subscription<Battery>(
      "/battery/battery_state", 10, [this](const Battery& msg) {
        std::lock_guard<std::mutex> lock(battery_mutex_);
        battery_ = msg;
        battery_time_ = Clock::now();
      });
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override
  {
    if (dock_id_.empty())
    {
      return hold("set the dock_id parameter to enable docking");
    }
    dock_retries_ = 0;
    phase_ = Phase::WaitForDock;
    deadline_ = Clock::now() + std::chrono::seconds(20);
    BTLogger::log("ChargeBattery: waiting for /dock_robot (" + dock_id_ + ")");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    try
    {
      switch (phase_)
      {
        case Phase::WaitForDock: return start_docking();
        case Phase::Dock: return check_docking();
        case Phase::Retry: return start_retry();
        case Phase::PreStage: return check_pre_staging();
        case Phase::Charge: return check_charge();
        case Phase::StopRecharge: return confirm_recharge_stopped();
        case Phase::WaitForUndock: return start_undocking();
        case Phase::Undock: return check_undocking();
        case Phase::Hold: return BT::NodeStatus::RUNNING;
      }
    }
    catch (const std::exception& error)
    {
      return hold(error.what());
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    cancel(dock_client_, dock_);
    cancel(navigate_client_, pre_stage_);
    cancel(undock_client_, undock_);
    stop_recharge();
    BTLogger::log("ChargeBattery: halted");
  }

private:
  enum class Phase { WaitForDock, Dock, Retry, PreStage, Charge, StopRecharge, WaitForUndock, Undock, Hold };

  template<class T>
  static T parameter(const rclcpp::Node::SharedPtr& node, const char* name, const T& fallback)
  {
    return node->has_parameter(name) ? node->get_parameter(name).get_value<T>()
                                     : node->declare_parameter<T>(name, fallback);
  }

  template<class Action>
  static void send(const std::shared_ptr<rclcpp_action::Client<Action>>& client,
                   const typename Action::Goal& goal,
                   std::shared_ptr<Request<Action>>& request)
  {
    request = std::make_shared<Request<Action>>();
    const auto state = request;
    const std::weak_ptr<rclcpp_action::Client<Action>> weak_client = client;
    typename rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback = [state, weak_client](auto handle) {
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->canceled)
        {
          state->handle = handle;
          state->rejected = !handle;
          return;
        }
      }
      if (handle)
      {
        if (const auto client = weak_client.lock())
        {
          cancel_goal(client, handle);
        }
      }
    };
    options.result_callback = [state](const auto& result) {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->result = result;
      state->finished = true;
      state->handle.reset();
    };
    if constexpr (std::is_same_v<Action, Dock>)
    {
      options.feedback_callback = [state](auto, const auto feedback) {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->feedback = feedback->state;
        state->retries = feedback->num_retries;
      };
    }
    client->async_send_goal(goal, options);
  }

  template<class Action>
  static void cancel_goal(const std::shared_ptr<rclcpp_action::Client<Action>>& client,
                          const typename Request<Action>::Handle::SharedPtr& handle)
  {
    if (!rclcpp::ok())
    {
      return;
    }
    try
    {
      client->async_cancel_goal(handle);
    }
    catch (const rclcpp_action::exceptions::UnknownGoalHandleError&)
    {
      // The action finished before cancellation reached the client.
    }
  }

  template<class Action>
  static void cancel(const std::shared_ptr<rclcpp_action::Client<Action>>& client,
                     std::shared_ptr<Request<Action>>& request)
  {
    if (auto state = std::exchange(request, {}))
    {
      typename Request<Action>::Handle::SharedPtr handle;
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->canceled = true;
        handle = std::move(state->handle);
      }
      if (handle)
      {
        cancel_goal(client, handle);
      }
    }
  }

  template<class Result>
  static bool succeeded(const Result& result)
  {
    return result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
           result.result->success;
  }

  template<class Result>
  static std::string error_message(const Result& result)
  {
    if (!result.result)
    {
      return "no action result";
    }
    return "error " + std::to_string(result.result->error_code) + " " + result.result->error_msg;
  }

  BT::NodeStatus hold(const std::string& reason)
  {
    cancel(dock_client_, dock_);
    cancel(navigate_client_, pre_stage_);
    cancel(undock_client_, undock_);
    stop_recharge();
    phase_ = Phase::Hold;
    BTLogger::log("ChargeBattery: patrol held — " + reason + ". Check Seggy before restarting patrol.");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus retry_docking(const Request<Dock>::Handle::WrappedResult& result)
  {
    const auto reason = "docking failed: " + error_message(result);
    const auto code = result.result ? result.result->error_code : Dock::Result::UNKNOWN;
    const bool recoverable = code == Dock::Result::FAILED_TO_STAGE ||
      code == Dock::Result::FAILED_TO_DETECT_DOCK || code == Dock::Result::FAILED_TO_CONTROL ||
      code == Dock::Result::FAILED_TO_CHARGE || code == Dock::Result::TIMEOUT;
    if (result.code != rclcpp_action::ResultCode::ABORTED || !recoverable ||
        dock_retries_ >= max_dock_retries_)
    {
      return hold(reason);
    }

    dock_.reset();
    stop_recharge();
    ++dock_retries_;
    phase_ = Phase::Retry;
    deadline_ = Clock::now() + std::chrono::seconds(2);
    BTLogger::log("ChargeBattery: " + reason + "; scheduling retry " +
                  std::to_string(dock_retries_) + "/" + std::to_string(max_dock_retries_));
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus start_retry()
  {
    if (Clock::now() < deadline_)
    {
      return BT::NodeStatus::RUNNING;
    }
    if (!navigate_client_)
    {
      phase_ = Phase::WaitForDock;
      deadline_ = Clock::now() + std::chrono::seconds(20);
      return BT::NodeStatus::RUNNING;
    }
    if (!navigate_client_->action_server_is_ready())
    {
      return hold("/navigate_to_pose unavailable for docking pre-staging");
    }

    Navigate::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = retry_pose_[0];
    goal.pose.pose.position.y = retry_pose_[1];
    goal.pose.pose.orientation.z = std::sin(retry_pose_[2] / 2);
    goal.pose.pose.orientation.w = std::cos(retry_pose_[2] / 2);
    send(navigate_client_, goal, pre_stage_);
    phase_ = Phase::PreStage;
    deadline_ = Clock::now() + std::chrono::minutes(5);
    BTLogger::log("ChargeBattery: navigating to pre-staging for docking retry");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus check_pre_staging()
  {
    Request<Navigate>::Handle::WrappedResult result{};
    bool rejected;
    bool finished;
    {
      std::lock_guard<std::mutex> lock(pre_stage_->mutex);
      result = pre_stage_->result;
      rejected = pre_stage_->rejected;
      finished = pre_stage_->finished;
    }
    if (rejected || (finished && result.code != rclcpp_action::ResultCode::SUCCEEDED))
    {
      return hold(rejected ? "pre-staging goal rejected" : "pre-staging failed: " + error_message(result));
    }
    if (finished)
    {
      pre_stage_.reset();
      phase_ = Phase::WaitForDock;
      deadline_ = Clock::now() + std::chrono::seconds(20);
      BTLogger::log("ChargeBattery: reached pre-staging");
    }
    else if (Clock::now() >= deadline_)
    {
      return hold("pre-staging timed out");
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus start_docking()
  {
    const bool recharge_ready = !recharge_start_ ||
      (recharge_start_->get_subscription_count() && recharge_stop_->get_subscription_count());
    const bool dock_ready = dock_client_->action_server_is_ready();
    const bool undock_ready = undock_client_->action_server_is_ready();
    if (dock_ready && undock_ready && recharge_ready)
    {
      Dock::Goal goal;
      goal.use_dock_id = true;
      goal.dock_id = dock_id_;
      goal.navigate_to_staging_pose = true;
      send(dock_client_, goal, dock_);
      phase_ = Phase::Dock;
      deadline_ = Clock::now() + std::chrono::minutes(30);
      BTLogger::log("ChargeBattery: docking at " + dock_id_);
    }
    else if (Clock::now() >= deadline_)
    {
      if (!dock_ready)
      {
        return hold("/dock_robot unavailable");
      }
      return hold(undock_ready ? "Gazebo recharge bridge unavailable" : "/undock_robot unavailable");
    }
    return BT::NodeStatus::RUNNING;
  }

  static const char* dock_stage(std::uint16_t stage)
  {
    switch (stage)
    {
      case Dock::Feedback::NAV_TO_STAGING_POSE: return "navigating to staging";
      case Dock::Feedback::INITIAL_PERCEPTION: return "detecting dock";
      case Dock::Feedback::CONTROLLING: return "approaching dock";
      case Dock::Feedback::WAIT_FOR_CHARGE: return "waiting for charge confirmation";
      case Dock::Feedback::RETRY: return "retrying docking";
      default: return "waiting for docking feedback";
    }
  }

  BT::NodeStatus check_docking()
  {
    Request<Dock>::Handle::WrappedResult result{};
    bool rejected;
    bool finished;
    std::uint16_t stage;
    std::uint16_t retries;
    {
      std::lock_guard<std::mutex> lock(dock_->mutex);
      result = dock_->result;
      rejected = dock_->rejected;
      finished = dock_->finished;
      stage = dock_->feedback;
      retries = dock_->retries;
    }
    if (rejected)
    {
      return hold("docking goal rejected");
    }
    if (finished && !succeeded(result))
    {
      return retry_docking(result);
    }
    if (finished)
    {
      dock_.reset();
      start_recharge();
      phase_ = Phase::Charge;
      charge_started_ = Clock::now();
      charge_confirmed_ = false;
      deadline_ = charge_started_ + std::chrono::hours(4);
      BTLogger::log("ChargeBattery: docked, waiting for full charge");
      return BT::NodeStatus::RUNNING;
    }
    if (Clock::now() >= deadline_)
    {
      return hold("docking timed out");
    }

    BTLogger::log(std::string("ChargeBattery: ") + dock_stage(stage) +
                  " (retries " + std::to_string(retries) + ")");
    // The alignment wrapper can wait for CHARGING before completing DockRobot.
    if (stage == Dock::Feedback::WAIT_FOR_CHARGE)
    {
      start_recharge();
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus check_charge()
  {
    Battery battery;
    Clock::time_point sample_time;
    {
      std::lock_guard<std::mutex> lock(battery_mutex_);
      battery = battery_;
      sample_time = battery_time_;
    }
    const auto now = Clock::now();
    if (battery.present && sample_time >= charge_started_ && now - sample_time < std::chrono::seconds(10))
    {
      const bool valid = std::isfinite(battery.percentage) &&
                         battery.percentage >= 0 && battery.percentage <= 1;
      if (battery.power_supply_status == Battery::POWER_SUPPLY_STATUS_FULL ||
          (valid && battery.percentage >= full_threshold_))
      {
        if (recharge_stop_)
        {
          recharge_stop_requested_ = now;
          next_stop_request_ = now + std::chrono::seconds(1);
          deadline_ = now + std::chrono::seconds(10);
          phase_ = Phase::StopRecharge;
          stop_recharge(true);
          BTLogger::log("ChargeBattery: charge target reached, waiting for Gazebo to stop recharge");
        }
        else
        {
          prepare_to_undock();
        }
        return BT::NodeStatus::RUNNING;
      }
      charge_confirmed_ = charge_confirmed_ ||
        battery.power_supply_status == Battery::POWER_SUPPLY_STATUS_CHARGING;
    }
    if (!charge_confirmed_ && now - charge_started_ >= std::chrono::seconds(30))
    {
      return hold("charging was not confirmed at the dock");
    }
    if (now >= deadline_)
    {
      return hold("battery did not reach the charge target before timeout");
    }
    return BT::NodeStatus::RUNNING;
  }

  void prepare_to_undock()
  {
    phase_ = Phase::WaitForUndock;
    deadline_ = Clock::now() + std::chrono::seconds(20);
    BTLogger::log("ChargeBattery: preparing to undock");
  }

  BT::NodeStatus confirm_recharge_stopped()
  {
    bool confirmed;
    {
      std::lock_guard<std::mutex> lock(battery_mutex_);
      const auto status = battery_.power_supply_status;
      confirmed = battery_.present && battery_time_ > recharge_stop_requested_ &&
        (status == Battery::POWER_SUPPLY_STATUS_DISCHARGING ||
         status == Battery::POWER_SUPPLY_STATUS_NOT_CHARGING ||
         status == Battery::POWER_SUPPLY_STATUS_FULL);
    }
    if (confirmed)
    {
      BTLogger::log("ChargeBattery: Gazebo recharge stopped");
      prepare_to_undock();
    }
    else if (Clock::now() >= deadline_)
    {
      return hold("Gazebo did not confirm that recharge stopped");
    }
    else if (Clock::now() >= next_stop_request_)
    {
      stop_recharge(true);
      next_stop_request_ = Clock::now() + std::chrono::seconds(1);
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus start_undocking()
  {
    if (undock_client_->action_server_is_ready())
    {
      Undock::Goal goal;
      send(undock_client_, goal, undock_);
      phase_ = Phase::Undock;
      deadline_ = Clock::now() + std::chrono::seconds(60);
      BTLogger::log("ChargeBattery: undocking");
    }
    else if (Clock::now() >= deadline_)
    {
      return hold("/undock_robot unavailable");
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus check_undocking()
  {
    Request<Undock>::Handle::WrappedResult result{};
    bool rejected;
    bool finished;
    {
      std::lock_guard<std::mutex> lock(undock_->mutex);
      result = undock_->result;
      rejected = undock_->rejected;
      finished = undock_->finished;
    }
    if (rejected || (finished && !succeeded(result)))
    {
      return hold(rejected ? "undocking goal rejected" : "undocking failed: " + error_message(result));
    }
    if (finished)
    {
      undock_.reset();
      BTLogger::log("ChargeBattery: undocked, resuming patrol");
      return BT::NodeStatus::SUCCESS;
    }
    if (Clock::now() >= deadline_)
    {
      return hold("undocking timed out");
    }
    return BT::NodeStatus::RUNNING;
  }

  void start_recharge()
  {
    if (recharge_start_ && !recharge_active_)
    {
      std_msgs::msg::Bool command;
      command.data = true;
      recharge_start_->publish(command);
      recharge_active_ = true;
    }
  }

  void stop_recharge(bool resend = false)
  {
    if (recharge_stop_ && (recharge_active_ || resend))
    {
      std_msgs::msg::Bool command;
      command.data = true;
      recharge_stop_->publish(command);
      recharge_active_ = false;
    }
  }

  std::string dock_id_;
  std::int64_t max_dock_retries_{0};
  std::int64_t dock_retries_{0};
  std::vector<double> retry_pose_;
  float full_threshold_;
  Phase phase_{Phase::Hold};
  Clock::time_point deadline_{};
  Clock::time_point charge_started_{};
  Clock::time_point recharge_stop_requested_{};
  Clock::time_point next_stop_request_{};
  bool charge_confirmed_{false};
  bool recharge_active_{false};
  std::mutex battery_mutex_;
  Battery battery_;
  Clock::time_point battery_time_{};

  std::shared_ptr<Request<Dock>> dock_;
  std::shared_ptr<Request<Navigate>> pre_stage_;
  std::shared_ptr<Request<Undock>> undock_;
  rclcpp_action::Client<Dock>::SharedPtr dock_client_;
  rclcpp_action::Client<Navigate>::SharedPtr navigate_client_;
  rclcpp_action::Client<Undock>::SharedPtr undock_client_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recharge_start_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recharge_stop_;
  rclcpp::Subscription<Battery>::SharedPtr battery_sub_;
};
