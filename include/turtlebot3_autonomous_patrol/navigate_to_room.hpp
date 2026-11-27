// navigate_to_room.hpp — BT action node that drives the robot to a map coordinate
// takes x/y input ports, builds a PoseStamped, sends it to Nav2's /navigate_to_pose
// uses the two-phase async pattern:
//   phase 1: async_send_goal → poll goal_future_ in onRunning()
//   phase 2: async_get_result → poll result_future_ in onRunning()
// onHalted() cancels the in-flight goal if the tree preempts (e.g. object detected)

#pragma once

#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2/utils.hpp>
#include <tf2_ros/transform_listener.hpp>
#include "turtlebot3_autonomous_patrol/bt_logger.hpp"

class NavigateToRoom : public BT::StatefulActionNode
{
  struct ArrivalTransforms
  {
    explicit ArrivalTransforms(rclcpp::Node& node)
      : listener(buffer, tf2_ros::TransformListener::RequiredInterfaces(node), false)
    {}

    tf2::BufferCore buffer;
    tf2_ros::TransformListener listener;
  };

public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

private:
  struct GoalFeedback
  {
    std::mutex mutex;
    std::shared_ptr<const NavigateToPose::Feedback> latest;
    std::chrono::steady_clock::time_point received_at;
  };

public:
  NavigateToRoom(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    BTLogger::init(node_);

    if (!config.blackboard->get("patrol_arrival_transforms", transforms_))
    {
      transforms_ = std::make_shared<ArrivalTransforms>(*node_);
      config.blackboard->set("patrol_arrival_transforms", transforms_);
    }
    action_client_ = rclcpp_action::create_client<NavigateToPose>(
      node_, "/navigate_to_pose");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("x", "X coordinate in map frame"),
      BT::InputPort<double>("y", "Y coordinate in map frame"),
      BT::InputPort<std::string>("label", "patrol stop", "Name used in navigation logs")
    };
  }

  // first tick: read coordinates, build goal, send to Nav2
  BT::NodeStatus onStart() override
  {
    if (!getInput<double>("x", goal_x_) || !getInput<double>("y", goal_y_))
    {
      throw BT::RuntimeError("missing required input [x] or [y]");
    }
    const auto label = getInput<std::string>("label");
    label_ = label && !label.value().empty() ? label.value() : "patrol stop";

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = goal_x_;
    goal_msg.pose.pose.position.y = goal_y_;
    goal_msg.pose.pose.orientation.w = 1.0;  // identity quaternion

    if (!action_client_->wait_for_action_server(std::chrono::seconds(20)))
    {
      BTLogger::log("NavigateToRoom: FAILURE — Nav2 server unavailable");
      return BT::NodeStatus::FAILURE;
    }

    std::ostringstream departure;
    departure << "NavigateToRoom: heading to ";
    append_goal(departure);
    append_map_pose(departure, "from", false);

    feedback_ = std::make_shared<GoalFeedback>();
    goal_handle_.reset();
    result_future_ = {};
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback =
      [feedback = feedback_](GoalHandle::SharedPtr, std::shared_ptr<const NavigateToPose::Feedback> update) {
        std::lock_guard<std::mutex> lock(feedback->mutex);
        feedback->latest = std::move(update);
        feedback->received_at = std::chrono::steady_clock::now();
      };
    goal_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);

    BTLogger::log(departure.str());
    running_since_ = std::chrono::steady_clock::now();
    last_logged_minute_ = std::chrono::minutes::zero();
    return BT::NodeStatus::RUNNING;
  }

  // subsequent ticks: poll goal acceptance (phase 1) then result (phase 2)
  BT::NodeStatus onRunning() override
  {
    // phase 1: wait for goal acceptance
    if (goal_future_.valid() &&
        goal_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
    {
      goal_handle_ = goal_future_.get();

      if (!goal_handle_)
      {
        BTLogger::log("NavigateToRoom: FAILURE — goal rejected");
        return BT::NodeStatus::FAILURE;
      }

      // goal accepted — start polling for result
      result_future_ = action_client_->async_get_result(goal_handle_);
      goal_future_ = {};  // invalidate so we don't re-enter this block
    }

    // phase 2: wait for navigation result
    if (result_future_.valid() &&
        result_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
    {
      auto result = result_future_.get();
      result_future_ = {};
      goal_handle_.reset();

      if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
      {
        log_arrival();
        return BT::NodeStatus::SUCCESS;
      }
      BTLogger::log("NavigateToRoom: FAILURE — navigation failed");
      return BT::NodeStatus::FAILURE;
    }

    const auto elapsed = std::chrono::steady_clock::now() - running_since_;
    const auto elapsed_minutes = std::chrono::duration_cast<std::chrono::minutes>(elapsed);
    if (elapsed_minutes > last_logged_minute_)
    {
      const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
      std::ostringstream message;
      message << "Running: " << std::setfill('0')
              << std::setw(2) << seconds / 3600 << ':'
              << std::setw(2) << seconds / 60 % 60 << ':'
              << std::setw(2) << seconds % 60 << " | ";
      append_goal(message);
      append_map_pose(message, "current", false);
      append_progress(message);
      BTLogger::log(message.str(), false);
      last_logged_minute_ = elapsed_minutes;
    }
    return BT::NodeStatus::RUNNING;
  }

  // tree preempted us (e.g. object detected mid-drive) — cancel the Nav2 goal
  void onHalted() override
  {
    if (goal_handle_ && rclcpp::ok())
    {
      try
      {
        action_client_->async_cancel_goal(goal_handle_);
        BTLogger::log("NavigateToRoom: halted — goal canceled");
      }
      catch (const rclcpp_action::exceptions::UnknownGoalHandleError&)
      {
        // Nav2 may have finished after the last tick and before this halt.
      }
    }
    goal_handle_.reset();
    goal_future_ = {};
    result_future_ = {};
  }

private:
  void log_arrival() const
  {
    std::ostringstream message;
    message << "NavigateToRoom: SUCCESS — arrived at ";
    append_goal(message);
    append_map_pose(message, "actual", true);
    BTLogger::log(message.str());
  }

  void append_goal(std::ostream& message) const
  {
    message << std::fixed << std::setprecision(3)
            << label_
            << " | goal map=(" << goal_x_ << ", " << goal_y_ << ") m";
  }

  void append_map_pose(std::ostream& message, const char* pose_label, bool arrival) const
  {
    try
    {
      const auto pose = transforms_->buffer.lookupTransform("map", "base_footprint", tf2::TimePointZero);
      const auto& position = pose.transform.translation;
      const auto& rotation = pose.transform.rotation;
      const double yaw = tf2::getYaw(tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w));
      const double distance = std::hypot(position.x - goal_x_, position.y - goal_y_);
      message << std::setprecision(3) << " | " << pose_label << " map=("
              << position.x << ", " << position.y << ") m"
              << std::setprecision(1) << ", yaw=" << yaw * 180.0 / std::acos(-1.0) << "°";
      if (arrival)
      {
        message << " | XY error=" << 100.0 * distance << " cm";
      }
      else
      {
        message << std::setprecision(2) << " | straight-line distance=" << distance << " m";
      }
    }
    catch (const tf2::TransformException&)
    {
      message << " | " << pose_label << " map pose unavailable";
    }
  }

  void append_progress(std::ostream& message) const
  {
    std::shared_ptr<const NavigateToPose::Feedback> latest;
    {
      std::lock_guard<std::mutex> lock(feedback_->mutex);
      if (std::chrono::steady_clock::now() - feedback_->received_at < std::chrono::seconds(10))
      {
        latest = feedback_->latest;
      }
    }
    if (!latest)
    {
      return;
    }
    if (std::isfinite(latest->distance_remaining) && latest->distance_remaining >= 0.0f)
    {
      message << std::setprecision(2) << " | Nav2 path remaining=" << latest->distance_remaining << " m";
    }
    const auto& eta = latest->estimated_time_remaining;
    const double eta_seconds = eta.sec + eta.nanosec * 1e-9;
    if (eta_seconds > 0.0)
    {
      message << " | ETA=" << std::setprecision(0) << eta_seconds << " s";
    }
    message << " | recoveries=" << latest->number_of_recoveries;
  }

  std::shared_ptr<ArrivalTransforms> transforms_;
  std::shared_ptr<GoalFeedback> feedback_;
  double goal_x_{0.0};
  double goal_y_{0.0};
  std::string label_;
  std::chrono::steady_clock::time_point running_since_;
  std::chrono::minutes last_logged_minute_{0};
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;
};
