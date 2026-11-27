// Run a stop's configured executable and relay the log field of its JSON events.
#pragma once

#include <behaviortree_cpp/action_node.h>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <fcntl.h>
#include <filesystem>
#include <map>
#include <memory>
#include <sstream>
#include <spawn.h>
#include <string>
#include <string_view>
#include <sys/wait.h>
#include <system_error>
#include <thread>
#include <unistd.h>

#include "turtlebot3_autonomous_patrol/bt_logger.hpp"

extern char** environ;

class RunPatrolTask : public BT::StatefulActionNode
{
  using Clock = std::chrono::steady_clock;
  struct Task
  {
    std::string name;
    std::string executable;
  };
  struct Catalogue
  {
    std::string path;
    std::map<std::string, Task> tasks;
  };

public:
  RunPatrolTask(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), blackboard_(config.blackboard)
  {
    const auto node = blackboard_->get<rclcpp::Node::SharedPtr>("node");
    BTLogger::init(node);
    if (!blackboard_->get("patrol_task_catalogue", catalogue_))
    {
      const auto filename = node->has_parameter("patrol_tasks_file")
        ? node->get_parameter("patrol_tasks_file").as_string()
        : node->declare_parameter<std::string>("patrol_tasks_file", "");
      catalogue_ = std::make_shared<Catalogue>();
      if (!filename.empty())
      {
        catalogue_->path = std::filesystem::absolute(filename).string();
        const auto stops = YAML::LoadFile(catalogue_->path)["stops"];
        if (!stops.IsMap())
        {
          throw BT::RuntimeError("patrol_tasks_file must contain a stops mapping");
        }
        for (const auto& entry : stops)
        {
          const auto stop = entry.first.as<std::string>();
          const auto task_name = entry.second["name"].as<std::string>();
          std::istringstream command(entry.second["executable"].as<std::string>());
          std::string package, executable, extra;
          if (stop.empty() || task_name.empty() || !(command >> package >> executable) ||
              (command >> extra) || !is_component(package) || !is_component(executable))
          {
            throw BT::RuntimeError("Invalid patrol task at ", stop, ": expected a name and 'package executable'");
          }
          std::filesystem::path prefix;
          ament_index_cpp::get_package_prefix(package, prefix);
          const auto path = prefix / "lib" / package / executable;
          if (::access(path.c_str(), X_OK) != 0 || !std::filesystem::is_regular_file(path))
          {
            throw BT::RuntimeError("Patrol task executable is not available: ", path.string());
          }
          if (!catalogue_->tasks.emplace(stop, Task{task_name, path.string()}).second)
          {
            throw BT::RuntimeError("Duplicate patrol task stop: ", stop);
          }
        }
      }
      blackboard_->set("patrol_task_catalogue", catalogue_);
    }
  }

  ~RunPatrolTask() override
  {
    if (child_ > 0)
    {
      stop_task();
    }
    close_output();
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("stop", "Stop key in the patrol task catalogue")};
  }

  BT::NodeStatus onStart() override
  {
    if (!getInput<std::string>("stop", stop_))
    {
      throw BT::RuntimeError("RunPatrolTask requires a stop");
    }
    const auto entry = catalogue_->tasks.find(stop_);
    if (entry == catalogue_->tasks.end())
    {
      return BT::NodeStatus::SUCCESS;
    }

    task_name_ = entry->second.name;
    try
    {
      start_process(entry->second.executable);
    }
    catch (const std::system_error& error)
    {
      BTLogger::log("RunPatrolTask: could not start " + task_name_ + " — " + error.what());
      return BT::NodeStatus::FAILURE;
    }
    buffer_.clear();
    deadline_ = Clock::now() + std::chrono::minutes(5);
    BTLogger::log("RunPatrolTask: running " + task_name_ + " at " + stop_);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    read_output();
    int status = 0;
    const auto result = ::waitpid(child_, &status, WNOHANG);
    if (result == child_)
    {
      child_ = -1;
      finish_output();
      if (WIFEXITED(status) && WEXITSTATUS(status) == 0)
      {
        return BT::NodeStatus::SUCCESS;
      }
      if (!WIFEXITED(status) || WEXITSTATUS(status) == motion_stop_unconfirmed_exit_code)
      {
        return hold_for_motion_check();
      }
      BTLogger::log("RunPatrolTask: " + task_name_ + (WIFEXITED(status)
        ? " exited with code " + std::to_string(WEXITSTATUS(status))
        : " was terminated by signal " + std::to_string(WTERMSIG(status))));
      return BT::NodeStatus::FAILURE;
    }
    if (result < 0 && errno != EINTR)
    {
      if (errno == ECHILD)
      {
        child_ = -1;
        finish_output();
        return hold_for_motion_check();
      }
      const auto error = std::error_code(errno, std::generic_category()).message();
      if (!stop_task()) return BT::NodeStatus::RUNNING;
      BTLogger::log("RunPatrolTask: could not supervise " + task_name_ + " — " + error);
      return BT::NodeStatus::FAILURE;
    }
    if (Clock::now() >= deadline_)
    {
      BTLogger::log("RunPatrolTask: timed out waiting for " + task_name_);
      return stop_task() ? BT::NodeStatus::FAILURE : BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    stop_task();
  }

private:
  static constexpr int motion_stop_unconfirmed_exit_code = 3;

  static bool is_component(const std::string& name)
  {
    return name != "." && name != ".." && name.find_first_of("/\\") == std::string::npos;
  }

  static void check_system(int error, const char* operation)
  {
    if (error != 0)
    {
      throw std::system_error(error, std::generic_category(), operation);
    }
  }

  void start_process(std::string executable)
  {
    int pipe_fds[2];
    if (::pipe2(pipe_fds, O_CLOEXEC) != 0)
    {
      check_system(errno, "pipe2");
    }

    posix_spawn_file_actions_t actions;
    posix_spawnattr_t attributes;
    bool attributes_ready = false;
    bool actions_ready = false;
    try
    {
      if (::fcntl(pipe_fds[0], F_SETFL, O_NONBLOCK) < 0)
      {
        check_system(errno, "fcntl");
      }
      check_system(posix_spawn_file_actions_init(&actions), "posix_spawn_file_actions_init");
      actions_ready = true;
      check_system(posix_spawn_file_actions_adddup2(&actions, pipe_fds[1], STDOUT_FILENO), "redirect stdout");
      check_system(posix_spawnattr_init(&attributes), "posix_spawnattr_init");
      attributes_ready = true;
      check_system(posix_spawnattr_setpgroup(&attributes, 0), "set process group");
      check_system(posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP), "set spawn flags");
      char* argv[] = {executable.data(), catalogue_->path.data(), stop_.data(), nullptr};
      check_system(posix_spawn(&child_, executable.c_str(), &actions, &attributes, argv, environ),
                   "posix_spawn");
    }
    catch (...)
    {
      if (attributes_ready) posix_spawnattr_destroy(&attributes);
      if (actions_ready) posix_spawn_file_actions_destroy(&actions);
      ::close(pipe_fds[0]);
      ::close(pipe_fds[1]);
      child_ = -1;
      throw;
    }

    posix_spawnattr_destroy(&attributes);
    posix_spawn_file_actions_destroy(&actions);
    ::close(pipe_fds[1]);
    output_ = pipe_fds[0];
  }

  void relay(const std::string& line) const
  {
    if (!rclcpp::ok())
    {
      return;
    }
    const auto event = nlohmann::json::parse(line, nullptr, false);
    if (!event.is_object() || !event.contains("version") || event["version"] != 1 ||
        !event.contains("stop") || event["stop"] != stop_ ||
        !event.contains("event") || !event.contains("log") || !event["log"].is_string())
    {
      return;
    }
    const auto& kind = event["event"];
    if (kind != "patrol_task_start" && kind != "patrol_task_progress" && kind != "patrol_task_finish")
    {
      return;
    }
    std::istringstream lines(event["log"].get<std::string>());
    std::string message;
    while (std::getline(lines, message))
    {
      if (!message.empty())
      {
        BTLogger::log("RunPatrolTask: " + message, false);
      }
    }
  }

  void read_output()
  {
    if (output_ < 0)
    {
      return;
    }
    char chunk[4096];
    // Bound work per BT tick and discard lines that exceed the protocol's buffer.
    for (int i = 0; i < 16; ++i)
    {
      const auto size = ::read(output_, chunk, sizeof(chunk));
      if (size <= 0)
      {
        return;
      }
      for (const char character : std::string_view(chunk, size))
      {
        if (character == '\n')
        {
          if (!discard_line_) relay(buffer_);
          discard_line_ = false;
          buffer_.clear();
        }
        else if (!discard_line_)
        {
          buffer_.push_back(character);
          if (buffer_.size() > 65536)
          {
            discard_line_ = true;
            buffer_.clear();
          }
        }
      }
    }
  }

  void finish_output()
  {
    read_output();
    if (!buffer_.empty())
    {
      relay(buffer_);
    }
    close_output();
  }

  void close_output()
  {
    if (output_ >= 0)
    {
      ::close(output_);
      output_ = -1;
    }
    buffer_.clear();
    discard_line_ = false;
  }

  bool wait_for_exit(Clock::duration timeout, int& status)
  {
    const auto deadline = Clock::now() + timeout;
    do
    {
      read_output();
      const auto result = ::waitpid(child_, &status, WNOHANG);
      if (result == child_)
      {
        child_ = -1;
        return true;
      }
      if (result < 0 && errno != EINTR)
      {
        if (errno == ECHILD) child_ = -1;
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    } while (Clock::now() < deadline);
    return false;
  }

  BT::NodeStatus hold_for_motion_check()
  {
    blackboard_->set("patrol_task_stop_unconfirmed", true);
    if (rclcpp::ok())
    {
      BTLogger::log("RunPatrolTask: task stop was not confirmed; check Seggy before restarting patrol");
    }
    return BT::NodeStatus::RUNNING;
  }

  bool stop_task()
  {
    if (child_ <= 0)
    {
      return true;
    }
    // The rotation script handles SIGTERM and waits up to 3 seconds for wheel stop.
    ::kill(-child_, SIGTERM);
    int status = 0;
    const bool stopped = wait_for_exit(std::chrono::seconds(4), status) &&
      WIFEXITED(status) && (WEXITSTATUS(status) == 0 || WEXITSTATUS(status) == 130);
    if (!stopped)
    {
      if (child_ > 0)
      {
        ::kill(-child_, SIGKILL);
        wait_for_exit(std::chrono::seconds(1), status);
      }
      hold_for_motion_check();
    }
    finish_output();
    return stopped;
  }

  BT::Blackboard::Ptr blackboard_;
  std::shared_ptr<Catalogue> catalogue_;
  std::string stop_, task_name_, buffer_;
  bool discard_line_{false};
  Clock::time_point deadline_{};
  pid_t child_{-1};
  int output_{-1};
};
