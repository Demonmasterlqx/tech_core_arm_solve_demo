// Copyright 2026 pnx
// Licensed under the MIT License.

#pragma once

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace handle_arm
{

struct LoadedTrajectory
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  double duration_sec{0.0};
};

LoadedTrajectory loadTrajectoryFromYamlFile(const std::filesystem::path & path);

class TrajectoryLoopNode : public rclcpp::Node
{
public:
  explicit TrajectoryLoopNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TrajectoryLoopNode() override;

private:
  struct Parameters
  {
    std::string trajectory_file;
    std::string trajectory_topic;
    std::string trigger_topic;
    std::string joint_state_topic;
    double trajectory_completion_extra_wait_sec{0.1};
    double interpolation_time_interval_sec{0.1};
    double interpolation_max_joint_step_rad{0.1};
    double joint_state_timeout_sec{0.2};
  };

  void loadParameters();
  void validateParameters() const;
  void handleTrigger(const std_msgs::msg::Bool::SharedPtr msg);
  void handleJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void workerMain();
  void joinFinishedWorkerIfNeeded();

  bool isActiveRequested() const;
  bool waitInterruptibly(double seconds) const;
  bool waitForCurrentJointPositions(
    const std::vector<std::string> & joint_names,
    std::vector<double> & positions,
    std::string & reason) const;
  bool tryGetLatestJointPositions(
    const std::vector<std::string> & joint_names,
    std::vector<double> & positions,
    std::string & reason) const;
  bool buildInterpolationTrajectory(
    const std::vector<double> & current_positions,
    trajectory_msgs::msg::JointTrajectory & interpolation,
    double & duration_sec,
    std::string & reason) const;
  void publishTrajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::string & stage_name);

  Parameters params_;
  LoadedTrajectory loaded_trajectory_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

  mutable std::mutex state_mutex_;
  mutable std::condition_variable state_cv_;
  bool active_requested_{false};
  bool shutdown_requested_{false};
  bool previous_trigger_state_{false};
  bool worker_running_{false};
  std::thread worker_thread_;

  mutable std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  std::chrono::steady_clock::time_point latest_joint_state_receive_time_{};
  bool has_latest_joint_state_{false};
};

}  // namespace handle_arm
