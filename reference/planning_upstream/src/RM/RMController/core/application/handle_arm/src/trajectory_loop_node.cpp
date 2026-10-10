// Copyright 2026 pnx
// Licensed under the MIT License.

#include "handle_arm/trajectory_loop_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace handle_arm
{
namespace
{

using SteadyClock = std::chrono::steady_clock;
constexpr auto kPollPeriod = std::chrono::milliseconds(10);

class ScopedExit
{
public:
  explicit ScopedExit(std::function<void()> fn)
  : fn_(std::move(fn))
  {
  }

  ~ScopedExit()
  {
    if (fn_) {
      fn_();
    }
  }

private:
  std::function<void()> fn_;
};

SteadyClock::duration secondsToSteadyDuration(const double seconds)
{
  return std::chrono::duration_cast<SteadyClock::duration>(
    std::chrono::duration<double>(seconds));
}

SteadyClock::duration clampWaitDuration(
  const SteadyClock::time_point deadline,
  const SteadyClock::duration maximum_wait)
{
  const auto now = SteadyClock::now();
  if (now >= deadline) {
    return SteadyClock::duration::zero();
  }
  return std::min(deadline - now, maximum_wait);
}

double readFiniteDouble(
  const YAML::Node & node,
  const std::string & context,
  const std::string & field)
{
  double value = 0.0;
  try {
    value = node.as<double>();
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(context + " field '" + field + "' is not a double: " + e.what());
  }
  if (!std::isfinite(value)) {
    throw std::runtime_error(context + " field '" + field + "' is not finite");
  }
  return value;
}

}  // namespace

LoadedTrajectory loadTrajectoryFromYamlFile(const std::filesystem::path & raw_path)
{
  if (raw_path.empty()) {
    throw std::runtime_error("Parameter 'trajectory_file' must not be empty");
  }

  const std::filesystem::path path =
    raw_path.is_absolute() ? raw_path.lexically_normal() :
    std::filesystem::absolute(raw_path).lexically_normal();

  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("Trajectory file does not exist: '" + path.string() + "'");
  }
  if (!std::filesystem::is_regular_file(path)) {
    throw std::runtime_error("Trajectory path is not a regular file: '" + path.string() + "'");
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(path.string());
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(
      "Failed to parse trajectory YAML '" + path.string() + "': " + e.what());
  }

  const YAML::Node joint_names_node = root["joint_names"];
  if (!joint_names_node || !joint_names_node.IsSequence() || joint_names_node.size() == 0U) {
    throw std::runtime_error(
      "Trajectory file '" + path.string() + "' must have non-empty 'joint_names'");
  }

  std::vector<std::string> joint_names;
  joint_names.reserve(joint_names_node.size());
  for (std::size_t i = 0; i < joint_names_node.size(); ++i) {
    if (!joint_names_node[i].IsScalar()) {
      throw std::runtime_error(
        "Trajectory file '" + path.string() + "': joint_names[" +
        std::to_string(i) + "] must be string");
    }
    const auto name = joint_names_node[i].as<std::string>();
    if (name.empty()) {
      throw std::runtime_error(
        "Trajectory file '" + path.string() + "': joint_names[" +
        std::to_string(i) + "] must not be empty");
    }
    joint_names.push_back(name);
  }

  const YAML::Node points_node = root["points"];
  if (!points_node || !points_node.IsSequence() || points_node.size() == 0U) {
    throw std::runtime_error(
      "Trajectory file '" + path.string() + "' must have non-empty 'points'");
  }

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = joint_names;

  double previous_time = -std::numeric_limits<double>::infinity();
  double last_time = 0.0;

  for (std::size_t i = 0; i < points_node.size(); ++i) {
    const YAML::Node point_node = points_node[i];
    const std::string point_context =
      "Trajectory file '" + path.string() + "' points[" + std::to_string(i) + "]";

    const YAML::Node time_node = point_node["time_from_start"];
    if (!time_node || !time_node.IsScalar()) {
      throw std::runtime_error(point_context + " missing scalar field 'time_from_start'");
    }
    const double time_from_start =
      readFiniteDouble(time_node, point_context, "time_from_start");
    if (time_from_start < 0.0) {
      throw std::runtime_error(point_context + " has negative 'time_from_start'");
    }
    if (time_from_start < previous_time) {
      throw std::runtime_error(point_context + " has non-monotonic 'time_from_start'");
    }

    const YAML::Node positions_node = point_node["positions"];
    if (!positions_node || !positions_node.IsSequence()) {
      throw std::runtime_error(point_context + " missing sequence field 'positions'");
    }
    if (positions_node.size() != joint_names.size()) {
      std::ostringstream oss;
      oss << point_context << " has positions size " << positions_node.size()
          << " but expected " << joint_names.size();
      throw std::runtime_error(oss.str());
    }

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
    point.positions.reserve(positions_node.size());
    for (std::size_t j = 0; j < positions_node.size(); ++j) {
      point.positions.push_back(
        readFiniteDouble(
          positions_node[j],
          point_context,
          "positions[" + std::to_string(j) + "]"));
    }

    const YAML::Node velocities_node = point_node["velocities"];
    if (velocities_node) {
      if (!velocities_node.IsSequence()) {
        throw std::runtime_error(point_context + " field 'velocities' must be sequence");
      }
      if (velocities_node.size() != joint_names.size()) {
        std::ostringstream oss;
        oss << point_context << " has velocities size " << velocities_node.size()
            << " but expected " << joint_names.size();
        throw std::runtime_error(oss.str());
      }

      point.velocities.reserve(velocities_node.size());
      for (std::size_t j = 0; j < velocities_node.size(); ++j) {
        point.velocities.push_back(
          readFiniteDouble(
            velocities_node[j],
            point_context,
            "velocities[" + std::to_string(j) + "]"));
      }
    }

    trajectory.points.push_back(std::move(point));
    previous_time = time_from_start;
    last_time = time_from_start;
  }

  return {std::move(trajectory), last_time};
}

TrajectoryLoopNode::TrajectoryLoopNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("handle_arm_trajectory_loop", options)
{
  loadParameters();
  validateParameters();
  loaded_trajectory_ = loadTrajectoryFromYamlFile(params_.trajectory_file);

  trajectory_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    params_.trajectory_topic,
    rclcpp::QoS(10));

  trigger_subscription_ = create_subscription<std_msgs::msg::Bool>(
    params_.trigger_topic,
    rclcpp::QoS(10),
    std::bind(&TrajectoryLoopNode::handleTrigger, this, std::placeholders::_1));

  joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
    params_.joint_state_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&TrajectoryLoopNode::handleJointState, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "handle_arm trajectory loop ready: trajectory_file=%s, points=%zu, duration=%.3f s, "
    "trajectory_topic=%s, trigger_topic=%s, joint_state_topic=%s",
    params_.trajectory_file.c_str(),
    loaded_trajectory_.trajectory.points.size(),
    loaded_trajectory_.duration_sec,
    params_.trajectory_topic.c_str(),
    params_.trigger_topic.c_str(),
    params_.joint_state_topic.c_str());
}

TrajectoryLoopNode::~TrajectoryLoopNode()
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    active_requested_ = false;
    shutdown_requested_ = true;
  }
  state_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void TrajectoryLoopNode::loadParameters()
{
  params_.trajectory_file = declare_parameter<std::string>("trajectory_file", "");
  params_.trajectory_topic = declare_parameter<std::string>(
    "trajectory_topic",
    "/arm_control_group_controller/joint_trajectory");
  params_.trigger_topic = declare_parameter<std::string>("trigger_topic", "~/toggle");
  params_.joint_state_topic = declare_parameter<std::string>("joint_state_topic", "/joint_states");
  params_.trajectory_completion_extra_wait_sec =
    declare_parameter<double>("trajectory_completion_extra_wait_sec", 0.1);
  params_.interpolation_time_interval_sec =
    declare_parameter<double>("interpolation_time_interval_sec", 0.1);
  params_.interpolation_max_joint_step_rad =
    declare_parameter<double>("interpolation_max_joint_step_rad", 0.1);
  params_.joint_state_timeout_sec = declare_parameter<double>("joint_state_timeout_sec", 0.2);
}

void TrajectoryLoopNode::validateParameters() const
{
  auto requireNonEmpty = [](const std::string & value, const char * name) {
      if (value.empty()) {
        throw std::invalid_argument(std::string("Parameter '") + name + "' must not be empty");
      }
    };

  requireNonEmpty(params_.trajectory_file, "trajectory_file");
  requireNonEmpty(params_.trajectory_topic, "trajectory_topic");
  requireNonEmpty(params_.trigger_topic, "trigger_topic");
  requireNonEmpty(params_.joint_state_topic, "joint_state_topic");

  if (params_.trajectory_completion_extra_wait_sec < 0.0) {
    throw std::invalid_argument("Parameter 'trajectory_completion_extra_wait_sec' must be >= 0");
  }
  if (params_.interpolation_time_interval_sec <= 0.0) {
    throw std::invalid_argument("Parameter 'interpolation_time_interval_sec' must be > 0");
  }
  if (params_.interpolation_max_joint_step_rad <= 0.0) {
    throw std::invalid_argument("Parameter 'interpolation_max_joint_step_rad' must be > 0");
  }
  if (params_.joint_state_timeout_sec < 0.0) {
    throw std::invalid_argument("Parameter 'joint_state_timeout_sec' must be >= 0");
  }
}

void TrajectoryLoopNode::handleTrigger(const std_msgs::msg::Bool::SharedPtr msg)
{
  joinFinishedWorkerIfNeeded();

  bool start_worker = false;
  bool stop_worker = false;
  bool ignored_start = false;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool is_rising_edge = msg->data && !previous_trigger_state_;
    previous_trigger_state_ = msg->data;
    if (!is_rising_edge) {
      return;
    }

    if (active_requested_) {
      active_requested_ = false;
      stop_worker = true;
    } else if (worker_running_) {
      ignored_start = true;
    } else {
      active_requested_ = true;
      worker_running_ = true;
      worker_thread_ = std::thread([this]() {workerMain();});
      start_worker = true;
    }
  }

  if (stop_worker) {
    state_cv_.notify_all();
    RCLCPP_INFO(get_logger(), "Trajectory loop stop requested by rising edge");
  } else if (start_worker) {
    RCLCPP_INFO(get_logger(), "Trajectory loop start requested by rising edge");
  } else if (ignored_start) {
    RCLCPP_WARN(
      get_logger(),
      "Ignored start toggle because previous trajectory loop worker is still stopping");
  }
}

void TrajectoryLoopNode::handleJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    latest_joint_state_ = *msg;
    latest_joint_state_receive_time_ = SteadyClock::now();
    has_latest_joint_state_ = true;
  }
  state_cv_.notify_all();
}

void TrajectoryLoopNode::workerMain()
{
  ScopedExit finish_worker([this]() {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        active_requested_ = false;
        worker_running_ = false;
      }
      state_cv_.notify_all();
    });

  try {
    std::vector<double> current_positions;
    std::string reason;
    if (!waitForCurrentJointPositions(
        loaded_trajectory_.trajectory.joint_names,
        current_positions,
        reason))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Refusing to start trajectory loop: %s",
        reason.c_str());
      return;
    }

    trajectory_msgs::msg::JointTrajectory interpolation;
    double interpolation_duration_sec = 0.0;
    if (buildInterpolationTrajectory(
        current_positions,
        interpolation,
        interpolation_duration_sec,
        reason))
    {
      publishTrajectory(interpolation, "startup_interpolation");
      const double wait_sec =
        interpolation_duration_sec + params_.trajectory_completion_extra_wait_sec;
      if (!waitInterruptibly(wait_sec)) {
        RCLCPP_INFO(get_logger(), "Trajectory loop stopped during startup interpolation");
        return;
      }
    } else {
      RCLCPP_INFO(get_logger(), "Startup interpolation skipped: %s", reason.c_str());
    }

    while (isActiveRequested() && rclcpp::ok()) {
      publishTrajectory(loaded_trajectory_.trajectory, "loop");
      const double wait_sec =
        loaded_trajectory_.duration_sec + params_.trajectory_completion_extra_wait_sec;
      if (!waitInterruptibly(wait_sec)) {
        break;
      }
    }

    RCLCPP_INFO(get_logger(), "Trajectory loop worker stopped");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Trajectory loop worker failed: %s", e.what());
  }
}

void TrajectoryLoopNode::joinFinishedWorkerIfNeeded()
{
  std::thread thread_to_join;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!worker_running_ && worker_thread_.joinable()) {
      thread_to_join = std::move(worker_thread_);
    }
  }

  if (thread_to_join.joinable()) {
    thread_to_join.join();
  }
}

bool TrajectoryLoopNode::isActiveRequested() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return active_requested_ && !shutdown_requested_;
}

bool TrajectoryLoopNode::waitInterruptibly(const double seconds) const
{
  std::unique_lock<std::mutex> lock(state_mutex_);
  if (seconds <= 0.0) {
    return active_requested_ && !shutdown_requested_ && rclcpp::ok();
  }

  const auto deadline = SteadyClock::now() + secondsToSteadyDuration(seconds);
  while (active_requested_ && !shutdown_requested_ && rclcpp::ok()) {
    if (SteadyClock::now() >= deadline) {
      break;
    }
    state_cv_.wait_until(lock, deadline);
  }

  return active_requested_ && !shutdown_requested_ && rclcpp::ok();
}

bool TrajectoryLoopNode::waitForCurrentJointPositions(
  const std::vector<std::string> & joint_names,
  std::vector<double> & positions,
  std::string & reason) const
{
  if (tryGetLatestJointPositions(joint_names, positions, reason)) {
    return true;
  }

  const auto deadline = SteadyClock::now() +
    secondsToSteadyDuration(params_.joint_state_timeout_sec);

  while (isActiveRequested() && rclcpp::ok() && SteadyClock::now() < deadline) {
    const auto wait_duration = clampWaitDuration(deadline, kPollPeriod);
    if (wait_duration == SteadyClock::duration::zero()) {
      break;
    }

    {
      std::unique_lock<std::mutex> lock(state_mutex_);
      state_cv_.wait_for(lock, wait_duration);
    }

    if (tryGetLatestJointPositions(joint_names, positions, reason)) {
      return true;
    }
  }

  if (!rclcpp::ok()) {
    reason = "ROS shutdown while waiting for JointState";
  } else if (!isActiveRequested()) {
    reason = "start cancelled while waiting for JointState";
  }
  return false;
}

bool TrajectoryLoopNode::tryGetLatestJointPositions(
  const std::vector<std::string> & joint_names,
  std::vector<double> & positions,
  std::string & reason) const
{
  sensor_msgs::msg::JointState latest_joint_state;
  SteadyClock::time_point receive_time;
  bool has_latest_joint_state = false;
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    latest_joint_state = latest_joint_state_;
    receive_time = latest_joint_state_receive_time_;
    has_latest_joint_state = has_latest_joint_state_;
  }

  positions.clear();

  if (!has_latest_joint_state) {
    reason = "no JointState has been received on topic '" + params_.joint_state_topic + "'";
    return false;
  }
  if (latest_joint_state.name.empty()) {
    reason = "latest JointState has empty name field";
    return false;
  }
  if (latest_joint_state.position.empty()) {
    reason = "latest JointState has empty position field";
    return false;
  }

  if (params_.joint_state_timeout_sec > 0.0) {
    const double age_sec =
      std::chrono::duration<double>(SteadyClock::now() - receive_time).count();
    if (age_sec > params_.joint_state_timeout_sec) {
      std::ostringstream oss;
      oss << "latest JointState is stale: age=" << age_sec
          << " s, timeout=" << params_.joint_state_timeout_sec << " s";
      reason = oss.str();
      return false;
    }
  }

  std::unordered_map<std::string, std::size_t> index_by_name;
  index_by_name.reserve(latest_joint_state.name.size());
  for (std::size_t i = 0; i < latest_joint_state.name.size(); ++i) {
    if (!latest_joint_state.name[i].empty()) {
      index_by_name.emplace(latest_joint_state.name[i], i);
    }
  }

  positions.reserve(joint_names.size());
  for (const auto & joint_name : joint_names) {
    const auto it = index_by_name.find(joint_name);
    if (it == index_by_name.end()) {
      reason = "latest JointState is missing required joint '" + joint_name + "'";
      positions.clear();
      return false;
    }

    const std::size_t source_index = it->second;
    if (source_index >= latest_joint_state.position.size()) {
      reason = "latest JointState position field is missing joint '" + joint_name + "'";
      positions.clear();
      return false;
    }

    const double position = latest_joint_state.position[source_index];
    if (!std::isfinite(position)) {
      reason = "latest JointState position for joint '" + joint_name + "' is not finite";
      positions.clear();
      return false;
    }
    positions.push_back(position);
  }

  return true;
}

bool TrajectoryLoopNode::buildInterpolationTrajectory(
  const std::vector<double> & current_positions,
  trajectory_msgs::msg::JointTrajectory & interpolation,
  double & duration_sec,
  std::string & reason) const
{
  duration_sec = 0.0;
  interpolation = trajectory_msgs::msg::JointTrajectory();

  const auto & target_trajectory = loaded_trajectory_.trajectory;
  if (target_trajectory.points.empty()) {
    reason = "loaded trajectory has no points";
    return false;
  }

  const auto & joint_names = target_trajectory.joint_names;
  const auto & target_positions = target_trajectory.points.front().positions;
  if (current_positions.size() != joint_names.size()) {
    reason = "current position size does not match target joint count";
    return false;
  }
  if (target_positions.size() != joint_names.size()) {
    reason = "target first point position size does not match joint count";
    return false;
  }

  double max_delta = 0.0;
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    if (!std::isfinite(target_positions[i])) {
      reason = "target first point position for joint '" + joint_names[i] + "' is not finite";
      return false;
    }
    max_delta = std::max(max_delta, std::abs(target_positions[i] - current_positions[i]));
  }

  if (max_delta <= std::numeric_limits<double>::epsilon()) {
    reason = "current position already matches trajectory start point";
    return false;
  }

  const auto step_count = static_cast<std::size_t>(
    std::ceil(max_delta / params_.interpolation_max_joint_step_rad));
  if (step_count == 0U) {
    reason = "interpolation step count is zero";
    return false;
  }

  interpolation.joint_names = joint_names;
  interpolation.points.reserve(step_count + 1U);
  for (std::size_t step = 0; step <= step_count; ++step) {
    const double ratio = static_cast<double>(step) / static_cast<double>(step_count);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.reserve(joint_names.size());
    for (std::size_t joint_index = 0; joint_index < joint_names.size(); ++joint_index) {
      const double start = current_positions[joint_index];
      const double target = target_positions[joint_index];
      point.positions.push_back(start + (target - start) * ratio);
    }
    point.velocities.assign(joint_names.size(), 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(
      static_cast<double>(step) * params_.interpolation_time_interval_sec);
    interpolation.points.push_back(std::move(point));
  }

  duration_sec = static_cast<double>(step_count) * params_.interpolation_time_interval_sec;
  return true;
}

void TrajectoryLoopNode::publishTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::string & stage_name)
{
  auto message = trajectory;
  message.header.stamp = now();
  trajectory_publisher_->publish(message);

  RCLCPP_INFO(
    get_logger(),
    "Published %s trajectory to %s: points=%zu",
    stage_name.c_str(),
    params_.trajectory_topic.c_str(),
    message.points.size());
}

}  // namespace handle_arm
