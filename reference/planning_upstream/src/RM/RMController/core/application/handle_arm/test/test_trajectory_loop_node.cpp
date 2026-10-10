// Copyright 2026 pnx
// Licensed under the MIT License.

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "handle_arm/trajectory_loop_node.hpp"

namespace handle_arm
{
namespace
{

using namespace std::chrono_literals;

std::filesystem::path makeTempDir()
{
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  auto dir = std::filesystem::temp_directory_path() /
    ("handle_arm_test_" + std::to_string(stamp));
  std::filesystem::create_directories(dir);
  return dir;
}

void writeFile(const std::filesystem::path & path, const std::string & content)
{
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("Failed to open test file for writing: " + path.string());
  }
  output << content;
}

std::string validTrajectoryYaml(const double first_position = 1.0, const double last_time = 0.03)
{
  std::ostringstream oss;
  oss << "format_version: 1\n"
      << "joint_names: [j0, j1]\n"
      << "source_topic: /joint_states\n"
      << "recorded_at: '2026-06-01T00:00:00.000Z'\n"
      << "points:\n"
      << "  - time_from_start: 0.0\n"
      << "    positions: [" << first_position << ", " << first_position + 1.0 << "]\n"
      << "    velocities: [0.0, 0.0]\n"
      << "  - time_from_start: " << last_time << "\n"
      << "    positions: [" << first_position + 2.0 << ", " << first_position + 3.0 << "]\n";
  return oss.str();
}

bool waitFor(const std::function<bool()> & predicate, const std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return predicate();
}

TEST(TrajectoryYamlLoaderTest, LoadsValidTrajectory)
{
  const auto temp_dir = makeTempDir();
  const auto trajectory_file = temp_dir / "valid.yaml";
  writeFile(trajectory_file, validTrajectoryYaml());

  const auto loaded = loadTrajectoryFromYamlFile(trajectory_file);

  EXPECT_DOUBLE_EQ(0.03, loaded.duration_sec);
  ASSERT_EQ(2U, loaded.trajectory.joint_names.size());
  EXPECT_EQ("j0", loaded.trajectory.joint_names[0]);
  ASSERT_EQ(2U, loaded.trajectory.points.size());
  EXPECT_DOUBLE_EQ(1.0, loaded.trajectory.points.front().positions[0]);

  std::error_code ec;
  std::filesystem::remove_all(temp_dir, ec);
}

TEST(TrajectoryYamlLoaderTest, RejectsInvalidTrajectoryFiles)
{
  const auto temp_dir = makeTempDir();

  const auto missing_joint_names = temp_dir / "missing_joint_names.yaml";
  writeFile(missing_joint_names, "points:\n  - time_from_start: 0\n    positions: [0]\n");
  EXPECT_THROW(loadTrajectoryFromYamlFile(missing_joint_names), std::runtime_error);

  const auto empty_points = temp_dir / "empty_points.yaml";
  writeFile(empty_points, "joint_names: [j0]\npoints: []\n");
  EXPECT_THROW(loadTrajectoryFromYamlFile(empty_points), std::runtime_error);

  const auto non_monotonic = temp_dir / "non_monotonic.yaml";
  writeFile(
    non_monotonic,
    "joint_names: [j0]\npoints:\n"
    "  - time_from_start: 1.0\n    positions: [0.0]\n"
    "  - time_from_start: 0.5\n    positions: [1.0]\n");
  EXPECT_THROW(loadTrajectoryFromYamlFile(non_monotonic), std::runtime_error);

  const auto bad_positions = temp_dir / "bad_positions.yaml";
  writeFile(
    bad_positions,
    "joint_names: [j0, j1]\npoints:\n  - time_from_start: 0\n    positions: [0]\n");
  EXPECT_THROW(loadTrajectoryFromYamlFile(bad_positions), std::runtime_error);

  const auto bad_velocities = temp_dir / "bad_velocities.yaml";
  writeFile(
    bad_velocities,
    "joint_names: [j0, j1]\npoints:\n"
    "  - time_from_start: 0\n    positions: [0, 0]\n    velocities: [0]\n");
  EXPECT_THROW(loadTrajectoryFromYamlFile(bad_velocities), std::runtime_error);

  EXPECT_THROW(loadTrajectoryFromYamlFile(std::filesystem::path()), std::runtime_error);

  std::error_code ec;
  std::filesystem::remove_all(temp_dir, ec);
}

class TrajectoryLoopNodeTest : public ::testing::Test
{
protected:
  struct TrajectoryEvent
  {
    std::vector<std::string> joint_names;
    std::vector<double> first_positions;
    std::vector<double> last_positions;
    std::size_t points_size{0U};
  };

  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    temp_dir_ = makeTempDir();
    trajectory_file_ = temp_dir_ / "trajectory.yaml";
    writeFile(trajectory_file_, validTrajectoryYaml(1.0, 0.03));

    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    topic_prefix_ = "/handle_arm_test_" + std::to_string(stamp);
    trajectory_topic_ = topic_prefix_ + "/trajectory";
    trigger_topic_ = topic_prefix_ + "/toggle";
    joint_state_topic_ = topic_prefix_ + "/joint_states";
  }

  void TearDown() override
  {
    stopSystem();
    if (!temp_dir_.empty()) {
      std::error_code ec;
      std::filesystem::remove_all(temp_dir_, ec);
    }
  }

  void startSystem(const double joint_state_timeout_sec = 0.1)
  {
    helper_node_ = std::make_shared<rclcpp::Node>("handle_arm_test_helper");

    trajectory_subscription_ =
      helper_node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      trajectory_topic_,
      rclcpp::QoS(10),
      [this](trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        TrajectoryEvent event;
        event.joint_names = msg->joint_names;
        event.points_size = msg->points.size();
        if (!msg->points.empty()) {
          event.first_positions = msg->points.front().positions;
          event.last_positions = msg->points.back().positions;
        }

        std::lock_guard<std::mutex> lock(events_mutex_);
        events_.push_back(std::move(event));
      });

    trigger_publisher_ = helper_node_->create_publisher<std_msgs::msg::Bool>(
      trigger_topic_,
      rclcpp::QoS(10));
    joint_state_publisher_ = helper_node_->create_publisher<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      rclcpp::SensorDataQoS());

    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.parameter_overrides({
      rclcpp::Parameter("trajectory_file", trajectory_file_.string()),
      rclcpp::Parameter("trajectory_topic", trajectory_topic_),
      rclcpp::Parameter("trigger_topic", trigger_topic_),
      rclcpp::Parameter("joint_state_topic", joint_state_topic_),
      rclcpp::Parameter("trajectory_completion_extra_wait_sec", 0.02),
      rclcpp::Parameter("interpolation_time_interval_sec", 0.01),
      rclcpp::Parameter("interpolation_max_joint_step_rad", 0.5),
      rclcpp::Parameter("joint_state_timeout_sec", joint_state_timeout_sec),
    });
    dut_node_ = std::make_shared<TrajectoryLoopNode>(options);

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(),
      2U);
    executor_->add_node(helper_node_);
    executor_->add_node(dut_node_);
    executor_thread_ = std::thread([this]() {executor_->spin();});

    ASSERT_TRUE(waitFor(
      [this]() {
        return dut_node_->count_subscribers(trajectory_topic_) > 0 &&
               trigger_publisher_->get_subscription_count() > 0 &&
               joint_state_publisher_->get_subscription_count() > 0;
      },
      1s));
  }

  void stopSystem()
  {
    if (executor_) {
      executor_->cancel();
    }
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    if (executor_) {
      if (helper_node_) {
        executor_->remove_node(helper_node_);
      }
      if (dut_node_) {
        executor_->remove_node(dut_node_);
      }
    }

    trajectory_subscription_.reset();
    trigger_publisher_.reset();
    joint_state_publisher_.reset();
    helper_node_.reset();
    dut_node_.reset();
    executor_.reset();
  }

  void publishBool(const bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    trigger_publisher_->publish(msg);
  }

  void publishJointState(const std::vector<double> & positions)
  {
    sensor_msgs::msg::JointState msg;
    msg.name = {"j0", "j1"};
    msg.position = positions;
    joint_state_publisher_->publish(msg);
  }

  std::vector<TrajectoryEvent> events() const
  {
    std::lock_guard<std::mutex> lock(events_mutex_);
    return events_;
  }

  bool waitForEventCount(const std::size_t count, const std::chrono::milliseconds timeout)
  {
    return waitFor([this, count]() {return events().size() >= count;}, timeout);
  }

  std::filesystem::path temp_dir_;
  std::filesystem::path trajectory_file_;
  std::string topic_prefix_;
  std::string trajectory_topic_;
  std::string trigger_topic_;
  std::string joint_state_topic_;

  std::shared_ptr<rclcpp::Node> helper_node_;
  std::shared_ptr<TrajectoryLoopNode> dut_node_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread executor_thread_;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

  mutable std::mutex events_mutex_;
  std::vector<TrajectoryEvent> events_;
};

TEST_F(TrajectoryLoopNodeTest, PublishesStartupInterpolationThenLoops)
{
  startSystem();
  publishJointState({0.0, 0.0});
  publishBool(true);

  ASSERT_TRUE(waitForEventCount(2U, 1s));
  const auto received = events();

  ASSERT_GE(received.size(), 2U);
  EXPECT_EQ((std::vector<std::string>{"j0", "j1"}), received[0].joint_names);
  ASSERT_GE(received[0].points_size, 2U);
  EXPECT_EQ((std::vector<double>{0.0, 0.0}), received[0].first_positions);
  EXPECT_EQ((std::vector<double>{1.0, 2.0}), received[0].last_positions);

  EXPECT_EQ(2U, received[1].points_size);
  EXPECT_EQ((std::vector<double>{1.0, 2.0}), received[1].first_positions);
  EXPECT_EQ((std::vector<double>{3.0, 4.0}), received[1].last_positions);
}

TEST_F(TrajectoryLoopNodeTest, RisingEdgeTogglesStartAndStop)
{
  startSystem();

  publishBool(false);
  std::this_thread::sleep_for(80ms);
  EXPECT_TRUE(events().empty());

  publishJointState({1.0, 2.0});
  publishBool(true);
  ASSERT_TRUE(waitForEventCount(1U, 1s));

  publishBool(true);
  ASSERT_TRUE(waitForEventCount(2U, 1s));

  const auto count_before_stop = events().size();
  publishBool(false);
  std::this_thread::sleep_for(20ms);
  publishBool(true);
  std::this_thread::sleep_for(120ms);
  EXPECT_EQ(count_before_stop, events().size());
}

TEST_F(TrajectoryLoopNodeTest, MissingJointStateRefusesStartup)
{
  startSystem(0.05);
  publishBool(true);

  std::this_thread::sleep_for(150ms);
  EXPECT_TRUE(events().empty());
}

}  // namespace
}  // namespace handle_arm
