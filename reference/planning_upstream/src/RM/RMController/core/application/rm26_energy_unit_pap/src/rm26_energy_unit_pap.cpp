// Copyright 2026 pnx
// Licensed under the MIT License.

#include "rm26_energy_unit_pap/rm26_energy_unit_pap.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/create_service.hpp>
#include <rclcpp/expand_topic_or_service_name.hpp>
#include <yaml-cpp/yaml.h>

namespace rm26_energy_unit_pap
{
namespace
{

using SteadyClock = std::chrono::steady_clock;
constexpr auto kLocalWaitPollPeriod = std::chrono::milliseconds(50);

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

double elapsedMs(const SteadyClock::time_point start_time)
{
    return std::chrono::duration<double, std::milli>(
               SteadyClock::now() - start_time)
        .count();
}

std::string ensureMessage(const std::string & fallback, std::string message)
{
    if (message.empty()) {
        return fallback;
    }
    return message;
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

std::size_t publisherSubscriptionCount(
    const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & publisher)
{
    return publisher->get_subscription_count() + publisher->get_intra_process_subscription_count();
}

}  // namespace

LoadedTrajectory loadTrajectoryFromYamlFile(const std::filesystem::path & path)
{
    if (path.empty()) {
        throw std::runtime_error("Trajectory file path must not be empty");
    }
    if (!path.is_absolute()) {
        throw std::runtime_error(
            "Trajectory file path must be absolute: '" + path.string() + "'");
    }
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
    if (!joint_names_node || !joint_names_node.IsSequence() || joint_names_node.size() == 0) {
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
    if (!points_node || !points_node.IsSequence() || points_node.size() == 0) {
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
        const double time_from_start = time_node.as<double>();
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
            point.positions.push_back(positions_node[j].as<double>());
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
                point.velocities.push_back(velocities_node[j].as<double>());
            }
        }

        trajectory.points.push_back(std::move(point));
        previous_time = time_from_start;
        last_time = time_from_start;
    }

    return {std::move(trajectory), last_time};
}

Rm26EnergyUnitPap::Rm26EnergyUnitPap(const rclcpp::NodeOptions & options)
: rclcpp::Node("rm26_energy_unit_pap", options)
{
    param_listener_ = std::make_shared<rm26_energy_unit_pap::ParamListener>(
        this->get_node_parameters_interface());
    params_ = param_listener_->get_params();

    validateParameters();
    loadTrajectories();

    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        params_.trajectory_topic,
        rclcpp::QoS(10));

    open_gripper_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        params_.open_gripper_topic,
        rclcpp::QoS(10));

    close_gripper_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        params_.close_gripper_topic,
        rclcpp::QoS(10));

    interrupt_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        params_.interrupt_topic,
        rclcpp::QoS(10),
        std::bind(&Rm26EnergyUnitPap::handleInterrupt, this, std::placeholders::_1));

    joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        params_.joint_state_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&Rm26EnergyUnitPap::handleJointState, this, std::placeholders::_1));

    using std::placeholders::_1;
    using std::placeholders::_2;

    store_pipeline_service_ = rclcpp::create_service<Trigger>(
        this->get_node_base_interface(),
        this->get_node_services_interface(),
        params_.store_pipeline_service_name,
        std::bind(&Rm26EnergyUnitPap::handleStoreEnergyUnit, this, _1, _2),
        rclcpp::ServicesQoS(),
        callback_group_);

    take_pipeline_service_ = rclcpp::create_service<Trigger>(
        this->get_node_base_interface(),
        this->get_node_services_interface(),
        params_.take_pipeline_service_name,
        std::bind(&Rm26EnergyUnitPap::handleTakeEnergyUnit, this, _1, _2),
        rclcpp::ServicesQoS(),
        callback_group_);

    RCLCPP_INFO(
        get_logger(),
        "rm26_energy_unit_pap ready: store=%s, take=%s, trajectory_topic=%s, "
        "joint_state_topic=%s, open_gripper_topic=%s, close_gripper_topic=%s, "
        "interrupt_topic=%s, interpolation=%s, interpolation_interval=%.3f s, "
        "interpolation_max_joint_step=%.3f rad, joint_state_timeout=%.3f s",
        params_.store_pipeline_service_name.c_str(),
        params_.take_pipeline_service_name.c_str(),
        params_.trajectory_topic.c_str(),
        params_.joint_state_topic.c_str(),
        params_.open_gripper_topic.c_str(),
        params_.close_gripper_topic.c_str(),
        params_.interrupt_topic.c_str(),
        params_.enable_trajectory_interpolation ? "enabled" : "disabled",
        params_.trajectory_interpolation_time_interval_sec,
        params_.trajectory_interpolation_max_joint_step_rad,
        params_.trajectory_interpolation_joint_state_timeout_sec);
}

void Rm26EnergyUnitPap::validateParameters() const
{
    auto requireNonEmpty = [](const std::string & value, const char * name) {
            if (value.empty()) {
                throw std::invalid_argument(std::string("Parameter '") + name + "' must not be empty");
            }
        };

    auto requireDistinctNames = [](const auto & named_values, const char * category) {
            for (std::size_t first_index = 0; first_index < named_values.size(); ++first_index) {
                for (std::size_t second_index = first_index + 1;
                    second_index < named_values.size();
                    ++second_index)
                {
                    if (named_values[first_index].second == named_values[second_index].second) {
                        throw std::invalid_argument(
                            std::string("Parameters '") + named_values[first_index].first +
                            "' and '" + named_values[second_index].first +
                            "' must differ because they represent distinct " + category +
                            " stages");
                    }
                }
            }
        };

    auto resolveServiceName = [this](const std::string & name) {
            return rclcpp::expand_topic_or_service_name(
                name,
                this->get_name(),
                this->get_namespace(),
                true);
        };

    auto resolveTopicName = [this](const std::string & name) {
            return rclcpp::expand_topic_or_service_name(
                name,
                this->get_name(),
                this->get_namespace(),
                false);
        };

    requireNonEmpty(params_.store_pipeline_service_name, "store_pipeline_service_name");
    requireNonEmpty(params_.take_pipeline_service_name, "take_pipeline_service_name");
    requireNonEmpty(params_.trajectory_topic, "trajectory_topic");
    requireNonEmpty(params_.joint_state_topic, "joint_state_topic");
    requireNonEmpty(
        params_.move_to_energy_unit_trajectory_file,
        "move_to_energy_unit_trajectory_file");
    requireNonEmpty(
        params_.take_out_energy_unit_trajectory_file,
        "take_out_energy_unit_trajectory_file");
    requireNonEmpty(params_.place_energy_unit_trajectory_file, "place_energy_unit_trajectory_file");
    requireNonEmpty(params_.restore_trajectory_file, "restore_trajectory_file");
    requireNonEmpty(params_.open_gripper_topic, "open_gripper_topic");
    requireNonEmpty(params_.close_gripper_topic, "close_gripper_topic");
    requireNonEmpty(params_.interrupt_topic, "interrupt_topic");

    requireDistinctNames(
        std::array<std::pair<const char *, std::string>, 2>{
            std::pair<const char *, std::string>{
                "store_pipeline_service_name",
                resolveServiceName(params_.store_pipeline_service_name)},
            std::pair<const char *, std::string>{
                "take_pipeline_service_name",
                resolveServiceName(params_.take_pipeline_service_name)}},
        "service");

    requireDistinctNames(
        std::array<std::pair<const char *, std::string>, 5>{
            std::pair<const char *, std::string>{
                "trajectory_topic",
                resolveTopicName(params_.trajectory_topic)},
            std::pair<const char *, std::string>{
                "joint_state_topic",
                resolveTopicName(params_.joint_state_topic)},
            std::pair<const char *, std::string>{
                "open_gripper_topic",
                resolveTopicName(params_.open_gripper_topic)},
            std::pair<const char *, std::string>{
                "close_gripper_topic",
                resolveTopicName(params_.close_gripper_topic)},
            std::pair<const char *, std::string>{
                "interrupt_topic",
                resolveTopicName(params_.interrupt_topic)}},
        "topic");

    if (params_.trajectory_completion_extra_wait_sec <= 0.0) {
        throw std::invalid_argument(
            "Parameter 'trajectory_completion_extra_wait_sec' must be > 0");
    }
    if (params_.trajectory_interpolation_time_interval_sec <= 0.0) {
        throw std::invalid_argument(
            "Parameter 'trajectory_interpolation_time_interval_sec' must be > 0");
    }
    if (params_.trajectory_interpolation_max_joint_step_rad <= 0.0) {
        throw std::invalid_argument(
            "Parameter 'trajectory_interpolation_max_joint_step_rad' must be > 0");
    }
    if (params_.trajectory_interpolation_joint_state_timeout_sec < 0.0) {
        throw std::invalid_argument(
            "Parameter 'trajectory_interpolation_joint_state_timeout_sec' must be >= 0");
    }
    if (params_.wait_after_open_gripper_sec < 0.0) {
        throw std::invalid_argument("Parameter 'wait_after_open_gripper_sec' must be >= 0");
    }
    if (params_.wait_after_close_gripper_sec < 0.0) {
        throw std::invalid_argument("Parameter 'wait_after_close_gripper_sec' must be >= 0");
    }
    if (params_.gripper_command_wait_for_subscription_timeout_sec < 0.0) {
        throw std::invalid_argument(
            "Parameter 'gripper_command_wait_for_subscription_timeout_sec' must be >= 0");
    }
}

void Rm26EnergyUnitPap::loadTrajectories()
{
    auto loadNamedTrajectory = [this](
            const std::string & stage_name,
            const std::string & file_path) {
            auto loaded = loadTrajectoryFromYamlFile(std::filesystem::path(file_path));
            NamedTrajectory named;
            named.stage_name = stage_name;
            named.file_path = file_path;
            named.trajectory = std::move(loaded.trajectory);
            named.duration_sec = loaded.duration_sec;

            RCLCPP_INFO(
                get_logger(),
                "Loaded trajectory '%s': file=%s, joints=%zu, points=%zu, duration=%.3f s",
                named.stage_name.c_str(),
                named.file_path.c_str(),
                named.trajectory.joint_names.size(),
                named.trajectory.points.size(),
                named.duration_sec);
            return named;
        };

    move_to_energy_unit_trajectory_ = loadNamedTrajectory(
        "move_to_energy_unit",
        params_.move_to_energy_unit_trajectory_file);
    take_out_energy_unit_trajectory_ = loadNamedTrajectory(
        "take_out_energy_unit",
        params_.take_out_energy_unit_trajectory_file);
    place_energy_unit_trajectory_ = loadNamedTrajectory(
        "place_energy_unit",
        params_.place_energy_unit_trajectory_file);
    restore_trajectory_ = loadNamedTrajectory(
        "restore",
        params_.restore_trajectory_file);
}

bool Rm26EnergyUnitPap::tryAcquireExecutionSlot()
{
    std::lock_guard<std::mutex> lock(execution_state_mutex_);
    if (execution_slot_taken_) {
        return false;
    }
    execution_slot_taken_ = true;
    interrupt_requested_.store(false);
    return true;
}

void Rm26EnergyUnitPap::releaseExecutionSlot()
{
    std::lock_guard<std::mutex> lock(execution_state_mutex_);
    execution_slot_taken_ = false;
    interrupt_requested_.store(false);
}

void Rm26EnergyUnitPap::handleInterrupt(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data) {
        return;
    }

    std::lock_guard<std::mutex> lock(execution_state_mutex_);
    if (!execution_slot_taken_) {
        return;
    }

    interrupt_requested_.store(true);
    RCLCPP_WARN(
        get_logger(),
        "Interrupt requested by topic %s",
        params_.interrupt_topic.c_str());
}

void Rm26EnergyUnitPap::handleJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    latest_joint_state_ = *msg;
    latest_joint_state_receive_time_ = SteadyClock::now();
    has_latest_joint_state_ = true;
}

bool Rm26EnergyUnitPap::isInterruptRequested() const
{
    return interrupt_requested_.load();
}

Rm26EnergyUnitPap::PipelineResult Rm26EnergyUnitPap::makeInterruptResult(
    const std::string & stage_name) const
{
    return {false, stage_name + " interrupted by interrupt topic"};
}

void Rm26EnergyUnitPap::handleStoreEnergyUnit(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
{
    (void)request;

    if (!tryAcquireExecutionSlot()) {
        response->success = false;
        response->message = "Another pipeline is already running";
        return;
    }

    ScopedExit release_slot([this]() { releaseExecutionSlot(); });

    const auto pipeline_start_time = SteadyClock::now();
    const auto result = executeStorePipeline();
    const double pipeline_elapsed_ms = elapsedMs(pipeline_start_time);

    response->success = result.success;
    response->message = ensureMessage(
        result.success ? "Store pipeline succeeded" : "Store pipeline failed",
        result.message);

    if (result.success) {
        RCLCPP_INFO(
            get_logger(),
            "Store pipeline succeeded in %.1f ms: %s",
            pipeline_elapsed_ms,
            response->message.c_str());
    } else {
        RCLCPP_ERROR(
            get_logger(),
            "Store pipeline failed in %.1f ms: %s",
            pipeline_elapsed_ms,
            response->message.c_str());
    }
}

void Rm26EnergyUnitPap::handleTakeEnergyUnit(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
{
    (void)request;

    if (!tryAcquireExecutionSlot()) {
        response->success = false;
        response->message = "Another pipeline is already running";
        return;
    }

    ScopedExit release_slot([this]() { releaseExecutionSlot(); });

    const auto pipeline_start_time = SteadyClock::now();
    const auto result = executeTakePipeline();
    const double pipeline_elapsed_ms = elapsedMs(pipeline_start_time);

    response->success = result.success;
    response->message = ensureMessage(
        result.success ? "Take pipeline succeeded" : "Take pipeline failed",
        result.message);

    if (result.success) {
        RCLCPP_INFO(
            get_logger(),
            "Take pipeline succeeded in %.1f ms: %s",
            pipeline_elapsed_ms,
            response->message.c_str());
    } else {
        RCLCPP_ERROR(
            get_logger(),
            "Take pipeline failed in %.1f ms: %s",
            pipeline_elapsed_ms,
            response->message.c_str());
    }
}

Rm26EnergyUnitPap::PipelineResult Rm26EnergyUnitPap::executeStorePipeline()
{
    constexpr std::size_t total_steps = 5;

    auto result = executeStep(
        "Store",
        1,
        total_steps,
        "Publish place_energy_unit interpolation trajectory",
        [this]() {
            return publishInterpolationBeforeTrajectory(
                place_energy_unit_trajectory_,
                "place_energy_unit_interpolation");
        });
    if (!result.success) {
        return result;
    }

    result = executeStep(
        "Store",
        2,
        total_steps,
        "Publish place_energy_unit trajectory",
        [this]() {
            return publishAndWaitTrajectory(
                place_energy_unit_trajectory_,
                "place_energy_unit");
        });
    if (!result.success) {
        return result;
    }

    result = executeStep(
        "Store",
        3,
        total_steps,
        "Open gripper",
        [this]() {
            return publishGripperBoolCommand(
                open_gripper_publisher_,
                params_.open_gripper_topic,
                "open_gripper");
        });
    if (!result.success) {
        return result;
    }

    result = executeStep(
        "Store",
        4,
        total_steps,
        "Wait after opening gripper",
        [this]() {
            return waitForSeconds(
                params_.wait_after_open_gripper_sec,
                "wait_after_open_gripper");
        });
    if (!result.success) {
        return result;
    }

    return executeStep(
        "Store",
        5,
        total_steps,
        "Publish restore trajectory",
        [this]() {
            return publishAndWaitTrajectory(
                restore_trajectory_,
                "restore");
        });
}

Rm26EnergyUnitPap::PipelineResult Rm26EnergyUnitPap::executeTakePipeline()
{
    constexpr std::size_t total_steps = 7;

    auto result = executeStep(
        "Take",
        1,
        total_steps,
        "Open gripper",
        [this]() {
            return publishGripperBoolCommand(
                open_gripper_publisher_,
                params_.open_gripper_topic,
                "open_gripper");
        });
    if (!result.success) {
        return result;
    }

    result = executeStep(
        "Take",
        2,
        total_steps,
        "Wait after opening gripper",
        [this]() {
            return waitForSeconds(
                params_.wait_after_open_gripper_sec,
                "wait_after_open_gripper");
        });
    if (!result.success) {
        return result;
    }

    result = executeStep(
        "Take",
        3,
        total_steps,
        "Publish move_to_energy_unit interpolation trajectory",
        [this]() {
            return publishInterpolationBeforeTrajectory(
                move_to_energy_unit_trajectory_,
                "move_to_energy_unit_interpolation");
        });
    if (!result.success) {
        return result;
    }

    result = executeStep(
        "Take",
        4,
        total_steps,
        "Publish move_to_energy_unit trajectory",
        [this]() {
            return publishAndWaitTrajectory(
                move_to_energy_unit_trajectory_,
                "move_to_energy_unit");
        });
    if (!result.success) {
        return result;
    }

    result = executeStep(
        "Take",
        5,
        total_steps,
        "Close gripper",
        [this]() {
            return publishGripperBoolCommand(
                close_gripper_publisher_,
                params_.close_gripper_topic,
                "close_gripper");
        });
    if (!result.success) {
        return result;
    }

    result = executeStep(
        "Take",
        6,
        total_steps,
        "Wait after closing gripper",
        [this]() {
            return waitForSeconds(
                params_.wait_after_close_gripper_sec,
                "wait_after_close_gripper");
        });
    if (!result.success) {
        return result;
    }

    return executeStep(
        "Take",
        7,
        total_steps,
        "Publish take_out_energy_unit trajectory",
        [this]() {
            return publishAndWaitTrajectory(
                take_out_energy_unit_trajectory_,
                "take_out_energy_unit");
        });
}

Rm26EnergyUnitPap::PipelineResult Rm26EnergyUnitPap::executeStep(
    const char * pipeline_name,
    std::size_t step_index,
    std::size_t total_steps,
    const std::string & step_name,
    const std::function<PipelineResult()> & step_fn)
{
    if (isInterruptRequested()) {
        return makeInterruptResult(step_name);
    }

    const auto step_start_time = SteadyClock::now();
    RCLCPP_INFO(
        get_logger(),
        "[%s %zu/%zu] %s - start",
        pipeline_name,
        step_index,
        total_steps,
        step_name.c_str());

    auto result = step_fn();
    if (result.success && isInterruptRequested()) {
        result = makeInterruptResult(step_name);
    }
    const double step_elapsed_ms = elapsedMs(step_start_time);

    if (result.success) {
        RCLCPP_INFO(
            get_logger(),
            "[%s %zu/%zu] %s - done in %.1f ms",
            pipeline_name,
            step_index,
            total_steps,
            step_name.c_str(),
            step_elapsed_ms);
    } else {
        RCLCPP_ERROR(
            get_logger(),
            "[%s %zu/%zu] %s - failed in %.1f ms: %s",
            pipeline_name,
            step_index,
            total_steps,
            step_name.c_str(),
            step_elapsed_ms,
            result.message.c_str());
    }

    return result;
}

Rm26EnergyUnitPap::PipelineResult Rm26EnergyUnitPap::publishInterpolationBeforeTrajectory(
    const NamedTrajectory & target_trajectory,
    const std::string & stage_name)
{
    if (!params_.enable_trajectory_interpolation) {
        return {true, stage_name + " skipped because interpolation is disabled"};
    }
    if (!rclcpp::ok()) {
        return {false, stage_name + " interrupted by ROS shutdown before interpolation"};
    }
    if (isInterruptRequested()) {
        return makeInterruptResult(stage_name);
    }

    std::vector<double> current_positions;
    std::string reason;
    if (!waitForCurrentJointPositions(
            target_trajectory.trajectory.joint_names,
            current_positions,
            reason))
    {
        if (!rclcpp::ok()) {
            return {false, stage_name + " interrupted by ROS shutdown while waiting for JointState"};
        }
        if (isInterruptRequested()) {
            return makeInterruptResult(stage_name);
        }

        RCLCPP_WARN(
            get_logger(),
            "Skip %s: %s",
            stage_name.c_str(),
            reason.c_str());
        return {true, stage_name + " skipped: " + reason};
    }

    NamedTrajectory interpolation_trajectory;
    if (!buildInterpolationTrajectory(
            target_trajectory,
            current_positions,
            stage_name,
            interpolation_trajectory,
            reason))
    {
        RCLCPP_INFO(
            get_logger(),
            "Skip %s: %s",
            stage_name.c_str(),
            reason.c_str());
        return {true, stage_name + " skipped: " + reason};
    }

    RCLCPP_INFO(
        get_logger(),
        "Generated %s trajectory before %s: points=%zu, duration=%.3f s",
        stage_name.c_str(),
        target_trajectory.stage_name.c_str(),
        interpolation_trajectory.trajectory.points.size(),
        interpolation_trajectory.duration_sec);

    return publishAndWaitTrajectory(interpolation_trajectory, stage_name);
}

bool Rm26EnergyUnitPap::waitForCurrentJointPositions(
    const std::vector<std::string> & joint_names,
    std::vector<double> & positions,
    std::string & reason) const
{
    if (tryGetLatestJointPositions(joint_names, positions, reason)) {
        return true;
    }

    const double timeout_sec = params_.trajectory_interpolation_joint_state_timeout_sec;
    if (timeout_sec <= 0.0) {
        return false;
    }

    const auto deadline =
        SteadyClock::now() +
        std::chrono::duration_cast<SteadyClock::duration>(
            std::chrono::duration<double>(timeout_sec));

    while (rclcpp::ok() && !isInterruptRequested() && SteadyClock::now() < deadline) {
        const auto wait_duration = clampWaitDuration(
            deadline,
            std::chrono::duration_cast<SteadyClock::duration>(kLocalWaitPollPeriod));
        if (wait_duration == SteadyClock::duration::zero()) {
            break;
        }

        std::this_thread::sleep_for(wait_duration);
        if (tryGetLatestJointPositions(joint_names, positions, reason)) {
            return true;
        }
    }

    return false;
}

bool Rm26EnergyUnitPap::tryGetLatestJointPositions(
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

    const double timeout_sec = params_.trajectory_interpolation_joint_state_timeout_sec;
    if (timeout_sec > 0.0) {
        const double age_sec =
            std::chrono::duration<double>(SteadyClock::now() - receive_time).count();
        if (age_sec > timeout_sec) {
            std::ostringstream oss;
            oss << "latest JointState is stale: age=" << age_sec
                << " s, timeout=" << timeout_sec << " s";
            reason = oss.str();
            return false;
        }
    }

    std::unordered_map<std::string, std::size_t> index_by_name;
    index_by_name.reserve(latest_joint_state.name.size());
    for (std::size_t i = 0; i < latest_joint_state.name.size(); ++i) {
        if (latest_joint_state.name[i].empty()) {
            continue;
        }
        index_by_name.emplace(latest_joint_state.name[i], i);
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

bool Rm26EnergyUnitPap::buildInterpolationTrajectory(
    const NamedTrajectory & target_trajectory,
    const std::vector<double> & current_positions,
    const std::string & stage_name,
    NamedTrajectory & interpolation_trajectory,
    std::string & reason) const
{
    if (!params_.enable_trajectory_interpolation) {
        reason = "interpolation is disabled";
        return false;
    }
    if (target_trajectory.trajectory.points.empty()) {
        reason = "target trajectory has no points";
        return false;
    }

    const auto & joint_names = target_trajectory.trajectory.joint_names;
    const auto & target_positions = target_trajectory.trajectory.points.front().positions;
    if (current_positions.size() != joint_names.size()) {
        reason = "current position size does not match target joint count";
        return false;
    }
    if (target_positions.size() != joint_names.size()) {
        reason = "target trajectory first point position size does not match joint count";
        return false;
    }

    double max_delta = 0.0;
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
        if (!std::isfinite(target_positions[i])) {
            reason = "target trajectory first point position for joint '" + joint_names[i] +
                "' is not finite";
            return false;
        }
        max_delta = std::max(max_delta, std::abs(target_positions[i] - current_positions[i]));
    }

    if (max_delta <= std::numeric_limits<double>::epsilon()) {
        reason = "current position already matches target start point";
        return false;
    }

    const auto step_count = static_cast<std::size_t>(
        std::ceil(max_delta / params_.trajectory_interpolation_max_joint_step_rad));
    if (step_count == 0U) {
        reason = "interpolation step count is zero";
        return false;
    }

    interpolation_trajectory = NamedTrajectory{};
    interpolation_trajectory.stage_name = stage_name;
    interpolation_trajectory.file_path = "<generated>";
    interpolation_trajectory.duration_sec =
        static_cast<double>(step_count) *
        params_.trajectory_interpolation_time_interval_sec;
    interpolation_trajectory.trajectory.joint_names = joint_names;
    interpolation_trajectory.trajectory.points.reserve(step_count + 1U);

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
            static_cast<double>(step) *
            params_.trajectory_interpolation_time_interval_sec);

        interpolation_trajectory.trajectory.points.push_back(std::move(point));
    }

    return true;
}

Rm26EnergyUnitPap::PipelineResult Rm26EnergyUnitPap::publishAndWaitTrajectory(
    const NamedTrajectory & trajectory,
    const std::string & stage_name)
{
    if (!rclcpp::ok()) {
        return {false, stage_name + " interrupted by ROS shutdown before trajectory publish"};
    }
    if (isInterruptRequested()) {
        return makeInterruptResult(stage_name);
    }

    auto message = trajectory.trajectory;
    message.header.stamp = now();
    int seq = 0;

    const auto publish_trajectory_times = [this, &message, &seq](){
        if(seq <= 10){
            trajectory_publisher_->publish(message);
            ++seq;
        }
        return;
    };
    publish_trajectory_times();

    const double wait_seconds =
        trajectory.duration_sec + params_.trajectory_completion_extra_wait_sec;
    RCLCPP_INFO(
        get_logger(),
        "%s trajectory published to %s: points=%zu, duration=%.3f s, extra_wait=%.3f s",
        stage_name.c_str(),
        params_.trajectory_topic.c_str(),
        message.points.size(),
        trajectory.duration_sec,
        params_.trajectory_completion_extra_wait_sec);

    auto wait_result = waitForSeconds(wait_seconds, stage_name + "_trajectory_completion_wait", publish_trajectory_times);
    if (!wait_result.success) {
        return wait_result;
    }

    std::ostringstream oss;
    oss << stage_name << " trajectory published and waited " << wait_seconds << " s";
    return {true, oss.str()};
}

Rm26EnergyUnitPap::PipelineResult Rm26EnergyUnitPap::waitForSeconds(
    double seconds,
    const std::string & stage_name,
    const std::function<void()> & wait_fn) const
{
    if (seconds <= 0.0) {
        if (isInterruptRequested()) {
            return makeInterruptResult(stage_name);
        }
        return {true, stage_name + " skipped"};
    }

    const auto deadline =
        SteadyClock::now() +
        std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<double>(seconds));

    while (rclcpp::ok() && !isInterruptRequested() && SteadyClock::now() < deadline) {
        const auto wait_duration = clampWaitDuration(
            deadline,
            std::chrono::duration_cast<SteadyClock::duration>(kLocalWaitPollPeriod));
        if (wait_duration == SteadyClock::duration::zero()) {
            break;
        }
        wait_fn();
        std::this_thread::sleep_for(wait_duration);
    }

    if (!rclcpp::ok()) {
        return {false, stage_name + " interrupted by ROS shutdown"};
    }
    if (isInterruptRequested()) {
        return makeInterruptResult(stage_name);
    }

    return {true, stage_name + " completed"};
}

Rm26EnergyUnitPap::PipelineResult Rm26EnergyUnitPap::publishGripperBoolCommand(
    const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & publisher,
    const std::string & topic_name,
    const std::string & stage_name) const
{
    // const auto subscription_deadline =
    //     SteadyClock::now() +
    //     std::chrono::duration_cast<SteadyClock::duration>(
    //         std::chrono::duration<double>());

    // while (rclcpp::ok() && !isInterruptRequested()) {
    //     if (publisherSubscriptionCount(publisher) > 0U) {
    //         break;
    //     }

    //     const auto wait_duration = clampWaitDuration(
    //         subscription_deadline,
    //         std::chrono::duration_cast<SteadyClock::duration>(kLocalWaitPollPeriod));
    //     if (wait_duration == SteadyClock::duration::zero()) {
    //         return {
    //             false,
    //             "Timed out waiting for subscribers on topic '" + topic_name + "' during " +
    //                 stage_name};
    //     }
    //     std::this_thread::sleep_for(wait_duration);
    // }

    if (!rclcpp::ok()) {
        return {false, stage_name + " interrupted by ROS shutdown"};
    }
    if (isInterruptRequested()) {
        return makeInterruptResult(stage_name);
    }

    std_msgs::msg::Bool command;
    command.data = true;
    int seq = 0;

    const auto publish_command_times = [this, &publisher, &command, &seq](){
        if(seq <= 10){
            publisher->publish(command);
            ++seq;
        }
        return;
    };
    publish_command_times();


    auto wait_result = waitForSeconds(params_.gripper_command_wait_for_subscription_timeout_sec, stage_name + "_gripper_control_wait", publish_command_times);

    return {true, stage_name + " published to " + topic_name};
}

}  // namespace rm26_energy_unit_pap
