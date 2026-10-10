#include "axis_free_rotate_ik/axis_free_rotate_ik_ros.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <Eigen/Geometry>

#include "rclcpp/wait_for_message.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace axis_free_rotate_ik
{
namespace
{

bool is_finite_positive(const double value)
{
    return std::isfinite(value) && value > 0.0;
}

bool ensure_finite_service_double(
    const double value,
    const std::string & field_name,
    std::string & error_message)
{
    if (std::isfinite(value)) {
        return true;
    }

    error_message = field_name + " must be finite";
    return false;
}

bool is_finite_vector3(const Eigen::Vector3d & vector)
{
    return vector.allFinite();
}

Eigen::Vector3d to_eigen(const geometry_msgs::msg::Vector3 & vector)
{
    return {vector.x, vector.y, vector.z};
}

Eigen::Vector3d pose_position_to_eigen(const geometry_msgs::msg::Pose & pose)
{
    return {pose.position.x, pose.position.y, pose.position.z};
}

bool pose_rotation_to_eigen(
    const geometry_msgs::msg::Pose & pose,
    Eigen::Matrix3d & rotation,
    std::string & error_message)
{
    const auto & q_msg = pose.orientation;
    if (!std::isfinite(q_msg.x) || !std::isfinite(q_msg.y) ||
        !std::isfinite(q_msg.z) || !std::isfinite(q_msg.w))
    {
        error_message = "Pose orientation contains non-finite values";
        return false;
    }

    Eigen::Quaterniond quaternion(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    const double norm = quaternion.norm();
    if (!std::isfinite(norm) || norm <= std::numeric_limits<double>::epsilon()) {
        error_message = "Pose orientation quaternion is zero or invalid";
        return false;
    }

    quaternion.normalize();
    rotation = quaternion.toRotationMatrix();
    return true;
}

bool pose_position_is_finite(const geometry_msgs::msg::Pose & pose)
{
    return std::isfinite(pose.position.x) &&
           std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z);
}

std::string point_index_message(const std::string & prefix, const size_t index, const std::string & reason)
{
    return prefix + " at point " + std::to_string(index) + ": " + reason;
}

std::vector<double> eigen_to_std_vector(const Eigen::VectorXd & vector)
{
    std::vector<double> output;
    output.reserve(static_cast<size_t>(vector.size()));
    for (Eigen::Index i = 0; i < vector.size(); ++i) {
        output.push_back(vector(i));
    }
    return output;
}

struct FixedTarget
{
    Eigen::Vector3d desired_position;
    Eigen::Matrix3d desired_orientation;
};

}  // namespace

AxisFreeRotateIKRos::AxisFreeRotateIKRos(const rclcpp::NodeOptions & options)
    : rclcpp::Node("axis_free_rotate_ik_ros", options)
{
    load_parameters();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    initialize_solver(wait_for_robot_description());
    create_publishers();
    create_services();
}

void AxisFreeRotateIKRos::load_parameters()
{
    try {
        param_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
        params_ = param_listener_->get_params();
    } catch (const std::exception & e) {
        throw std::runtime_error(std::string("Failed to load axis_free_rotate_ik_ros parameters: ") + e.what());
    }

    robot_description_topic_ = params_.robot_description_topic;
    robot_description_timeout_ = params_.robot_description_timeout;
    base_link_ = params_.base_link;
    end_effector_link_ = params_.end_effector_link;
    joint_trajectory_topic_ = params_.joint_trajectory_topic;

    if (robot_description_topic_.empty()) {
        throw std::runtime_error("robot_description_topic must not be empty");
    }
    if (!is_finite_positive(robot_description_timeout_)) {
        throw std::runtime_error("robot_description_timeout must be finite and positive");
    }
    if (base_link_.empty()) {
        throw std::runtime_error("base_link must not be empty");
    }
    if (end_effector_link_.empty()) {
        throw std::runtime_error("end_effector_link must not be empty");
    }
}

std::string AxisFreeRotateIKRos::wait_for_robot_description()
{
    std_msgs::msg::String robot_description_msg;
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    auto subscription = create_subscription<std_msgs::msg::String>(
        robot_description_topic_,
        qos,
        [](const std_msgs::msg::String::SharedPtr) {
        });

    const auto timeout = std::chrono::duration<double>(robot_description_timeout_);
    const bool received = rclcpp::wait_for_message<std_msgs::msg::String>(
        robot_description_msg,
        subscription,
        get_node_options().context(),
        timeout);

    if (!received) {
        throw std::runtime_error(
                  "Timed out waiting for robot_description on topic '" + robot_description_topic_ + "'");
    }
    if (robot_description_msg.data.empty()) {
        throw std::runtime_error("Received empty robot_description on topic '" + robot_description_topic_ + "'");
    }

    return robot_description_msg.data;
}

void AxisFreeRotateIKRos::initialize_solver(const std::string & robot_description)
{
    try {
        solver_ = std::make_unique<AxisFreeRotateIK>(
            robot_description,
            base_link_,
            end_effector_link_);
    } catch (const std::exception & e) {
        throw std::runtime_error(std::string("Failed to initialize AxisFreeRotateIK: ") + e.what());
    }

    RCLCPP_INFO(
        get_logger(),
        "AxisFreeRotateIK ROS wrapper initialized: base_link=%s, end_effector_link=%s, joints=%zu",
        base_link_.c_str(),
        end_effector_link_.c_str(),
        solver_->joint_names().size());
}

void AxisFreeRotateIKRos::create_publishers()
{
    if (joint_trajectory_topic_.empty()) {
        RCLCPP_INFO(get_logger(), "JointTrajectory publishing disabled: joint_trajectory_topic is empty");
        return;
    }

    trajectory_publisher_ =
        create_publisher<trajectory_msgs::msg::JointTrajectory>(joint_trajectory_topic_, 10);
    RCLCPP_INFO(
        get_logger(),
        "Created JointTrajectory publisher on topic: %s",
        joint_trajectory_topic_.c_str());
}

void AxisFreeRotateIKRos::create_services()
{
    fixed_ik_service_ = create_service<SolveFixedIK>(
        "~/solve_fixed_ik",
        std::bind(
            &AxisFreeRotateIKRos::handle_fixed_ik,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    axis_free_ik_service_ = create_service<SolveAxisFreeIK>(
        "~/solve_axis_free_ik",
        std::bind(
            &AxisFreeRotateIKRos::handle_axis_free_ik,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
}

bool AxisFreeRotateIKRos::transform_pose_to_base(
    const geometry_msgs::msg::PoseStamped & input,
    const double tf_timeout,
    geometry_msgs::msg::PoseStamped & output,
    std::string & error_message) const
{
    if (!ensure_finite_service_double(tf_timeout, "tf_timeout", error_message)) {
        return false;
    }

    if (input.header.frame_id.empty()) {
        error_message = "Pose frame_id cannot be empty";
        return false;
    }

    if (!pose_position_is_finite(input.pose)) {
        error_message = "Pose position contains non-finite values";
        return false;
    }

    Eigen::Matrix3d unused_rotation;
    if (!pose_rotation_to_eigen(input.pose, unused_rotation, error_message)) {
        return false;
    }

    if (input.header.frame_id == base_link_) {
        output = input;
        output.header.frame_id = base_link_;
        return true;
    }

    try {
        output = tf_buffer_->transform(
            input,
            base_link_,
            tf2::durationFromSec(std::max(0.0, tf_timeout)));
    } catch (const tf2::TransformException & e) {
        error_message = "Failed to transform pose from '" + input.header.frame_id + "' to '" +
            base_link_ + "': " + e.what();
        return false;
    }

    if (!pose_position_is_finite(output.pose)) {
        error_message = "Transformed pose position contains non-finite values";
        return false;
    }
    if (!pose_rotation_to_eigen(output.pose, unused_rotation, error_message)) {
        error_message = "Transformed " + error_message;
        return false;
    }

    return true;
}

std::optional<Eigen::VectorXd> AxisFreeRotateIKRos::vector_to_optional_q(
    const std::vector<double> & values,
    const std::string & field_name,
    std::string & error_message) const
{
    if (values.empty()) {
        return std::nullopt;
    }

    const size_t expected_size = solver_->joint_names().size();
    if (values.size() != expected_size) {
        std::ostringstream stream;
        stream << field_name << " size must be " << expected_size << ", got " << values.size();
        error_message = stream.str();
        return std::nullopt;
    }

    Eigen::VectorXd output(static_cast<Eigen::Index>(values.size()));
    for (size_t i = 0; i < values.size(); ++i) {
        if (!std::isfinite(values[i])) {
            error_message = field_name + " contains non-finite values";
            return std::nullopt;
        }
        output(static_cast<Eigen::Index>(i)) = values[i];
    }

    return output;
}

bool AxisFreeRotateIKRos::apply_common_request_fields(
    const std::vector<double> & initial_guess,
    const std::vector<double> & delta_max_q,
    const int32_t max_iterations,
    const double tolerance,
    const double damping,
    const double alpha,
    IKRequest & ik_request,
    std::string & error_message) const
{
    if (!ensure_finite_service_double(tolerance, "tolerance", error_message) ||
        !ensure_finite_service_double(damping, "damping", error_message) ||
        !ensure_finite_service_double(alpha, "alpha", error_message))
    {
        return false;
    }
    if (alpha > 1.0) {
        error_message = "alpha must be in the range (0, 1] when set";
        return false;
    }

    const auto optional_initial_guess =
        vector_to_optional_q(initial_guess, "initial_guess", error_message);
    if (!initial_guess.empty() && !optional_initial_guess.has_value()) {
        return false;
    }
    ik_request.initial_guess = optional_initial_guess;

    const auto optional_delta_max_q =
        vector_to_optional_q(delta_max_q, "delta_max_q", error_message);
    if (!delta_max_q.empty() && !optional_delta_max_q.has_value()) {
        return false;
    }
    if (optional_delta_max_q.has_value() && (optional_delta_max_q->array() < 0.0).any()) {
        error_message = "delta_max_q must be non-negative";
        return false;
    }
    ik_request.delta_max_q = optional_delta_max_q;

    if (max_iterations > 0) {
        ik_request.max_iterations = max_iterations;
    }
    if (is_finite_positive(tolerance)) {
        ik_request.tolerance = tolerance;
    }
    if (is_finite_positive(damping)) {
        ik_request.damping = damping;
    }
    if (std::isfinite(alpha) && alpha > 0.0) {
        ik_request.alpha = alpha;
    }

    return true;
}

bool AxisFreeRotateIKRos::apply_common_trajectory_request_fields(
    const std::vector<double> & initial_guess,
    const std::vector<double> & delta_max_q,
    const int32_t max_iterations,
    const double tolerance,
    const double damping,
    const double alpha,
    IKTrajectoryRequest & trajectory_request,
    std::string & error_message) const
{
    if (!ensure_finite_service_double(tolerance, "tolerance", error_message) ||
        !ensure_finite_service_double(damping, "damping", error_message) ||
        !ensure_finite_service_double(alpha, "alpha", error_message))
    {
        return false;
    }
    if (alpha > 1.0) {
        error_message = "alpha must be in the range (0, 1] when set";
        return false;
    }

    const auto optional_initial_guess =
        vector_to_optional_q(initial_guess, "initial_guess", error_message);
    if (!initial_guess.empty() && !optional_initial_guess.has_value()) {
        return false;
    }
    trajectory_request.initial_guess = optional_initial_guess;

    const auto optional_delta_max_q =
        vector_to_optional_q(delta_max_q, "delta_max_q", error_message);
    if (!delta_max_q.empty() && !optional_delta_max_q.has_value()) {
        return false;
    }
    if (optional_delta_max_q.has_value() && (optional_delta_max_q->array() < 0.0).any()) {
        error_message = "delta_max_q must be non-negative";
        return false;
    }
    trajectory_request.delta_max_q = optional_delta_max_q;

    if (max_iterations > 0) {
        trajectory_request.max_iterations = max_iterations;
    }
    if (is_finite_positive(tolerance)) {
        trajectory_request.tolerance = tolerance;
    }
    if (is_finite_positive(damping)) {
        trajectory_request.damping = damping;
    }
    if (std::isfinite(alpha) && alpha > 0.0) {
        trajectory_request.alpha = alpha;
    }

    return true;
}

trajectory_msgs::msg::JointTrajectory AxisFreeRotateIKRos::build_trajectory(
    const std::vector<Eigen::VectorXd> & points,
    const double point_time_interval) const
{
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = now();
    trajectory.joint_names = solver_->joint_names();

    trajectory.points.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = eigen_to_std_vector(points[i]);
        point.velocities.assign(point.positions.size(), 0.0);
        point.time_from_start =
            rclcpp::Duration::from_seconds(point_time_interval * static_cast<double>(i + 1));
        trajectory.points.push_back(std::move(point));
    }

    return trajectory;
}

void AxisFreeRotateIKRos::publish_trajectory_if_requested(
    const bool publish_requested,
    const trajectory_msgs::msg::JointTrajectory & trajectory) const
{
    if (!publish_requested || !trajectory_publisher_) {
        return;
    }

    trajectory_publisher_->publish(trajectory);
}

void AxisFreeRotateIKRos::handle_fixed_ik(
    const std::shared_ptr<SolveFixedIK::Request> request,
    std::shared_ptr<SolveFixedIK::Response> response)
{
    response->trajectory = trajectory_msgs::msg::JointTrajectory();

    if (!solver_) {
        response->message = "AxisFreeRotateIK solver is not initialized";
        return;
    }
    if (request->target_poses.empty()) {
        response->message = "target_poses cannot be empty";
        return;
    }
    if (!is_finite_positive(request->point_time_interval)) {
        response->message = "point_time_interval must be finite and positive";
        return;
    }

    IKRequest ik_request;
    std::string error_message;
    if (!apply_common_request_fields(
            request->initial_guess,
            request->delta_max_q,
            request->max_iterations,
            request->tolerance,
            request->damping,
            request->alpha,
            ik_request,
            error_message))
    {
        response->message = error_message;
        return;
    }

    std::vector<FixedTarget> fixed_targets;
    fixed_targets.reserve(request->target_poses.size());
    for (size_t i = 0; i < request->target_poses.size(); ++i) {
        geometry_msgs::msg::PoseStamped target_pose;
        if (!transform_pose_to_base(request->target_poses[i], request->tf_timeout, target_pose, error_message)) {
            response->message = point_index_message("Fixed IK request invalid", i, error_message);
            return;
        }

        Eigen::Matrix3d desired_orientation;
        if (!pose_rotation_to_eigen(target_pose.pose, desired_orientation, error_message)) {
            response->message = point_index_message("Fixed IK request invalid", i, error_message);
            return;
        }

        fixed_targets.push_back({
                pose_position_to_eigen(target_pose.pose),
                desired_orientation});
    }

    std::vector<Eigen::VectorXd> solved_points;
    solved_points.reserve(fixed_targets.size());

    for (size_t i = 0; i < fixed_targets.size(); ++i) {
        ik_request.desired_position = fixed_targets[i].desired_position;
        ik_request.desired_orientation = fixed_targets[i].desired_orientation;

        const IKResult result = solver_->solve(ik_request);
        if (!result.success) {
            response->message = point_index_message("Fixed IK failed", i, result.message);
            return;
        }

        solved_points.push_back(result.q);
        ik_request.initial_guess = result.q;
    }

    response->trajectory = build_trajectory(solved_points, request->point_time_interval);
    publish_trajectory_if_requested(request->publish_trajectory, response->trajectory);
    response->message = "Fixed IK succeeded for " + std::to_string(solved_points.size()) + " points";
}

void AxisFreeRotateIKRos::handle_axis_free_ik(
    const std::shared_ptr<SolveAxisFreeIK::Request> request,
    std::shared_ptr<SolveAxisFreeIK::Response> response)
{
    response->trajectory = trajectory_msgs::msg::JointTrajectory();

    if (!solver_) {
        response->message = "AxisFreeRotateIK solver is not initialized";
        return;
    }
    if (request->target_poses.empty()) {
        response->message = "target_poses cannot be empty";
        return;
    }
    if (!is_finite_positive(request->point_time_interval)) {
        response->message = "point_time_interval must be finite and positive";
        return;
    }

    const Eigen::Vector3d local_axis = to_eigen(request->local_axis);
    if (!is_finite_vector3(local_axis) || local_axis.norm() <= std::numeric_limits<double>::epsilon()) {
        response->message = "local_axis must be finite and non-zero";
        return;
    }

    IKTrajectoryRequest trajectory_request;
    std::string error_message;
    if (!apply_common_trajectory_request_fields(
            request->initial_guess,
            request->delta_max_q,
            request->max_iterations,
            request->tolerance,
            request->damping,
            request->alpha,
            trajectory_request,
            error_message))
    {
        response->message = error_message;
        return;
    }

    const auto optional_max_step = vector_to_optional_q(
        request->max_joint_step_between_points,
        "max_joint_step_between_points",
        error_message);
    if (!request->max_joint_step_between_points.empty() && !optional_max_step.has_value()) {
        response->message = error_message;
        return;
    }
    if (optional_max_step.has_value() && (optional_max_step->array() < 0.0).any()) {
        response->message = "max_joint_step_between_points must be non-negative";
        return;
    }
    trajectory_request.max_joint_step_between_points = optional_max_step;

    if (!ensure_finite_service_double(request->position_tolerance, "position_tolerance", error_message) ||
        !ensure_finite_service_double(request->axis_tolerance, "axis_tolerance", error_message))
    {
        response->message = error_message;
        return;
    }
    if (is_finite_positive(request->position_tolerance)) {
        trajectory_request.position_tolerance = request->position_tolerance;
    }
    if (is_finite_positive(request->axis_tolerance)) {
        trajectory_request.axis_tolerance = request->axis_tolerance;
    }

    trajectory_request.points.reserve(request->target_poses.size());
    for (size_t i = 0; i < request->target_poses.size(); ++i) {
        geometry_msgs::msg::PoseStamped target_pose;
        if (!transform_pose_to_base(request->target_poses[i], request->tf_timeout, target_pose, error_message)) {
            response->message = point_index_message("Axis-free IK request invalid", i, error_message);
            return;
        }

        Eigen::Matrix3d desired_orientation;
        if (!pose_rotation_to_eigen(target_pose.pose, desired_orientation, error_message)) {
            response->message = point_index_message("Axis-free IK request invalid", i, error_message);
            return;
        }

        IKTrajectoryPoint point;
        point.desired_position = pose_position_to_eigen(target_pose.pose);
        point.desired_direction = desired_orientation * local_axis;
        point.local_axis = local_axis;
        trajectory_request.points.push_back(point);
    }

    const IKTrajectoryResult result = solver_->solveTrajectory(trajectory_request);
    if (!result.success) {
        response->message = result.message;
        return;
    }

    std::vector<Eigen::VectorXd> solved_points;
    solved_points.reserve(result.points.size());
    for (const auto & point : result.points) {
        solved_points.push_back(point.q);
    }

    response->trajectory = build_trajectory(solved_points, request->point_time_interval);
    publish_trajectory_if_requested(request->publish_trajectory, response->trajectory);
    response->message = "Axis-free IK succeeded for " + std::to_string(solved_points.size()) + " points";
}

}  // namespace axis_free_rotate_ik

RCLCPP_COMPONENTS_REGISTER_NODE(axis_free_rotate_ik::AxisFreeRotateIKRos)
