#include "axis_free_rotate_ik/axis_free_rotate_ik.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

namespace axis_free_rotate_ik
{
namespace
{

constexpr double kRotationMatrixTolerance = 1e-6;
constexpr double kOppositeAxisTolerance = 1e-6;
constexpr double kPi = 3.14159265358979323846;

bool is_xml_description(const std::string & robot_description)
{
    const auto first_non_space = std::find_if_not(
        robot_description.begin(), robot_description.end(),
        [](unsigned char c) {
                return std::isspace(c) != 0;
            });

    return first_non_space != robot_description.end() && *first_non_space == '<';
}

bool all_finite(const Eigen::VectorXd & vector)
{
    return vector.allFinite();
}

bool all_finite(const Eigen::Vector3d & vector)
{
    return vector.allFinite();
}

bool all_finite(const Eigen::Matrix3d & matrix)
{
    return matrix.allFinite();
}

bool is_finite_positive(const double value)
{
    return std::isfinite(value) && value > 0.0;
}

double free_position_tolerance(const IKRequest & request)
{
    return request.position_tolerance.value_or(request.tolerance);
}

double free_axis_tolerance(const IKRequest & request)
{
    return request.axis_tolerance.value_or(request.tolerance);
}

// Apply per-point solver options on top of trajectory defaults.
void apply_per_point_solver_options(
    const IKTrajectoryRequest & request,
    const IKTrajectoryPoint & point,
    IKRequest & point_request)
{
    if (point.solver_options.has_value()) {
        const auto & opts = *point.solver_options;
        point_request.max_iterations = opts.max_iterations.value_or(request.max_iterations);
        point_request.position_tolerance = opts.position_tolerance.has_value()
            ? opts.position_tolerance : request.position_tolerance;
        point_request.axis_tolerance = opts.axis_tolerance.has_value()
            ? opts.axis_tolerance : request.axis_tolerance;
        point_request.damping = opts.damping.value_or(request.damping);
        point_request.alpha = opts.alpha.value_or(request.alpha);
        point_request.delta_max_q = opts.delta_max_q.has_value()
            ? opts.delta_max_q : request.delta_max_q;
    }
}

Eigen::Vector3d stable_perpendicular_axis(const Eigen::Vector3d & axis)
{
    const Eigen::Vector3d unit_axis = axis.normalized();
    Eigen::Vector3d basis = Eigen::Vector3d::UnitX();
    if (std::abs(unit_axis.dot(basis)) > 0.9) {
        basis = Eigen::Vector3d::UnitY();
    }

    return unit_axis.cross(basis).normalized();
}

void clip_delta(Eigen::VectorXd & delta_q, const std::optional<Eigen::VectorXd> & delta_max_q)
{
    if (!delta_max_q.has_value()) {
        return;
    }

    for (Eigen::Index i = 0; i < delta_q.size(); ++i) {
        const double limit = std::abs((*delta_max_q)(i));
        delta_q(i) = std::clamp(delta_q(i), -limit, limit);
    }
}

pinocchio::SE3 base_to_end_effector_placement(
    const pinocchio::Data & data,
    const pinocchio::FrameIndex base_frame_id,
    const pinocchio::FrameIndex end_effector_frame_id)
{
    /**
     * data.oMf[frame] is the pose of frame in the world frame
     * data.oMf[base_frame_id].inverse() * data.oMf[end_effector_frame_id] means the pose of end_effector_frame_id in the base_frame_id frame
     */
    return data.oMf[base_frame_id].inverse() * data.oMf[end_effector_frame_id];
}

std::string iteration_failure_message(
    const int max_iterations,
    const double position_error_norm,
    const double orientation_error_norm)
{
    std::ostringstream stream;
    stream << "IK did not converge after " << max_iterations
           << " iterations. Position error: " << position_error_norm
           << ", orientation error: " << orientation_error_norm;
    return stream.str();
}

struct AxisFreeError
{
    double position;
    double axis;
};

AxisFreeError compute_axis_free_error(
    const pinocchio::Model & model,
    pinocchio::Data & data,
    const pinocchio::FrameIndex base_frame_id,
    const pinocchio::FrameIndex end_effector_frame_id,
    const Eigen::VectorXd & q,
    const IKTrajectoryPoint & point)
{
    if (q.size() != model.nq || !q.allFinite()) {
        return {
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity()};
    }

    pinocchio::framesForwardKinematics(model, data, q);
    const pinocchio::SE3 base_to_end_effector =
        base_to_end_effector_placement(data, base_frame_id, end_effector_frame_id);

    const Eigen::Vector3d desired_axis = point.desired_direction.normalized();
    const Eigen::Vector3d local_axis = point.local_axis.normalized();
    const Eigen::Vector3d current_axis =
        (base_to_end_effector.rotation() * local_axis).normalized();

    const double axis_dot = std::clamp(current_axis.dot(desired_axis), -1.0, 1.0);
    return {
        (point.desired_position - base_to_end_effector.translation()).norm(),
        std::acos(axis_dot)};
}

IKTrajectoryResult trajectory_failure(
    std::string message,
    const std::optional<size_t> first_failed_index = std::nullopt)
{
    return {false, {}, first_failed_index, std::move(message)};
}

} // namespace

AxisFreeRotateIK::AxisFreeRotateIK(
    const std::string & robot_description,
    const std::string & base_link,
    const std::string & end_effector_link
    /* const collision_check */)
    : base_link_(base_link),
    end_effector_link_(end_effector_link),
    model_(),
    data_(model_)
{
    pinocchio::Model full_model;
    if (is_xml_description(robot_description)) {
        pinocchio::urdf::buildModelFromXML(robot_description, full_model);
    } else {
        pinocchio::urdf::buildModel(robot_description, full_model);
    }

    if (!full_model.existFrame(base_link)) {
        throw std::runtime_error("Base link '" + base_link + "' does not exist in the robot model");
    }
    if (!full_model.existFrame(end_effector_link)) {
        throw std::runtime_error(
                  "End-effector link '" + end_effector_link + "' does not exist in the robot model");
    }

    const pinocchio::FrameIndex full_base_frame_id = full_model.getFrameId(base_link);
    const pinocchio::FrameIndex full_end_effector_frame_id = full_model.getFrameId(end_effector_link);

    std::vector<pinocchio::JointIndex> chain_joint_ids_reverse;
    std::set<pinocchio::JointIndex> chain_joint_set;

    pinocchio::FrameIndex current_frame_id = full_end_effector_frame_id;
    while (current_frame_id != full_base_frame_id) {
        if (current_frame_id >= static_cast<pinocchio::FrameIndex>(full_model.nframes)) {
            throw std::runtime_error(
                      "End-effector link '" + end_effector_link + "' is not a child of base link '" +
                      base_link + "'");
        }

        const auto & current_frame = full_model.frames[current_frame_id];
        const pinocchio::JointIndex parent_joint = current_frame.parentJoint;
        if (parent_joint > 0 && chain_joint_set.insert(parent_joint).second) {
            chain_joint_ids_reverse.push_back(parent_joint);
        }

        const pinocchio::FrameIndex parent_frame_id = current_frame.parentFrame;
        if (parent_frame_id == current_frame_id ||
            parent_frame_id >= static_cast<pinocchio::FrameIndex>(full_model.nframes))
        {
            throw std::runtime_error(
                      "End-effector link '" + end_effector_link + "' is not a child of base link '" +
                      base_link + "'");
        }
        current_frame_id = parent_frame_id;
    }

    std::reverse(chain_joint_ids_reverse.begin(), chain_joint_ids_reverse.end());
    if (chain_joint_ids_reverse.empty()) {
        throw std::runtime_error(
                  "No movable joints found between base link '" + base_link +
                  "' and end-effector link '" +
                  end_effector_link + "'");
    }

    joints_ordered_list_.reserve(chain_joint_ids_reverse.size());
    for (const pinocchio::JointIndex joint_id : chain_joint_ids_reverse) {
        if (full_model.nqs[joint_id] != 1 || full_model.nvs[joint_id] != 1) {
            throw std::runtime_error(
                      "Joint '" + full_model.names[joint_id] +
                      "' is not a single-DoF joint. AxisFreeRotateIK only supports single-DoF chains");
        }
        joints_ordered_list_.push_back(full_model.names[joint_id]);
    }

    std::vector<pinocchio::JointIndex> joints_to_lock;
    joints_to_lock.reserve(full_model.njoints - chain_joint_set.size() - 1);
    for (pinocchio::JointIndex joint_id = 1;
        joint_id < static_cast<pinocchio::JointIndex>(full_model.njoints);
        ++joint_id)
    {
        if (chain_joint_set.find(joint_id) == chain_joint_set.end()) {
            joints_to_lock.push_back(joint_id);
        }
    }

    const Eigen::VectorXd neutral_configuration = pinocchio::neutral(full_model);
    model_ = pinocchio::buildReducedModel(full_model, joints_to_lock, neutral_configuration);

    if (!model_.existFrame(base_link)) {
        throw std::runtime_error("Base link '" + base_link + "' was lost while building reduced model");
    }
    if (!model_.existFrame(end_effector_link)) {
        throw std::runtime_error(
                  "End-effector link '" + end_effector_link + "' was lost while building reduced model");
    }

    for (const auto & joint_name : joints_ordered_list_) {
        if (!model_.existJointName(joint_name)) {
            throw std::runtime_error(
                      "Joint '" + joint_name + "' was lost while building reduced model");
        }
    }

    if (model_.nq != static_cast<int>(joints_ordered_list_.size()) ||
        model_.nv != static_cast<int>(joints_ordered_list_.size()))
    {
        throw std::runtime_error(
                  "Reduced model dimension does not match extracted single-DoF chain joint count");
    }

    base_frame_id_ = model_.getFrameId(base_link);
    end_effector_frame_id_ = model_.getFrameId(end_effector_link);
    data_ = pinocchio::Data(model_);
    cache_model_limits();
}

void AxisFreeRotateIK::cache_model_limits()
{
    limited_q_indices_.clear();
    lower_limits_ = model_.lowerPositionLimit;
    upper_limits_ = model_.upperPositionLimit;

    const std::vector<bool> has_limit = model_.hasConfigurationLimit();
    limited_q_indices_.reserve(static_cast<size_t>(model_.nq));
    for (Eigen::Index i = 0; i < model_.nq; ++i) {
        if (i >= static_cast<Eigen::Index>(has_limit.size()) ||
            !has_limit[static_cast<size_t>(i)])
        {
            continue;
        }

        const double lower = lower_limits_(i);
        const double upper = upper_limits_(i);
        if (!std::isfinite(lower) || !std::isfinite(upper) || lower >= upper) {
            continue;
        }

        limited_q_indices_.push_back(i);
    }
}

void AxisFreeRotateIK::clamp_to_model_limits(Eigen::VectorXd & q) const
{
    for (const Eigen::Index idx : limited_q_indices_) {
        q(idx) = std::clamp(q(idx), lower_limits_(idx), upper_limits_(idx));
    }
}

void AxisFreeRotateIK::compute_frame_jacobian_in_base(
    const Eigen::VectorXd & q,
    SolverWorkspace & ws)
{
    ws.jacobian_world.setZero();
    pinocchio::computeFrameJacobian(
        model_,
        data_,
        q,
        end_effector_frame_id_,
        pinocchio::LOCAL_WORLD_ALIGNED,
        ws.jacobian_world);

    const Eigen::Matrix3d world_to_base = data_.oMf[base_frame_id_].rotation().transpose();

    ws.jacobian_base.topRows<3>().noalias() = world_to_base * ws.jacobian_world.topRows<3>();
    ws.jacobian_base.bottomRows<3>().noalias() = world_to_base * ws.jacobian_world.bottomRows<3>();
}

void AxisFreeRotateIK::solve_damped_least_squares(
    const Eigen::Ref<const Eigen::MatrixXd> & jacobian,
    const Eigen::Matrix<double, 6, 1> & error,
    const double damping,
    SolverWorkspace & ws) const
{
    ws.normal_matrix.noalias() = jacobian * jacobian.transpose();
    ws.normal_matrix.diagonal().array() += damping * damping;

    ws.task_step = ws.normal_matrix.ldlt().solve(error);
    ws.delta_q.noalias() = jacobian.transpose() * ws.task_step;
}

IKResult AxisFreeRotateIK::solve(const IKRequest & input_request)
{
    const auto validity = check_request_validity(input_request);
    if (!validity.success) {
        return validity;
    }

    if (input_request.desired_orientation.has_value()) {
        return solve_fix_ik(input_request);
    }
    return solve_free_ik(input_request);
}

IKTrajectoryResult AxisFreeRotateIK::solveTrajectory(const IKTrajectoryRequest & request)
{
    if (request.points.empty()) {
        return trajectory_failure("Trajectory points cannot be empty");
    }

    if (!std::isfinite(request.alpha) || request.alpha <= 0.0 || request.alpha > 1.0) {
        return trajectory_failure("Alpha must be in the range (0, 1]", 0);
    }
    if (!is_finite_positive(request.tolerance)) {
        return trajectory_failure("Tolerance must be finite and positive", 0);
    }
    if (request.position_tolerance.has_value() && !is_finite_positive(*request.position_tolerance)) {
        return trajectory_failure("Position tolerance must be finite and positive", 0);
    }
    if (request.axis_tolerance.has_value() && !is_finite_positive(*request.axis_tolerance)) {
        return trajectory_failure("Axis tolerance must be finite and positive", 0);
    }
    if (!is_finite_positive(request.damping)) {
        return trajectory_failure("Damping must be finite and positive", 0);
    }
    if (request.max_iterations <= 0) {
        return trajectory_failure("Max iterations must be positive", 0);
    }

    if (request.initial_guess.has_value()) {
        if (request.initial_guess->size() != model_.nq) {
            return trajectory_failure("Initial guess size does not match chain joint count", 0);
        }
        if (!all_finite(*request.initial_guess)) {
            return trajectory_failure("Initial guess contains non-finite values", 0);
        }
    }

    if (request.delta_max_q.has_value()) {
        if (request.delta_max_q->size() != model_.nv) {
            return trajectory_failure("Delta max q size does not match chain joint count", 0);
        }
        if (!all_finite(*request.delta_max_q)) {
            return trajectory_failure("Delta max q contains non-finite values", 0);
        }
        if ((request.delta_max_q->array() < 0.0).any()) {
            return trajectory_failure("Delta max q must be non-negative", 0);
        }
    }

    if (request.max_joint_step_between_points.has_value()) {
        if (request.max_joint_step_between_points->size() != model_.nv) {
            return trajectory_failure(
                "Max joint step between points size does not match chain joint count", 0);
        }
        if (!all_finite(*request.max_joint_step_between_points)) {
            return trajectory_failure("Max joint step between points contains non-finite values", 0);
        }
        if ((request.max_joint_step_between_points->array() < 0.0).any()) {
            return trajectory_failure("Max joint step between points must be non-negative", 0);
        }
    }

    for (size_t i = 0; i < request.points.size(); ++i) {
        const auto & point = request.points[i];
        if (!all_finite(point.desired_position)) {
            return trajectory_failure("Trajectory point desired position contains non-finite values", i);
        }
        if (!all_finite(point.desired_direction)) {
            return trajectory_failure("Trajectory point desired direction contains non-finite values", i);
        }
        if (!all_finite(point.local_axis)) {
            return trajectory_failure("Trajectory point local axis contains non-finite values", i);
        }
        if (point.desired_direction.norm() <= std::numeric_limits<double>::epsilon()) {
            return trajectory_failure("Trajectory point desired direction vector cannot be zero", i);
        }
        if (point.local_axis.norm() <= std::numeric_limits<double>::epsilon()) {
            return trajectory_failure("Trajectory point local axis cannot be zero", i);
        }
        // Per-point validation for optional solver options
        if (point.solver_options.has_value()) {
            const auto & opts = *point.solver_options;
            if (opts.position_tolerance.has_value() && !is_finite_positive(*opts.position_tolerance)) {
                return trajectory_failure("Position tolerance must be finite and positive", i);
            }
            if (opts.axis_tolerance.has_value() && !is_finite_positive(*opts.axis_tolerance)) {
                return trajectory_failure("Axis tolerance must be finite and positive", i);
            }
            if (opts.damping.has_value() && !is_finite_positive(*opts.damping)) {
                return trajectory_failure("Damping must be finite and positive", i);
            }
            if (opts.alpha.has_value() && (!std::isfinite(*opts.alpha) || *opts.alpha <= 0.0 || *opts.alpha > 1.0)) {
                return trajectory_failure("Alpha must be in the range (0, 1]", i);
            }
            if (opts.max_iterations.has_value() && *opts.max_iterations <= 0) {
                return trajectory_failure("Max iterations must be positive", i);
            }
            if (opts.delta_max_q.has_value()) {
                if (opts.delta_max_q->size() != model_.nv) {
                    return trajectory_failure("Delta max q size does not match chain joint count", i);
                }
                if (!all_finite(*opts.delta_max_q)) {
                    return trajectory_failure("Delta max q contains non-finite values", i);
                }
                if ((opts.delta_max_q->array() < 0.0).any()) {
                    return trajectory_failure("Delta max q must be non-negative", i);
                }
            }
        }
    }

    IKTrajectoryResult trajectory_result;
    trajectory_result.success = false;
    trajectory_result.points.reserve(request.points.size());

    std::optional<Eigen::VectorXd> previous_solution;
    for (size_t i = 0; i < request.points.size(); ++i) {
        const auto & point = request.points[i];

        IKRequest point_request;
        point_request.desired_position = point.desired_position;
        point_request.desired_direction = point.desired_direction;
        point_request.desired_up_vector = point.local_axis;
        point_request.delta_max_q = request.delta_max_q;
        point_request.max_iterations = request.max_iterations;
        point_request.tolerance = request.tolerance;
        point_request.position_tolerance = request.position_tolerance;
        point_request.axis_tolerance = request.axis_tolerance;
        point_request.damping = request.damping;
        point_request.alpha = request.alpha;
        apply_per_point_solver_options(request, point, point_request);
        if (previous_solution.has_value()) {
            point_request.initial_guess = previous_solution;
        } else {
            point_request.initial_guess = request.initial_guess;
        }

        const InternalIKResult point_ik_result = solve_free_ik_internal(point_request);

        IKTrajectoryPointResult point_result{
            point_ik_result.success,
            i,
            point_ik_result.q,
            point_ik_result.position_error,
            point_ik_result.axis_error,
            point_ik_result.message};

        if (!point_ik_result.success) {
            const AxisFreeError final_error = compute_axis_free_error(
                model_, data_, base_frame_id_, end_effector_frame_id_, point_ik_result.q, point);
            point_result.position_error = final_error.position;
            point_result.axis_error = final_error.axis;
            trajectory_result.points.push_back(std::move(point_result));
            trajectory_result.first_failed_index = i;
            trajectory_result.message =
                "Trajectory IK failed at point " + std::to_string(i) + ": " + point_ik_result.message;
            return trajectory_result;
        }

        if (previous_solution.has_value() && request.max_joint_step_between_points.has_value()) {
            const Eigen::VectorXd joint_step =
                pinocchio::difference(model_, *previous_solution, point_ik_result.q);
            if ((joint_step.cwiseAbs().array() >
                request.max_joint_step_between_points->array()).any())
            {
                point_result.success = false;
                point_result.message =
                    "Joint step between trajectory points exceeds max_joint_step_between_points";
                trajectory_result.points.push_back(std::move(point_result));
                trajectory_result.first_failed_index = i;
                trajectory_result.message =
                    "Trajectory IK failed at point " + std::to_string(i) +
                    ": joint step between adjacent points exceeds configured limit";
                return trajectory_result;
            }
        }

        previous_solution = point_ik_result.q;
        trajectory_result.points.push_back(std::move(point_result));
    }

    trajectory_result.success = true;
    trajectory_result.first_failed_index = std::nullopt;
    trajectory_result.message = "Trajectory IK converged";
    return trajectory_result;
}

const std::vector<std::string> & AxisFreeRotateIK::joint_names() const
{
    return joints_ordered_list_;
}

IKResult AxisFreeRotateIK::check_request_validity(const IKRequest & request)
{
    if (!all_finite(request.desired_position)) {
        return {false, Eigen::VectorXd(), "Desired position contains non-finite values"};
    }

    if (!request.desired_orientation.has_value() &&
        !request.desired_direction.has_value() &&
        !request.desired_up_vector.has_value())
    {
        return {
            false,
            Eigen::VectorXd(),
            "At least one of desired orientation, direction or up vector must be specified"};
    }

    if (request.desired_orientation.has_value()) {
        const Eigen::Matrix3d & desired_orientation = *request.desired_orientation;
        if (!all_finite(desired_orientation)) {
            return {false, Eigen::VectorXd(), "Desired orientation contains non-finite values"};
        }

        const Eigen::Matrix3d orthogonality_error =
            desired_orientation.transpose() * desired_orientation - Eigen::Matrix3d::Identity();
        if (orthogonality_error.norm() > kRotationMatrixTolerance ||
            std::abs(desired_orientation.determinant() - 1.0) > kRotationMatrixTolerance)
        {
            return {false, Eigen::VectorXd(), "Desired orientation is not a valid rotation matrix"};
        }
        if (request.desired_direction.has_value()) {
            return {false, Eigen::VectorXd(),
                    "Cannot specify both desired orientation and desired direction"};
        }
        if (request.desired_up_vector.has_value()) {
            return {false, Eigen::VectorXd(),
                    "Cannot specify both desired orientation and desired up vector"};
        }
    } else {
        if (!request.desired_direction.has_value() || !request.desired_up_vector.has_value()) {
            return {
                false,
                Eigen::VectorXd(),
                "Both desired direction and desired up vector must be specified for free rotation IK"};
        }

        if (!all_finite(*request.desired_direction)) {
            return {false, Eigen::VectorXd(), "Desired direction contains non-finite values"};
        }
        if (!all_finite(*request.desired_up_vector)) {
            return {false, Eigen::VectorXd(), "Desired up vector contains non-finite values"};
        }
        if (request.desired_direction->norm() <= std::numeric_limits<double>::epsilon()) {
            return {false, Eigen::VectorXd(), "Desired direction vector cannot be zero"};
        }
        if (request.desired_up_vector->norm() <= std::numeric_limits<double>::epsilon()) {
            return {false, Eigen::VectorXd(), "Desired up vector cannot be zero"};
        }
    }

    if (!std::isfinite(request.alpha) || request.alpha <= 0.0 || request.alpha > 1.0) {
        return {false, Eigen::VectorXd(), "Alpha must be in the range (0, 1]"};
    }
    if (!is_finite_positive(request.tolerance)) {
        return {false, Eigen::VectorXd(), "Tolerance must be finite and positive"};
    }
    if (request.position_tolerance.has_value() && !is_finite_positive(*request.position_tolerance)) {
        return {false, Eigen::VectorXd(), "Position tolerance must be finite and positive"};
    }
    if (request.axis_tolerance.has_value() && !is_finite_positive(*request.axis_tolerance)) {
        return {false, Eigen::VectorXd(), "Axis tolerance must be finite and positive"};
    }
    if (!is_finite_positive(request.damping)) {
        return {false, Eigen::VectorXd(), "Damping must be finite and positive"};
    }
    if (request.max_iterations <= 0) {
        return {false, Eigen::VectorXd(), "Max iterations must be positive"};
    }

    if (request.initial_guess.has_value()) {
        if (request.initial_guess->size() != model_.nq) {
            return {false, Eigen::VectorXd(), "Initial guess size does not match chain joint count"};
        }
        if (!all_finite(*request.initial_guess)) {
            return {false, Eigen::VectorXd(), "Initial guess contains non-finite values"};
        }
    }

    if (request.delta_max_q.has_value()) {
        if (request.delta_max_q->size() != model_.nv) {
            return {false, Eigen::VectorXd(), "Delta max q size does not match chain joint count"};
        }
        if (!all_finite(*request.delta_max_q)) {
            return {false, Eigen::VectorXd(), "Delta max q contains non-finite values"};
        }
        if ((request.delta_max_q->array() < 0.0).any()) {
            return {false, Eigen::VectorXd(), "Delta max q must be non-negative"};
        }
    }

    return {true, Eigen::VectorXd(), ""};
}

IKResult AxisFreeRotateIK::solve_fix_ik(const IKRequest & request)
{
    SolverWorkspace ws(model_.nq, model_.nv);
    if (request.initial_guess.has_value()) {
        ws.q = *request.initial_guess;
    } else {
        ws.q = pinocchio::neutral(model_);
    }
    clamp_to_model_limits(ws.q);

    double last_position_error_squared = std::numeric_limits<double>::infinity();
    double last_orientation_error_squared = std::numeric_limits<double>::infinity();
    const double tolerance_squared = request.tolerance * request.tolerance;

    for (int iteration = 0; iteration < request.max_iterations; ++iteration) {
        pinocchio::framesForwardKinematics(model_, data_, ws.q);
        const pinocchio::SE3 base_to_end_effector =
            base_to_end_effector_placement(data_, base_frame_id_, end_effector_frame_id_);

        const Eigen::Vector3d position_error =
            request.desired_position - base_to_end_effector.translation();
        const Eigen::Matrix3d rotation_error =
            (*request.desired_orientation) * base_to_end_effector.rotation().transpose();
        const Eigen::Vector3d orientation_error = pinocchio::log3(rotation_error);

        last_position_error_squared = position_error.squaredNorm();
        last_orientation_error_squared = orientation_error.squaredNorm();
        if (last_position_error_squared <= tolerance_squared &&
            last_orientation_error_squared <= tolerance_squared)
        {
            return {true, ws.q, "IK converged"};
        }

        ws.task_error.head<3>() = position_error;
        ws.task_error.tail<3>() = orientation_error;

        compute_frame_jacobian_in_base(ws.q, ws);

        solve_damped_least_squares(ws.jacobian_base, ws.task_error, request.damping, ws);
        ws.delta_q *= request.alpha;
        clip_delta(ws.delta_q, request.delta_max_q);

        if (!ws.delta_q.allFinite()) {
            return {false, ws.q, "IK step contains non-finite values"};
        }

        pinocchio::integrate(model_, ws.q, ws.delta_q, ws.q_next);
        ws.q.swap(ws.q_next);
        clamp_to_model_limits(ws.q);
        if (!ws.q.allFinite()) {
            return {false, ws.q, "IK configuration contains non-finite values"};
        }
    }

    return {
        false,
        ws.q,
        iteration_failure_message(
            request.max_iterations,
            std::sqrt(last_position_error_squared),
            std::sqrt(last_orientation_error_squared))};
}

IKResult AxisFreeRotateIK::solve_free_ik(const IKRequest & request)
{
    const InternalIKResult result = solve_free_ik_internal(request);
    return {result.success, result.q, result.message};
}

AxisFreeRotateIK::InternalIKResult AxisFreeRotateIK::solve_free_ik_internal(const IKRequest & request)
{
    SolverWorkspace ws(model_.nq, model_.nv);
    if (request.initial_guess.has_value()) {
        ws.q = *request.initial_guess;
    } else {
        ws.q = pinocchio::neutral(model_);
    }
    clamp_to_model_limits(ws.q);

    const Eigen::Vector3d desired_axis = request.desired_direction->normalized();
    const Eigen::Vector3d local_axis = request.desired_up_vector->normalized();
    const double position_tolerance = free_position_tolerance(request);
    const double axis_tolerance = free_axis_tolerance(request);
    const double position_tolerance_squared = position_tolerance * position_tolerance;
    const double axis_tolerance_cos = axis_tolerance >= kPi ? -1.0 : std::cos(axis_tolerance);

    double last_position_error_squared = std::numeric_limits<double>::infinity();
    double last_axis_dot = -1.0;

    for (int iteration = 0; iteration < request.max_iterations; ++iteration) {
        pinocchio::framesForwardKinematics(model_, data_, ws.q);
        const pinocchio::SE3 base_to_end_effector =
            base_to_end_effector_placement(data_, base_frame_id_, end_effector_frame_id_);

        const Eigen::Vector3d position_error =
            request.desired_position - base_to_end_effector.translation();
        const Eigen::Vector3d current_axis =
            (base_to_end_effector.rotation() * local_axis).normalized();

        const double axis_dot = std::clamp(current_axis.dot(desired_axis), -1.0, 1.0);
        last_position_error_squared = position_error.squaredNorm();
        last_axis_dot = axis_dot;

        if (last_position_error_squared <= position_tolerance_squared &&
            axis_dot >= axis_tolerance_cos)
        {
            return {
                true,
                ws.q,
                std::sqrt(last_position_error_squared),
                std::acos(axis_dot),
                "IK converged"};
        }

        Eigen::Vector3d axis_error = current_axis.cross(desired_axis);
        if (axis_dot < -1.0 + kOppositeAxisTolerance &&
            axis_error.squaredNorm() < kOppositeAxisTolerance * kOppositeAxisTolerance)
        {
            axis_error = kPi * stable_perpendicular_axis(current_axis);
        }

        ws.task_error.head<3>() = position_error;
        ws.task_error.tail<3>() = axis_error;

        compute_frame_jacobian_in_base(ws.q, ws);

        const Eigen::Matrix3d axis_projection =
            Eigen::Matrix3d::Identity() - current_axis * current_axis.transpose();

        ws.task_jacobian.topRows<3>() = ws.jacobian_base.topRows<3>();
        ws.task_jacobian.bottomRows<3>().noalias() =
            axis_projection * ws.jacobian_base.bottomRows<3>();

        solve_damped_least_squares(ws.task_jacobian, ws.task_error, request.damping, ws);
        ws.delta_q *= request.alpha;
        clip_delta(ws.delta_q, request.delta_max_q);

        if (!ws.delta_q.allFinite()) {
            return {
                false,
                ws.q,
                std::sqrt(last_position_error_squared),
                std::acos(last_axis_dot),
                "IK step contains non-finite values"};
        }

        pinocchio::integrate(model_, ws.q, ws.delta_q, ws.q_next);
        ws.q.swap(ws.q_next);
        clamp_to_model_limits(ws.q);
        if (!ws.q.allFinite()) {
            return {
                false,
                ws.q,
                std::sqrt(last_position_error_squared),
                std::acos(last_axis_dot),
                "IK configuration contains non-finite values"};
        }
    }

    const double last_position_error_norm = std::sqrt(last_position_error_squared);
    const double last_axis_error_norm = std::acos(last_axis_dot);
    return {
        false,
        ws.q,
        last_position_error_norm,
        last_axis_error_norm,
        iteration_failure_message(
            request.max_iterations,
            last_position_error_norm,
            last_axis_error_norm)};
}

} // namespace axis_free_rotate_ik
