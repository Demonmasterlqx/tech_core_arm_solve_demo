#pragma once

#include <Eigen/Core>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace tech_core::planning {

using JointVector = Eigen::VectorXd;
// Geometric point Jacobian expressed in the same frame as the target:
// controlled-point linear velocity above angular velocity.
using Jacobian = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using ForwardKinematics = std::function<void(
    const JointVector&, Eigen::Vector3d&, Eigen::Matrix3d&, Jacobian*)>;

enum class IKMethod { Baseline, FullStep, Reduced5D, AdaptiveLM, BoundedQP };

struct TaskTarget {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d direction = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d local_axis = Eigen::Vector3d::UnitZ();
};

struct JointLimits {
    explicit JointLimits(int dof = 7)
        : lower(JointVector::Constant(dof, -std::numeric_limits<double>::infinity())),
          upper(JointVector::Constant(dof, std::numeric_limits<double>::infinity())), angular(dof, true) {}
    JointVector lower, upper;
    std::vector<bool> angular;
};

struct IKOptions {
    IKMethod method = IKMethod::Baseline;
    int max_iterations = 300;
    double position_tolerance = 1e-4;
    double axis_tolerance = 1e-4;
    double damping = 1e-4;
    // Baseline alone uses alpha; accelerated methods start with a full step.
    double alpha = 0.2;
    std::optional<JointVector> max_joint_step;
    int max_backtracks = 12;
    // Reduced5D/AdaptiveLM use a Euclidean radius. BoundedQP includes a
    // per-joint box radius in its optimization, together with joint limits.
    double trust_radius = 0.5;
    // True requests termination; checked before every FK, including line searches.
    std::function<bool()> should_stop;
};

struct IKResult {
    bool success = false;
    JointVector q;
    int iterations = 0;  // Number of attempted joint updates.
    int fk_evals = 0;    // Includes FK-only line searches and final evaluation.
    double position_error = std::numeric_limits<double>::infinity();
    double axis_error = std::numeric_limits<double>::infinity();
    std::string reason;
};

// Projects angular velocity onto the two-dimensional tangent plane of the axis.
// This is the task velocity Jacobian, not the derivative of a cross-product
// residual away from the solution.
Jacobian task_jacobian(const Jacobian& geometric, const Eigen::Vector3d& current_axis);

// Damped tangent predictor; angular_delta is an angular displacement, not a
// direction-vector difference. Callers apply limits and check the resulting FK.
JointVector predict_task_step(const Jacobian& geometric,
                             const Eigen::Vector3d& current_axis,
                             const Eigen::Vector3d& position_delta,
                             const Eigen::Vector3d& angular_delta,
                             double damping = 1e-4);

IKResult solve_ik(const ForwardKinematics& fk, const TaskTarget& target,
                  const JointVector& initial, const JointLimits& limits,
                  const IKOptions& options = {});

} // namespace tech_core::planning
