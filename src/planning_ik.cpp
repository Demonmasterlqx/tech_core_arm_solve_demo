#include "tech_core/planning_ik.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <algorithm>
#include <vector>
#include <cmath>
#include <stdexcept>

namespace tech_core::planning {
namespace {
constexpr double pi = 3.14159265358979323846;
using Error = Eigen::Matrix<double, 6, 1>;

Eigen::Vector3d perpendicular(const Eigen::Vector3d& axis) {
    const Eigen::Vector3d basis = std::abs(axis.x()) > 0.9
        ? Eigen::Vector3d::UnitY() : Eigen::Vector3d::UnitX();
    return axis.cross(basis).normalized();
}

JointVector dls(const Jacobian& jacobian, const Error& error, double damping) {
    Eigen::Matrix<double, 6, 6> normal = jacobian * jacobian.transpose();
    normal.diagonal().array() += damping * damping;
    return jacobian.transpose() * normal.ldlt().solve(error);
}

JointVector reduced_dls(const Jacobian& geometric, const Eigen::Vector3d& axis,
                        const Error& error, double damping) {
    Eigen::Matrix<double, 2, 3> basis;
    basis.row(0) = perpendicular(axis).transpose();
    basis.row(1) = axis.cross(basis.row(0).transpose()).transpose();
    Eigen::Matrix<double, 5, Eigen::Dynamic> jacobian(5, geometric.cols());
    jacobian.topRows<3>() = geometric.topRows<3>();
    jacobian.bottomRows<2>().noalias() = basis * geometric.bottomRows<3>();
    Eigen::Matrix<double, 5, 1> reduced_error;
    reduced_error.head<3>() = error.head<3>();
    reduced_error.tail<2>().noalias() = basis * error.tail<3>();
    Eigen::Matrix<double, 5, 5> normal = jacobian * jacobian.transpose();
    normal.diagonal().array() += damping * damping;
    const Eigen::LLT<Eigen::Matrix<double, 5, 5>> factor(normal);
    if (factor.info() != Eigen::Success)
        return JointVector::Constant(geometric.cols(), std::numeric_limits<double>::quiet_NaN());
    return jacobian.transpose() * factor.solve(reduced_error);
}

struct BoxStep {
    JointVector step;
    bool success = false;
    bool stopped = false;
};

// Strictly convex bound-constrained quadratic:
// .5*x'*(J'J + lambda^2 I)*x - (J'e)'*x, lower <= x <= upper.
// A primal feasible active-set method: solve on the free coordinates, move
// only as far as the first blocking bound, then release active constraints
// whose gradient signs violate KKT. Clamping an unconstrained step cannot
// perform this redistribution and is deliberately not used here.
BoxStep bounded_step(const Jacobian& jacobian, const Error& error, double damping,
                     const JointVector& lower, const JointVector& upper,
                     const std::function<bool()>& should_stop) {
    using Hessian = Eigen::MatrixXd;
    const int n = jacobian.cols();
    Hessian hessian = jacobian.transpose() * jacobian;
    hessian.diagonal().array() += damping * damping;
    const JointVector linear = -jacobian.transpose() * error;
    BoxStep result;
    result.step = JointVector::Zero(n);
    std::vector<int> active(n, 0); // -1 lower, +1 upper, 2 fixed, 0 free.
    for (int i = 0; i < n; ++i)
        if (lower[i] == upper[i]) { active[i] = 2; result.step[i] = lower[i]; }
    // A handful of pivots normally suffice; finite cap and callback bound degenerate cases.
    for (int pivot = 0; pivot < std::max(128, 16*n); ++pivot) {
        if (should_stop && should_stop()) { result.stopped = true; return result; }
        JointVector fixed = JointVector::Zero(n);
        for (int i = 0; i < n; ++i) if (active[i]) fixed[i] = result.step[i];
        Hessian free_hessian = hessian;
        JointVector rhs = -linear - hessian * fixed;
        for (int i = 0; i < n; ++i) if (active[i]) {
            free_hessian.row(i).setZero();
            free_hessian.col(i).setZero();
            free_hessian(i, i) = 1;
            rhs[i] = fixed[i];
        }
        const Eigen::LDLT<Hessian> factor(free_hessian);
        if (factor.info() != Eigen::Success) return result;
        const JointVector minimum = factor.solve(rhs);
        if (!minimum.allFinite()) return result;
        const JointVector direction = minimum - result.step;
        double fraction = 1;
        int blocking = -1, blocking_side = 0;
        for (int i = 0; i < n; ++i) if (!active[i]) {
            double candidate = 1;
            int side = 0;
            if (minimum[i] < lower[i] && direction[i] < 0) {
                candidate = (lower[i] - result.step[i]) / direction[i]; side = -1;
            } else if (minimum[i] > upper[i] && direction[i] > 0) {
                candidate = (upper[i] - result.step[i]) / direction[i]; side = 1;
            }
            if (side && candidate < fraction) {
                fraction = std::max(0.0, candidate); blocking = i; blocking_side = side;
            }
        }
        result.step += fraction * direction;
        if (blocking >= 0) {
            active[blocking] = blocking_side;
            result.step[blocking] = blocking_side < 0 ? lower[blocking] : upper[blocking];
            continue;
        }
        const JointVector gradient = hessian * result.step + linear;
        const double tolerance = 1e-10 * (1 + linear.lpNorm<Eigen::Infinity>() +
            hessian.cwiseAbs().rowwise().sum().maxCoeff() * result.step.lpNorm<Eigen::Infinity>());
        int release = -1;
        double worst = tolerance;
        for (int i = 0; i < n; ++i) {
            const double violation = active[i] == -1 ? -gradient[i] : active[i] == 1 ? gradient[i] : 0;
            if (violation > worst) { worst = violation; release = i; }
        }
        if (release < 0) {
            // Roundoff alone can place a free solution just outside its box.
            // Reject significant infeasibility or stationarity loss rather than
            // returning a non-optimal clipped update as an optimized step.
            for (int i = 0; i < n; ++i)
                if (result.step[i] < lower[i] - 1e-12 || result.step[i] > upper[i] + 1e-12 ||
                    (!active[i] && std::abs(gradient[i]) > 10 * tolerance)) return result;
            result.success = true;
            return result;
        }
        active[release] = 0;
    }
    return result;
}

struct State {
    Eigen::Vector3d axis;
    Error error;
    double position_squared = 0;
    double axis_dot = 0;
    double angle = 0;
    double objective = 0;
};

bool evaluate(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation,
              const TaskTarget& target, State& state) {
    if (!position.allFinite() || !rotation.allFinite()) return false;
    state.axis = rotation * target.local_axis;
    const double norm = state.axis.norm();
    if (!(norm > 0) || !std::isfinite(norm)) return false;
    state.axis /= norm;
    state.error.head<3>() = target.position - position;
    state.error.tail<3>() = state.axis.cross(target.direction);
    state.axis_dot = std::clamp(state.axis.dot(target.direction), -1.0, 1.0);
    if (state.axis_dot < -1.0 + 1e-6 && state.error.tail<3>().squaredNorm() < 1e-12)
        state.error.tail<3>() = pi * perpendicular(state.axis);
    state.position_squared = state.error.head<3>().squaredNorm();
    state.angle = std::acos(state.axis_dot);
    state.objective = state.position_squared + state.angle * state.angle;
    return state.error.allFinite() && std::isfinite(state.objective);
}

bool valid(const ForwardKinematics& fk, const TaskTarget& target,
           const JointVector& q, const JointLimits& limits, const IKOptions& options) {
    if (!fk || q.size() == 0 || limits.lower.size() != q.size() || limits.upper.size() != q.size() || limits.angular.size() != static_cast<std::size_t>(q.size()) || !q.allFinite() || !target.position.allFinite() ||
        !target.direction.allFinite() || !target.local_axis.allFinite() ||
        !(target.direction.norm() > 0) || !std::isfinite(target.direction.norm()) ||
        !(target.local_axis.norm() > 0) || !std::isfinite(target.local_axis.norm())) return false;
    for (int i = 0; i < q.size(); ++i)
        if (std::isnan(limits.lower[i]) || std::isnan(limits.upper[i]) ||
            limits.lower[i] > limits.upper[i] ||
            limits.lower[i] == std::numeric_limits<double>::infinity() ||
            limits.upper[i] == -std::numeric_limits<double>::infinity()) return false;
    const auto positive = [](double x) { return std::isfinite(x) && x > 0; };
    const bool known_method = options.method == IKMethod::Baseline || options.method == IKMethod::FullStep ||
        options.method == IKMethod::Reduced5D || options.method == IKMethod::AdaptiveLM ||
        options.method == IKMethod::BoundedQP;
    return known_method && options.max_iterations > 0 && options.max_backtracks >= 0 &&
        positive(options.position_tolerance) && positive(options.axis_tolerance) &&
        positive(options.damping) && positive(options.alpha) && options.alpha <= 1 && positive(options.trust_radius) &&
        (!options.max_joint_step || (options.max_joint_step->size() == q.size() && options.max_joint_step->allFinite() &&
                                    (options.max_joint_step->array() >= 0).all()));
}

void clip_step(JointVector& step, const IKOptions& options) {
    if (options.max_joint_step)
        for (int i = 0; i < step.size(); ++i) {
            const double limit = std::abs((*options.max_joint_step)[i]);
            step[i] = std::clamp(step[i], -limit, limit);
        }
}

JointVector clamp(const JointVector& q, const JointLimits& limits) {
    return q.cwiseMax(limits.lower).cwiseMin(limits.upper);
}
} // namespace

Jacobian task_jacobian(const Jacobian& geometric, const Eigen::Vector3d& current_axis) {
    if (geometric.cols()==0 || !geometric.allFinite() || !current_axis.allFinite() || !(current_axis.norm() > 0) ||
        !std::isfinite(current_axis.norm()))
        throw std::invalid_argument("task_jacobian requires finite inputs and nonzero axis");
    const Eigen::Vector3d axis = current_axis.normalized();
    Jacobian result(6, geometric.cols());
    result.topRows<3>() = geometric.topRows<3>();
    result.bottomRows<3>().noalias() =
        (Eigen::Matrix3d::Identity() - axis * axis.transpose()) * geometric.bottomRows<3>();
    return result;
}

JointVector predict_task_step(const Jacobian& geometric, const Eigen::Vector3d& current_axis,
                             const Eigen::Vector3d& position_delta,
                             const Eigen::Vector3d& angular_delta, double damping) {
    if (!position_delta.allFinite() || !angular_delta.allFinite() ||
        !std::isfinite(damping) || damping <= 0)
        throw std::invalid_argument("Invalid task predictor input");
    Error error;
    error.head<3>() = position_delta;
    error.tail<3>() = angular_delta;
    return dls(task_jacobian(geometric, current_axis), error, damping);
}

IKResult solve_ik(const ForwardKinematics& fk, const TaskTarget& input_target,
                  const JointVector& initial, const JointLimits& limits,
                  const IKOptions& options) {
    IKResult result;
    result.q = initial;
    if (!valid(fk, input_target, initial, limits, options)) {
        result.reason = "invalid input";
        return result;
    }
    TaskTarget target = input_target;
    target.direction.normalize();
    target.local_axis.normalize();
    result.q = clamp(initial, limits);
    const double tolerance_squared = options.position_tolerance * options.position_tolerance;
    const double tolerance_cos = options.axis_tolerance >= pi ? -1 : std::cos(options.axis_tolerance);
    auto stopped = [&]() { return options.should_stop && options.should_stop(); };
    auto report = [&](const State& state) {
        result.position_error = std::sqrt(state.position_squared);
        result.axis_error = state.angle;
    };
    auto converged = [&](const State& state) {
        return state.position_squared <= tolerance_squared && state.axis_dot >= tolerance_cos;
    };
    double adaptive_damping = std::max(options.damping, 1e-2);
    State state;
    Eigen::Vector3d position;
    Eigen::Matrix3d rotation;
    Jacobian geometric(6, initial.size());
    for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
        if (stopped()) { result.reason = "stopped"; return result; }
        fk(result.q, position, rotation, &geometric);
        ++result.fk_evals;
        if (geometric.cols() != initial.size() || !geometric.allFinite() || !evaluate(position, rotation, target, state)) {
            result.reason = "non-finite kinematics"; return result;
        }
        report(state);
        if (converged(state)) { result.success = true; result.reason = "converged"; return result; }
        // A fixed coordinate has no available velocity. Removing its column
        // lets the remaining joints share the task instead of repeatedly
        // allocating motion to a coordinate that will be clamped away.
        for (int joint = 0; joint < initial.size(); ++joint)
            if (limits.lower[joint] == limits.upper[joint]) geometric.col(joint).setZero();
        ++result.iterations;
        Jacobian task(6, initial.size());
        if (options.method != IKMethod::Reduced5D) {
            task.topRows<3>() = geometric.topRows<3>();
            task.bottomRows<3>().noalias() =
                (Eigen::Matrix3d::Identity() - state.axis * state.axis.transpose()) * geometric.bottomRows<3>();
        }
        if (options.method == IKMethod::Baseline) {
            // Upstream: six-row LDLT, damping squared, alpha, per-joint clip, limits.
            JointVector step = options.alpha * dls(task, state.error, options.damping);
            clip_step(step, options);
            if (!step.allFinite()) { result.reason = "non-finite step"; return result; }
            result.q = clamp(result.q + step, limits);
            continue;
        }

        const bool adaptive = options.method == IKMethod::AdaptiveLM;
        const double damping = adaptive ? adaptive_damping : options.damping;
        JointVector step;
        if (options.method == IKMethod::BoundedQP) {
            JointVector radius = JointVector::Constant(initial.size(), options.trust_radius);
            if (options.max_joint_step) radius = radius.cwiseMin(*options.max_joint_step);
            const JointVector lower = (limits.lower - result.q).cwiseMax(-radius);
            const JointVector upper = (limits.upper - result.q).cwiseMin(radius);
            const BoxStep bounded = bounded_step(task, state.error, damping, lower, upper, options.should_stop);
            if (!bounded.success) {
                result.reason = bounded.stopped ? "stopped" : "bounded QP failed KKT or pivot limit";
                return result;
            }
            step = bounded.step;
        } else {
            step = options.method == IKMethod::Reduced5D
                ? reduced_dls(geometric, state.axis, state.error, damping)
                : dls(task, state.error, damping);
        }
        if (!step.allFinite()) { result.reason = "non-finite step"; return result; }
        if ((adaptive || options.method == IKMethod::Reduced5D) && step.norm() > options.trust_radius)
            step *= options.trust_radius / step.norm();
        if (options.method != IKMethod::BoundedQP) clip_step(step, options);
        bool accepted = false;
        double fraction = 1;
        for (int backtrack = 0; backtrack <= options.max_backtracks; ++backtrack) {
            if (stopped()) { result.reason = "stopped"; return result; }
            const JointVector trial = clamp(result.q + fraction * step, limits);
            if ((trial - result.q).squaredNorm() < 1e-28) break;
            State candidate;
            fk(trial, position, rotation, nullptr);
            ++result.fk_evals;
            if (evaluate(position, rotation, target, candidate) && candidate.objective < state.objective) {
                if (adaptive) {
                    const Error predicted = state.error - task * (trial - result.q);
                    const double predicted_decrease = state.error.squaredNorm() - predicted.squaredNorm();
                    const double ratio = (state.objective - candidate.objective) /
                        std::max(predicted_decrease, 1e-16);
                    if (ratio > 0.75) adaptive_damping = std::max(options.damping, adaptive_damping * 0.5);
                    else if (ratio < 0.25) adaptive_damping = std::min(1e3, adaptive_damping * 2);
                }
                result.q = trial;
                report(candidate);
                accepted = true;
                if (converged(candidate)) {
                    result.success = true; result.reason = "converged"; return result;
                }
                break;
            }
            fraction *= 0.5;
        }
        if (!accepted) {
            if (adaptive && adaptive_damping < 1e3) {
                adaptive_damping = std::min(1e3, adaptive_damping * 10);
                continue;
            }
            result.reason = "no descending feasible step";
            return result;
        }
    }
    // Report errors at the returned configuration (upstream returns stale errors
    // after its final update). This FK does not change the baseline joint path.
    if (stopped()) { result.reason = "stopped"; return result; }
    fk(result.q, position, rotation, nullptr);
    ++result.fk_evals;
    if (!evaluate(position, rotation, target, state)) {
        result.position_error = result.axis_error = std::numeric_limits<double>::infinity();
        result.reason = "non-finite kinematics";
        return result;
    }
    report(state);
    result.reason = "iteration limit";
    return result;
}

} // namespace tech_core::planning
