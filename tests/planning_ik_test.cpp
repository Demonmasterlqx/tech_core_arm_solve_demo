#include "tech_core/planning_ik.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace tech_core::planning;
namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

// Analytic 3P + ZYX wrist + redundant diagonal translation. World-frame
// angular Jacobian is independently derived from the serial rotation axes.
void fk(const JointVector& q, Eigen::Vector3d& p, Eigen::Matrix3d& r, Jacobian* j) {
    const Eigen::Matrix3d rz = Eigen::AngleAxisd(q[5], Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Matrix3d ry = Eigen::AngleAxisd(q[4], Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Matrix3d rx = Eigen::AngleAxisd(q[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
    p = q.head<3>() + q[6] * Eigen::Vector3d(0.1, -0.2, 0.15);
    r = rz * ry * rx;
    if (j) {
        j->setZero();
        j->topLeftCorner<3, 3>().setIdentity();
        j->block<3, 1>(0, 6) = Eigen::Vector3d(0.1, -0.2, 0.15);
        j->block<3, 1>(3, 3) = rz * ry * Eigen::Vector3d::UnitX();
        j->block<3, 1>(3, 4) = rz * Eigen::Vector3d::UnitY();
        j->block<3, 1>(3, 5) = Eigen::Vector3d::UnitZ();
    }
}

TaskTarget target_at(const JointVector& q) {
    TaskTarget target;
    Eigen::Matrix3d rotation;
    fk(q, target.position, rotation, nullptr);
    target.direction = rotation * target.local_axis;
    return target;
}

void jacobian_test() {
    JointVector q;
    q << .2, -.1, .4, .6, -.4, .3, .2;
    Eigen::Vector3d position;
    Eigen::Matrix3d rotation;
    Jacobian geometric;
    fk(q, position, rotation, &geometric);
    const Eigen::Vector3d local = Eigen::Vector3d(.3, -.2, 1).normalized();
    const Eigen::Vector3d axis = rotation * local;
    const Jacobian task = task_jacobian(geometric, axis);
    const double h = 1e-6;
    for (int k = 0; k < 7; ++k) {
        JointVector plus = q, minus = q;
        plus[k] += h; minus[k] -= h;
        Eigen::Vector3d pp, pm;
        Eigen::Matrix3d rp, rm;
        fk(plus, pp, rp, nullptr); fk(minus, pm, rm, nullptr);
        require(((pp - pm) / (2 * h) - task.col(k).head<3>()).norm() < 1e-9,
                "position Jacobian finite difference");
        const Eigen::Vector3d tangent = axis.cross((rp * local - rm * local) / (2 * h));
        require((tangent - task.col(k).tail<3>()).norm() < 1e-9,
                "axis tangent Jacobian finite difference");
    }
    Eigen::Vector3d delta_p(1e-4, -2e-4, 3e-4), delta_a = 1e-4 * axis.unitOrthogonal();
    const JointVector predictor = predict_task_step(geometric, axis, delta_p, delta_a);
    Eigen::Matrix<double, 6, 1> desired;
    desired << delta_p, delta_a;
    require((task * predictor - desired).norm() < 1e-10, "tangent predictor agreement");
}

void baseline_step_test() {
    JointVector q = JointVector::Zero();
    JointVector goal;
    goal << .1, -.2, .15, .4, -.3, .2, .1;
    TaskTarget target = target_at(goal);
    Eigen::Vector3d p;
    Eigen::Matrix3d r;
    Jacobian jacobian;
    fk(q, p, r, &jacobian);
    const Eigen::Vector3d a = r * target.local_axis;
    Jacobian projected = jacobian;
    projected.bottomRows<3>() = (Eigen::Matrix3d::Identity() - a * a.transpose()) * jacobian.bottomRows<3>();
    Eigen::Matrix<double, 6, 1> error;
    error << target.position - p, a.cross(target.direction);
    Eigen::Matrix<double, 6, 6> normal = projected * projected.transpose();
    normal.diagonal().array() += 1e-8;
    JointVector expected = .2 * projected.transpose() * normal.ldlt().solve(error);
    IKOptions options;
    options.max_iterations = 1;
    options.max_joint_step = JointVector::Constant(.01);
    JointLimits limits;
    limits.lower = JointVector::Constant(-.007);
    limits.upper = JointVector::Constant(.008);
    expected = expected.cwiseMax(-.01).cwiseMin(.01).cwiseMax(limits.lower).cwiseMin(limits.upper);
    const auto result = solve_ik(fk, target, q, limits, options);
    require((result.q - expected).norm() < 1e-13, "upstream baseline one-step parity");
    require(!result.success && result.iterations == 1, "iteration-limit semantics");
}

void convergence_test() {
    JointLimits limits;
    limits.lower = JointVector::Constant(-4);
    limits.upper = JointVector::Constant(4);
    for (const auto method : {IKMethod::Baseline, IKMethod::FullStep,
                             IKMethod::Reduced5D, IKMethod::AdaptiveLM, IKMethod::BoundedQP}) {
        IKOptions options;
        options.method = method;
        int total_iterations = 0;
        for (int sample = 0; sample < 16; ++sample) {
            JointVector goal;
            for (int k = 0; k < 7; ++k) goal[k] = .6 * std::sin(.3 + sample * .7 + k);
            const TaskTarget target = target_at(goal);
            const auto result = solve_ik(fk, target, JointVector::Zero(), limits, options);
            require(result.success, "reachable analytic chain target");
            require(result.position_error <= 1e-4 && result.axis_error <= 1e-4,
                    "task tolerances preserved");
            require((result.q.array() >= limits.lower.array()).all() &&
                    (result.q.array() <= limits.upper.array()).all(), "solution respects limits");
            total_iterations += result.iterations;
        }
        TaskTarget opposite;
        opposite.direction = -Eigen::Vector3d::UnitZ();
        const auto result = solve_ik(fk, opposite, JointVector::Zero(), limits, options);
        require(result.success && result.axis_error <= 1e-4, "antiparallel axis must escape");
        std::cout << "method=" << static_cast<int>(method)
                  << " reachable=16/16 iterations=" << total_iterations
                  << " antiparallel_iterations=" << result.iterations << '\n';
    }
    limits.lower.setZero(); limits.upper.setZero();
    TaskTarget unreachable;
    unreachable.position.x() = 1;
    const auto failure = solve_ik(fk, unreachable, JointVector::Zero(), limits);
    require(!failure.success && failure.q.isZero() && failure.position_error == 1,
            "fixed joint limits cannot claim success");
}

void fixed_coordinate_test() {
    const auto redundant = [](const JointVector& q,Eigen::Vector3d& p,Eigen::Matrix3d& r,Jacobian* j) {
        p=Eigen::Vector3d(10*q[0]+q[6],q[1],q[2]);r.setIdentity();
        if(j) {j->setZero();(*j)(0,0)=10;(*j)(0,6)=1;(*j)(1,1)=1;(*j)(2,2)=1;}
    };
    JointLimits limits;limits.lower.setConstant(-1);limits.upper.setConstant(1);
    limits.lower[0]=limits.upper[0]=.1;
    JointVector start=JointVector::Zero();start[0]=.1;
    TaskTarget target;target.position.x()=1.3;
    for(auto method:{IKMethod::Baseline,IKMethod::Reduced5D}) {
        IKOptions options;options.method=method;
        const auto result=solve_ik(redundant,target,start,limits,options);
        require(result.success && result.q[0]==.1 && std::abs(result.q[6]-.3)<1e-4,
                "Fixed coordinates must reallocate the task to available joints");
    }
}

void bounded_qp_test() {
    // The high-leverage actuator q0 is already at its upper bound. A lower
    // leverage redundant actuator q6 can reach the target exactly. Post-clamp
    // DLS allocates 100/101 of the requested displacement to blocked q0 on
    // every iteration, whereas the constrained optimum reallocates it to q6.
    const auto redundant_fk = [](const JointVector& q, Eigen::Vector3d& p,
                                 Eigen::Matrix3d& r, Jacobian* j) {
        p = Eigen::Vector3d(10 * q[0] + q[6], q[1], q[2]);
        r.setIdentity();
        if (j) {
            j->setZero(); (*j)(0, 0) = 10; (*j)(0, 6) = 1;
            (*j)(1, 1) = 1; (*j)(2, 2) = 1;
        }
    };
    JointVector initial = JointVector::Zero(); initial[0] = .1;
    JointLimits limits;
    limits.lower.setConstant(-1); limits.upper.setConstant(1); limits.upper[0] = .1;
    TaskTarget target; target.position.x() = 1.3;
    IKOptions options; options.method = IKMethod::FullStep;
    const auto clipped = solve_ik(redundant_fk, target, initial, limits, options);
    options.method = IKMethod::BoundedQP;
    const auto bounded = solve_ik(redundant_fk, target, initial, limits, options);
    require(!clipped.success && clipped.iterations == 300 && clipped.position_error > .01,
            "post-clamp DLS stalls on blocked high-leverage joint");
    require(bounded.success && bounded.iterations == 1 && bounded.position_error <= 1e-4,
            "box QP must redistribute to redundant feasible joint");
    require(std::abs(bounded.q[0] - .1) < 1e-12 && std::abs(bounded.q[6] - .3) < 1e-6,
            "bounded optimum uses the compensating actuator");
    std::cout << "bounded redundancy: clipped_error=" << clipped.position_error
              << " QP_error=" << bounded.position_error << " QP_iterations=" << bounded.iterations << '\n';

    // Independently inspect primal feasibility and gradient KKT conditions on
    // 80 deterministic correlated linear tasks with asymmetric joint boxes,
    // fixed joints, and a separate max_joint_step restriction. For a convex
    // quadratic these conditions certify the globally optimal local step.
    for (int sample = 0; sample < 80; ++sample) {
        Eigen::Matrix<double, 3, 7> linear;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 7; ++j)
                linear(i, j) = std::sin(.31 + .7 * sample + .3 * i + .9 * j) +
                    .4 * std::cos(.6 * sample + 1.3 * i - .2 * j);
        const auto linear_fk = [&](const JointVector& q, Eigen::Vector3d& p,
                                   Eigen::Matrix3d& r, Jacobian* jacobian) {
            p = linear * q; r.setIdentity();
            if (jacobian) { jacobian->setZero(); jacobian->topRows<3>() = linear; }
        };
        target.position = Eigen::Vector3d(.7 * std::sin(sample + .2),
                                          .5 * std::cos(sample + .4), .4);
        for (int j = 0; j < 7; ++j) {
            limits.lower[j] = -.02 - .15 * std::abs(std::sin(sample + j));
            limits.upper[j] = .01 + .17 * std::abs(std::cos(.4 * sample + j));
        }
        if (sample % 3 == 0) limits.lower[2] = limits.upper[2] = 0;
        options.max_iterations = 1;
        options.damping = sample % 2 == 0 ? .07 : 1e-4;
        options.max_joint_step = JointVector::Constant(.08);
        const auto solved = solve_ik(linear_fk, target, JointVector::Zero(), limits, options);
        require(solved.reason != "bounded QP failed KKT or pivot limit", "box QP solves correlated task");
        const JointVector lower = limits.lower.cwiseMax(-.08);
        const JointVector upper = limits.upper.cwiseMin(.08);
        const JointVector gradient = linear.transpose() * (linear * solved.q - target.position) +
            options.damping * options.damping * solved.q;
        for (int j = 0; j < 7; ++j) {
            require(solved.q[j] >= lower[j] - 1e-12 && solved.q[j] <= upper[j] + 1e-12,
                    "QP primal feasibility");
            if (lower[j] == upper[j]) continue;
            if (solved.q[j] <= lower[j] + 1e-9)
                require(gradient[j] >= -1e-8, "QP lower-bound dual sign");
            else if (solved.q[j] >= upper[j] - 1e-9)
                require(gradient[j] <= 1e-8, "QP upper-bound dual sign");
            else require(std::abs(gradient[j]) < 1e-8, "QP free-coordinate stationarity");
        }
    }
    options.should_stop = [] { return true; };
    require(solve_ik(redundant_fk, target, initial, limits, options).reason == "stopped",
            "bounded method respects termination");
    int stop_checks = 0;
    options.should_stop = [&] { return ++stop_checks > 1; };
    const auto pivot_stop = solve_ik(redundant_fk, target, JointVector::Zero(), limits, options);
    require(pivot_stop.reason == "stopped" && pivot_stop.fk_evals == 1,
            "termination checked inside QP active-set loop");
    options.should_stop = {};
    limits.lower.setZero(); limits.upper.setZero();
    target.position = Eigen::Vector3d::UnitX();
    const auto fixed = solve_ik(redundant_fk, target, JointVector::Zero(), limits, options);
    require(!fixed.success && fixed.q.isZero() && fixed.reason == "no descending feasible step",
            "all-fixed QP terminates without false success");
}

void invalid_test() {
    TaskTarget target;
    JointVector initial = JointVector::Zero();
    JointLimits limits;
    target.direction.setZero();
    require(!solve_ik(fk, target, initial, limits).success, "zero direction rejected");
    target.direction = Eigen::Vector3d::UnitZ();
    initial[0] = std::numeric_limits<double>::quiet_NaN();
    require(solve_ik(fk, target, initial, limits).fk_evals == 0, "NaN input rejected before FK");
    initial.setZero();
    limits.lower[0] = 1; limits.upper[0] = -1;
    require(solve_ik(fk, target, initial, limits).fk_evals == 0, "inverted limits rejected");
    limits = JointLimits{};
    IKOptions invalid_options;
    invalid_options.alpha = 1.01;
    require(solve_ik(fk, target, initial, limits, invalid_options).fk_evals == 0,
            "alpha above one rejected before FK");
    invalid_options.alpha = .2;
    invalid_options.max_joint_step = JointVector::Constant(-.1);
    require(solve_ik(fk, target, initial, limits, invalid_options).fk_evals == 0,
            "negative maximum joint step rejected before FK");
    const auto bad_fk = [](const JointVector&, Eigen::Vector3d& p, Eigen::Matrix3d& r, Jacobian* j) {
        p.setConstant(std::numeric_limits<double>::infinity()); r.setIdentity();
        if (j) j->setZero();
    };
    require(!solve_ik(bad_fk, target, initial, limits).success, "nonfinite FK rejected");
    IKOptions stop;
    stop.should_stop = [] { return true; };
    const auto stopped = solve_ik(fk, target, initial, limits, stop);
    require(stopped.reason == "stopped" && stopped.fk_evals == 0, "deadline before FK");
    int calls = 0;
    stop.method = IKMethod::FullStep;
    stop.should_stop = [&] { return ++calls > 1; };
    target.position.x() = .1;
    const auto mid_stop = solve_ik(fk, target, initial, limits, stop);
    require(mid_stop.reason == "stopped" && mid_stop.fk_evals == 1,
            "deadline checked before backtracking FK");
}
} // namespace

int main() {
    try {
        jacobian_test(); baseline_step_test(); convergence_test(); fixed_coordinate_test(); invalid_test(); bounded_qp_test();
        std::cout << "planning IK tests passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
