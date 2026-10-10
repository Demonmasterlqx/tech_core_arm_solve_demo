#include "tech_core/arm_simulation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tech_core {
namespace {
constexpr double position_gain = 8.0;
constexpr double rotation_weight = 0.15;
constexpr double damping = 0.005;

// Positive-definite damped normal matrix, solved by Cholesky without a new
// linear-algebra dependency. Damping makes singular configurations solvable.
std::array<double, 6> solve(std::array<double, 36> matrix, std::array<double, 6> rhs) {
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j <= i; ++j) {
            double value = matrix[6*i+j];
            for (int k = 0; k < j; ++k) value -= matrix[6*i+k]*matrix[6*j+k];
            if (i == j) {
                if (!(value > 0) || !std::isfinite(value))
                    throw std::runtime_error("Invalid arm Jacobian normal matrix");
                matrix[6*i+j] = std::sqrt(value);
            } else matrix[6*i+j] = value / matrix[6*j+j];
        }
    }
    for (int i = 0; i < 6; ++i) {
        for (int k = 0; k < i; ++k) rhs[i] -= matrix[6*i+k]*rhs[k];
        rhs[i] /= matrix[6*i+i];
    }
    for (int i = 5; i >= 0; --i) {
        for (int k = i+1; k < 6; ++k) rhs[i] -= matrix[6*k+i]*rhs[k];
        rhs[i] /= matrix[6*i+i];
    }
    return rhs;
}
}

ArmSimulation::ArmSimulation(std::string model_path) {
    char error[2048]{};
    model_.reset(mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error)));
    if (!model_) throw std::runtime_error(std::string("MuJoCo arm model: ") + error);
    // This API owns all moving joints. Reject a free base or extra mechanism
    // rather than silently integrating only part of a different scene.
    for (int j = 0; j < model_->njnt; ++j) {
        const char* name = mj_id2name(model_.get(), mjOBJ_JOINT, j);
        if (model_->jnt_type[j] != mjJNT_HINGE || !model_->jnt_limited[j] || !name)
            throw std::runtime_error("Arm mock requires named, limited hinge joints and a fixed base");
        const double low = model_->jnt_range[2*j], high = model_->jnt_range[2*j+1];
        if (!std::isfinite(low) || !std::isfinite(high) || low >= high)
            throw std::runtime_error("Invalid arm joint range");
        joints_.push_back(j); names_.emplace_back(name); limits_.push_back({low, high});
        velocity_limits_.push_back(maximum_joint_speed);
    }
    if (joints_.empty()) throw std::runtime_error("Arm model has no joints");
    tool_ = mj_name2id(model_.get(), mjOBJ_SITE, "arm_tool");
    home_ = mj_name2id(model_.get(), mjOBJ_KEY, "home");
    if (tool_ < 0 || home_ < 0) throw std::runtime_error("Arm model requires arm_tool site and home keyframe");
    data_.reset(mj_makeData(model_.get()));
    if (!data_) throw std::runtime_error("Unable to allocate arm mjData");
    reset();
}

void ArmSimulation::reset() {
    mj_resetDataKeyframe(model_.get(), data_.get(), home_);
    data_->time = 0;
    mju_zero(data_->qvel, model_->nv);
    mj_forward(model_.get(), data_.get());
    if (!finite()) throw std::runtime_error("Nonfinite arm home configuration");
    const auto q = joint_positions();
    for (std::size_t i = 0; i < q.size(); ++i)
        if (q[i] < limits_[i][0] || q[i] > limits_[i][1])
            throw std::runtime_error("Arm home configuration exceeds joint limits");
    stop();
}

void ArmSimulation::validate_command(const std::vector<double>& values) const {
    if (values.size() != joints_.size()) throw std::invalid_argument("Arm command dimension must match joint_names()");
    for (double value : values)
        if (!std::isfinite(value)) throw std::invalid_argument("Arm commands must be finite");
}

void ArmSimulation::set_joint_positions(const std::vector<double>& positions) {
    validate_command(positions);
    position_command_ = positions;
    for (std::size_t i = 0; i < joints_.size(); ++i)
        position_command_[i] = std::clamp(position_command_[i], limits_[i][0], limits_[i][1]);
    mode_ = ArmControlMode::Position;
}

void ArmSimulation::set_joint_velocities(const std::vector<double>& velocities) {
    validate_command(velocities);
    velocity_command_ = velocities;
    for (std::size_t i = 0; i < joints_.size(); ++i)
        velocity_command_[i] = std::clamp(velocity_command_[i], -velocity_limits_[i], velocity_limits_[i]);
    mode_ = ArmControlMode::Velocity;
}

void ArmSimulation::set_cartesian_velocity(const std::array<double, 3>& velocity) {
    for (double value : velocity)
        if (!std::isfinite(value)) throw std::invalid_argument("Cartesian velocity must be finite");
    if (!finite()) throw std::runtime_error("Nonfinite arm state");
    if (mode_ != ArmControlMode::Cartesian)
        std::copy_n(data_->site_xmat + 9*tool_, 9, orientation_reference_.begin());
    // Scale using a normalized intermediate to avoid overflow on large finite inputs.
    const double largest = std::max({std::abs(velocity[0]), std::abs(velocity[1]), std::abs(velocity[2])});
    cartesian_command_ = velocity;
    if (largest > 0) {
        const double norm = std::hypot(velocity[0]/largest, velocity[1]/largest, velocity[2]/largest);
        if (largest > maximum_cartesian_speed/norm)
            for (int k = 0; k < 3; ++k) cartesian_command_[k] = (velocity[k]/largest)*(maximum_cartesian_speed/norm);
    }
    mode_ = ArmControlMode::Cartesian;
}

void ArmSimulation::stop() {
    if (!finite()) throw std::runtime_error("Cannot hold nonfinite arm state");
    position_command_ = joint_positions();
    velocity_command_.assign(joints_.size(), 0);
    cartesian_command_ = {};
    mju_zero(data_->qvel, model_->nv);
    mode_ = ArmControlMode::Position;
    mj_forward(model_.get(), data_.get());
}

std::vector<double> ArmSimulation::cartesian_rates() const {
    const int n = static_cast<int>(joints_.size());
    std::vector<mjtNum> translation(3*model_->nv), rotation(3*model_->nv);
    mj_jacSite(model_.get(), data_.get(), translation.data(), rotation.data(), tool_);
    std::vector<double> jacobian(6*n);
    for (int row = 0; row < 3; ++row) for (int j = 0; j < n; ++j) {
        const int dof = model_->jnt_dofadr[joints_[j]];
        jacobian[row*n+j] = translation[row*model_->nv+dof];
        jacobian[(row+3)*n+j] = rotation_weight*rotation[row*model_->nv+dof];
    }
    std::array<double, 6> target{cartesian_command_[0], cartesian_command_[1], cartesian_command_[2], 0, 0, 0};
    const double* current = data_->site_xmat + 9*tool_;
    // Sum cross products of corresponding orientation columns in WORLD frame.
    // The modest orientation feedback corrects integration drift; translation
    // and orientation remain approximate near singularities and joint stops.
    for (int axis = 0; axis < 3; ++axis) {
        const int a = (axis+1)%3, b = (axis+2)%3;
        double error = 0;
        for (int col = 0; col < 3; ++col)
            error += current[3*a+col]*orientation_reference_[3*b+col] - current[3*b+col]*orientation_reference_[3*a+col];
        target[axis+3] = rotation_weight*std::clamp(1.5*error, -0.5, 0.5);
    }
    std::array<double, 36> normal{};
    for (int r = 0; r < 6; ++r) for (int c = 0; c < 6; ++c) {
        for (int j = 0; j < n; ++j) normal[6*r+c] += jacobian[r*n+j]*jacobian[c*n+j];
        if (r == c) normal[6*r+c] += damping*damping;
    }
    const auto multiplier = solve(normal, target);
    std::vector<double> rates(n, 0);
    double scale = 1;
    for (int j = 0; j < n; ++j) {
        for (int r = 0; r < 6; ++r) rates[j] += jacobian[r*n+j]*multiplier[r];
        if (!std::isfinite(rates[j])) throw std::runtime_error("Nonfinite arm inverse kinematics");
        scale = std::max(scale, std::abs(rates[j])/velocity_limits_[j]);
    }
    for (double& rate : rates) rate /= scale;
    return rates;
}

void ArmSimulation::step() {
    if (!finite()) throw std::runtime_error("Nonfinite arm state before step");
    const double dt = model_->opt.timestep;
    if (!std::isfinite(dt) || dt <= 0 || dt > 0.1) throw std::runtime_error("Arm timestep must be in (0, 0.1] seconds");
    mj_forward(model_.get(), data_.get());
    auto rates = mode_ == ArmControlMode::Cartesian ? cartesian_rates() : velocity_command_;
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        const int qadr = model_->jnt_qposadr[joints_[i]], vadr = model_->jnt_dofadr[joints_[i]];
        const double q = data_->qpos[qadr];
        double rate = mode_ == ArmControlMode::Position ?
            std::min(position_gain, 1/dt)*(position_command_[i]-q) : rates[i];
        rate = std::clamp(rate, -velocity_limits_[i], velocity_limits_[i]);
        const double next = std::clamp(q + dt*rate, limits_[i][0], limits_[i][1]);
        data_->qpos[qadr] = next;
        data_->qvel[vadr] = (next-q)/dt;
    }
    // Deliberately no mj_step: its torque/contact integration is outside this
    // mock. mj_forward supplies transforms and Jacobians for rendering/IK;
    // computed force/acceleration arrays are not physical command feedback.
    data_->time += dt;
    mj_forward(model_.get(), data_.get());
    if (!finite()) throw std::runtime_error("Nonfinite arm state after step");
}

std::vector<double> ArmSimulation::joint_positions() const {
    std::vector<double> result;
    for (int j : joints_) result.push_back(data_->qpos[model_->jnt_qposadr[j]]);
    return result;
}
std::vector<double> ArmSimulation::joint_velocities() const {
    std::vector<double> result;
    for (int j : joints_) result.push_back(data_->qvel[model_->jnt_dofadr[j]]);
    return result;
}
std::array<double, 3> ArmSimulation::tool_position() const {
    const auto* p = data_->site_xpos + 3*tool_;
    return {p[0], p[1], p[2]};
}
bool ArmSimulation::finite() const {
    for (int i = 0; i < model_->nq; ++i) if (!std::isfinite(data_->qpos[i])) return false;
    for (int i = 0; i < model_->nv; ++i) if (!std::isfinite(data_->qvel[i])) return false;
    for (int i = 0; i < 3; ++i) if (!std::isfinite(data_->site_xpos[3*tool_+i])) return false;
    for (int i = 0; i < 9; ++i) if (!std::isfinite(data_->site_xmat[9*tool_+i])) return false;
    return std::isfinite(time());
}
} // namespace tech_core
