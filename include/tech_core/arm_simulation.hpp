#pragma once

#include <mujoco/mujoco.h>
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace tech_core {
enum class ArmControlMode { Position, Velocity, Cartesian };

// Fixed-base kinematic mock: commands integrate qpos/qvel directly, followed
// by MuJoCo FK. Neither motor torques nor collision reactions drive motion.
// Angles/rates use radians; Cartesian velocities are world-frame metres/s.
class ArmSimulation {
public:
    static constexpr double maximum_joint_speed = 1.0; // Mock limit, not hardware data.
    static constexpr double maximum_cartesian_speed = 0.10;
    explicit ArmSimulation(std::string model_path = TECH_CORE_ARM_MODEL_PATH);
    void reset();
    void step();
    void set_joint_positions(const std::vector<double>& positions);
    void set_joint_velocities(const std::vector<double>& velocities);
    void set_cartesian_velocity(const std::array<double, 3>& world_metres_per_sec);
    void stop();
    std::vector<double> joint_positions() const;
    std::vector<double> joint_velocities() const;
    const std::vector<std::string>& joint_names() const { return names_; }
    const std::vector<std::array<double, 2>>& joint_limits() const { return limits_; }
    const std::vector<double>& velocity_limits() const { return velocity_limits_; }
    std::array<double, 3> tool_position() const;
    ArmControlMode mode() const { return mode_; }
    double time() const { return data_->time; }
    bool finite() const;
    mjModel* model() { return model_.get(); }
    mjData* data() { return data_.get(); }
    const mjModel* model() const { return model_.get(); }
    const mjData* data() const { return data_.get(); }

private:
    void validate_command(const std::vector<double>& values) const;
    std::vector<double> cartesian_rates() const;
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr, mj_deleteData};
    std::vector<int> joints_;
    std::vector<std::string> names_;
    std::vector<std::array<double, 2>> limits_;
    std::vector<double> velocity_limits_, position_command_, velocity_command_;
    std::array<double, 3> cartesian_command_{};
    std::array<double, 9> orientation_reference_{};
    int tool_ = -1, home_ = -1;
    ArmControlMode mode_ = ArmControlMode::Position;
};
} // namespace tech_core
