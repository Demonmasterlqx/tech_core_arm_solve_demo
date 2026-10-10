#pragma once

#include "tech_core/rules.hpp"
#include <mujoco/mujoco.h>
#include <array>
#include <memory>
#include <string>

namespace tech_core {
constexpr double pi = 3.14159265358979323846;
constexpr double radians(double degrees) { return degrees * pi / 180.0; }
constexpr double degrees(double angle) { return angle * 180.0 / pi; }

// Manual OXYZ coordinates, SI units; theta/phi/alpha are degrees at this API.
struct Pose {
    double x = -0.09, y = 0.2, z = 0.6;
    double theta = 0, phi = 90, alpha = 0;
};
std::array<double, 4> pose_quaternion(const Pose& pose);
void validate_pose(const Pose& pose, int level);

struct SimulationConfig {
    int level = 3;
    std::array<double, 2> target_degrees = {35, -40};
    Pose pose;
    // The second station has the same x/z and a shifted y. Its default yaw
    // separates the wide panel proxies without changing the prescribed E gap.
    std::array<double, 3> secondary_angles = {30, 90, 0};
    double pair_spacing = 0.4;
    std::string model_path = TECH_CORE_MODEL_PATH;
};
// Controls represent a virtual gripper acting on an already aligned energy
// unit. The guided joints deliberately exclude free grasping and arm IK.
struct Command {
    double insertion = 0, lift = 0, flip = 0, twist = 0;
};

class Simulation {
public:
    explicit Simulation(SimulationConfig config = {});
    void reset();
    void step();
    void drive_demo();
    void set_command(std::size_t core, Command command);
    Command command(std::size_t core) const;
    Observation observation(std::size_t core) const;
    bool confirm(std::size_t core);
    void set_card_present(bool present) { card_present_ = present; }
    void set_robot_alive(bool alive) { robot_alive_ = alive; }
    bool card_present() const { return card_present_; }
    void set_gripper_enabled(bool enabled) { gripper_enabled_ = enabled; }
    bool gripper_enabled() const { return gripper_enabled_; }
    mjModel* model() { return model_.get(); }
    mjData* data() { return data_.get(); }
    const mjModel* model() const { return model_.get(); }
    const mjData* data() const { return data_.get(); }
    const AssemblyRules& rules() const { return *rules_; }
    const SimulationConfig& config() const { return config_; }
    double time() const { return data_->time; }
    double return_torque(std::size_t core) const { return return_torque_.at(core); }
    double holding_torque(std::size_t core) const { return holding_torque_.at(core); }
    bool finite() const;

private:
    void update_station();
    void validate_geometry();
    void update_lights();
    int lookup(mjtObj type, const std::string& name) const;
    SimulationConfig config_;
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr, mj_deleteData};
    std::unique_ptr<AssemblyRules> rules_;
    // Ordered insertion, lift, flip, twist.
    std::array<std::array<int, 4>, 2> joints_{}, actuators_{};
    std::array<int, 2> mocap_{};
    std::array<Command, 2> commands_{};
    std::array<double, 2> return_torque_{}, holding_torque_{}, lock_angle_{};
    std::array<bool, 2> lock_active_{};
    bool card_present_ = true, robot_alive_ = true, gripper_enabled_ = true;
};
} // namespace tech_core
