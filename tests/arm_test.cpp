#include "tech_core/arm_simulation.hpp"
#include "tech_core/arm_keyboard.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using namespace tech_core;
void require(bool value, const std::string& message) {
    if (!value) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string& message) {
    require(std::isfinite(actual) && std::abs(actual-expected) <= tolerance,
            message + ": expected " + std::to_string(expected) + ", got " + std::to_string(actual));
}
template<class F> void invalid(F action, const std::string& message) {
    bool rejected = false;
    try { action(); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, message);
}
void advance(ArmSimulation& sim, double seconds) {
    const int steps = static_cast<int>(std::ceil(seconds/sim.model()->opt.timestep));
    for (int step = 0; step < steps; ++step) {
        const auto previous = sim.joint_positions();
        sim.step();
        require(sim.finite(), "All mock state and FK must be finite");
        for (int warning = 0; warning < mjNWARNING; ++warning)
            require(sim.data()->warning[warning].number == 0, "No MuJoCo warnings during arm motion");
        const auto q = sim.joint_positions(), v = sim.joint_velocities();
        for (std::size_t j = 0; j < q.size(); ++j) {
            require(q[j] >= sim.joint_limits()[j][0]-1e-12 && q[j] <= sim.joint_limits()[j][1]+1e-12,
                    "Position must respect source hard stops");
            require(std::abs(v[j]) <= sim.velocity_limits()[j]+1e-10, "Velocity must respect mock rate limit");
            near(q[j]-previous[j], v[j]*sim.model()->opt.timestep, 1e-12, "Reported velocity must match actual displacement");
        }
    }
}

void source_fk_and_structure(ArmSimulation& sim) {
    require(sim.joint_names() == std::vector<std::string>{"j0", "j1", "j2", "j3", "j4",
                "end_differential_controller/j5", "end_differential_controller/j6"}, "Original seven source joints in source order");
    require(sim.model()->nq == 7 && sim.model()->nv == 7, "Fixed arm must have exactly seven scalar DOFs");
    for (int j = 0; j < sim.model()->njnt; ++j)
        require(sim.model()->jnt_type[j] == mjJNT_HINGE, "No free base or chassis joint");
    const std::array<double, 7> home{0, 1.0, 1.4, 0, 0.7, 0, 0};
    const auto q = sim.joint_positions();
    for (int j = 0; j < 7; ++j) near(q[j], home[j], 1e-12, "Source-chain nonsingular home");
    // Independent URDF homogeneous-transform calculation, not derived from
    // the runtime site's own transforms or the controller's Jacobian.
    const std::array<double, 3> expected{0.151340465, 0.0627496563, 0.887989955};
    const auto p = sim.tool_position();
    for (int k = 0; k < 3; ++k) near(p[k], expected[k], 2e-8, "Home FK matches original URDF ee_link");
    near(sim.time(), 0, 0, "Reset clock");
    for (int j = 0; j < 7; ++j) sim.data()->qpos[sim.model()->jnt_qposadr[j]] = 0;
    mj_forward(sim.model(), sim.data());
    const std::array<double, 3> zero_fk{-0.08933873257293728, 0.06274909100958441, 0.3362941445247948};
    for (int k = 0; k < 3; ++k) near(sim.tool_position()[k], zero_fk[k], 2e-10, "Zero FK matches original URDF ee_link");
    sim.reset();
}

void joint_control(ArmSimulation& sim) {
    auto target = sim.joint_positions();
    for (std::size_t j = 0; j < target.size(); ++j) target[j] += j%2 ? -0.12 : 0.12;
    const auto before = sim.joint_positions();
    sim.set_joint_positions(target);
    require(sim.mode() == ArmControlMode::Position, "Position control mode");
    require(sim.joint_positions() == before, "Position command is a target, not an instantaneous teleport");
    advance(sim, 2.0);
    for (std::size_t j = 0; j < target.size(); ++j)
        near(sim.joint_positions()[j], target[j], 1e-6, "Position target reached");
    sim.reset();
    std::vector<double> velocity(7);
    for (int j = 0; j < 7; ++j) velocity[j] = j%2 ? -0.4 : 0.4;
    const auto start = sim.joint_positions();
    sim.set_joint_velocities(velocity);
    require(sim.mode() == ArmControlMode::Velocity, "Velocity control mode");
    advance(sim, 0.2);
    for (int j = 0; j < 7; ++j)
        near(sim.joint_positions()[j], start[j]+velocity[j]*sim.time(), 1e-11, "Velocity command integrates in radians/s");
    sim.stop();
    const auto held = sim.joint_positions();
    for (double v : sim.joint_velocities()) near(v, 0, 0, "Stop zeros velocity immediately");
    advance(sim, 0.1);
    require(sim.joint_positions() == held, "Stop preserves all joint positions without drift");
    sim.reset();
    source_fk_and_structure(sim);
}

void limits(ArmSimulation& sim) {
    for (int sign : {-1, 1}) {
        sim.reset();
        // Begin just inside each source stop, then request an extreme speed.
        // This exercises overshoot prevention and the ability to move away
        // from a stop, without relying on a hardware torque model.
        for (int j = 0; j < 7; ++j) {
            const double bound = sim.joint_limits()[j][sign > 0 ? 1 : 0];
            sim.data()->qpos[sim.model()->jnt_qposadr[j]] = bound-sign*0.005;
        }
        mj_forward(sim.model(), sim.data());
        sim.set_joint_velocities(std::vector<double>(7, sign*1e300));
        advance(sim, 0.02);
        for (int j = 0; j < 7; ++j) {
            near(sim.joint_positions()[j], sim.joint_limits()[j][sign > 0 ? 1 : 0], 1e-12, "Exact hard stop");
            near(sim.joint_velocities()[j], 0, 1e-12, "No outward velocity at hard stop");
        }
        sim.set_joint_velocities(std::vector<double>(7, -sign*0.2));
        advance(sim, 0.02);
        for (int j = 0; j < 7; ++j)
            require(sign*(sim.joint_positions()[j]-sim.joint_limits()[j][sign > 0 ? 1 : 0]) < -0.003,
                    "Joint must be able to leave hard stop");
        sim.set_joint_positions(std::vector<double>(7, sign*1e300));
        advance(sim, 1.5);
        for (int j = 0; j < 7; ++j)
            near(sim.joint_positions()[j], sim.joint_limits()[j][sign > 0 ? 1 : 0], 1e-6, "Extreme position target clamps to source range");
    }
    sim.reset();
}

void cartesian_keyboard(ArmSimulation& sim) {
    const int site = mj_name2id(sim.model(), mjOBJ_SITE, "arm_tool");
    for (int axis = 0; axis < 3; ++axis) for (int sign : {-1, 1}) {
        sim.reset();
        ArmKeyboardInput keys;
        if (axis == 0) { keys.forward = sign > 0; keys.backward = sign < 0; }
        if (axis == 1) { keys.left = sign > 0; keys.right = sign < 0; }
        if (axis == 2) { keys.up = sign > 0; keys.down = sign < 0; }
        const auto command = arm_keyboard_velocity(keys, 0.025);
        for (int k = 0; k < 3; ++k) near(command[k], k == axis ? sign*0.025 : 0, 1e-12, "World-axis keyboard mapping");
        const auto start = sim.tool_position();
        std::array<double, 9> orientation;
        std::copy_n(sim.data()->site_xmat+9*site, 9, orientation.begin());
        sim.set_cartesian_velocity(command);
        require(sim.mode() == ArmControlMode::Cartesian, "Cartesian control mode");
        advance(sim, 0.4);
        for (int k = 0; k < 3; ++k)
            near(sim.tool_position()[k]-start[k], command[k]*sim.time(), 0.0003, "Actual FK follows keyboard world translation");
        double trace = 0;
        for (int k = 0; k < 9; ++k) trace += orientation[k]*sim.data()->site_xmat[9*site+k];
        const double angle = std::acos(std::clamp((trace-1)/2, -1.0, 1.0));
        require(angle < 0.005, "Cartesian translation should approximately preserve tool orientation");
        sim.stop();
        const auto stopped = sim.tool_position();
        advance(sim, 0.1);
        for (int k = 0; k < 3; ++k) near(sim.tool_position()[k], stopped[k], 1e-12, "Key release must stop immediately");
    }
    ArmKeyboardInput opposite;
    opposite.forward = opposite.backward = opposite.left = opposite.right = opposite.up = opposite.down = true;
    require(arm_keyboard_velocity(opposite, 0.04) == std::array<double, 3>{0, 0, 0}, "Opposite keys cancel");
    ArmKeyboardInput diagonal;
    diagonal.forward = diagonal.left = diagonal.up = true;
    const auto vector = arm_keyboard_velocity(diagonal, 0.04);
    near(std::hypot(vector[0], vector[1], vector[2]), 0.04, 1e-12, "Diagonal keyboard motion has bounded speed");
    sim.reset();
    const auto start = sim.tool_position();
    sim.set_cartesian_velocity({1e300, 0, 0});
    advance(sim, 0.02);
    const auto end = sim.tool_position();
    const double distance = std::hypot(end[0]-start[0], end[1]-start[1], end[2]-start[2]);
    require(distance > 0 && distance <= 0.101*sim.time(), "Huge finite Cartesian input is safely speed limited");
    sim.reset();
    // The original straight configuration is near singular. Damping/rate
    // limiting must remain finite even if the requested direction is blocked.
    for (int j = 0; j < 7; ++j) sim.data()->qpos[sim.model()->jnt_qposadr[j]] = 0;
    mj_forward(sim.model(), sim.data());
    sim.set_cartesian_velocity({0.05, 0.05, 0.05});
    advance(sim, 0.2);
    sim.reset();
}

void invalid_inputs(ArmSimulation& sim) {
    invalid([&] { sim.set_joint_positions({}); }, "Empty position vector rejected");
    invalid([&] { sim.set_joint_velocities(std::vector<double>(8)); }, "Wrong velocity dimension rejected");
    for (double bad : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                       -std::numeric_limits<double>::infinity()}) {
        for (int j = 0; j < 7; ++j) {
            auto input = sim.joint_positions(); input[j] = bad;
            invalid([&] { sim.set_joint_positions(input); }, "Nonfinite position rejected");
            invalid([&] { sim.set_joint_velocities(input); }, "Nonfinite velocity rejected");
        }
        for (int axis = 0; axis < 3; ++axis) {
            std::array<double, 3> input{}; input[axis] = bad;
            invalid([&] { sim.set_cartesian_velocity(input); }, "Nonfinite Cartesian command rejected");
        }
    }
    const auto held = sim.joint_positions();
    advance(sim, 0.05);
    require(sim.joint_positions() == held, "Rejected input must leave previous hold command intact");
    sim.data()->qpos[0] = std::numeric_limits<double>::quiet_NaN();
    require(!sim.finite(), "External nonfinite state is detected");
    bool rejected = false;
    try { sim.step(); } catch (const std::runtime_error&) { rejected = true; }
    require(rejected, "Corrupt state rejected before MuJoCo updates");
    sim.reset();
    source_fk_and_structure(sim);
}
}

int main() {
    try {
        ArmSimulation sim;
        source_fk_and_structure(sim);
        joint_control(sim);
        limits(sim);
        cartesian_keyboard(sim);
        invalid_inputs(sim);
        std::cout << "Arm contracts passed: source FK, seven fixed-base joints, position/velocity control, stops, keyboard XYZ, orientation, invalid input\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Arm contract failure: " << error.what() << '\n';
        return 1;
    }
}
