#pragma once

#include <mujoco/mujoco.h>
#include <string>
#include <vector>

namespace tech_core {
// Robot metadata travels with MJCF, including models loaded outside this repo.
// SI units; joint order is explicit and independent of MuJoCo's storage order.
struct RobotModel {
    std::string id, root_body, tool_site;
    std::vector<std::string> joint_names;
    std::vector<int> joints, recovery_joint_order;
    std::vector<double> velocity_limits, planning_home;
    int posture_joint = -1; // Optional redundancy preference; -1 disables it.
    bool demo_start_at_preset = false;
    static RobotModel read(const mjModel* model);
};
std::string robot_text(const mjModel* model, const std::string& name, bool required = true);
std::vector<std::string> robot_words(const mjModel* model, const std::string& name, bool required = true);
std::vector<double> robot_numbers(const mjModel* model, const std::string& name, bool required = true);
// Built-in id or an external model directory. External packages need no rebuild.
std::string robot_model_path(const std::string& robot = "rm26", bool planning = false);
std::vector<std::string> available_robots();
} // namespace tech_core
