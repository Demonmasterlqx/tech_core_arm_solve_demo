#pragma once

#include "tech_core/arm_simulation.hpp"
#include <string>

namespace tech_core {
struct ArmViewOptions {
    double speed = 0.10;  // End-effector translation speed, metres/second.
    bool overlay = true;
};

void run_arm_viewer(ArmSimulation& sim, double duration = 0,
                    const std::string& screenshot = "",
                    const ArmViewOptions& options = {});
void save_arm_snapshot(ArmSimulation& sim, const std::string& path,
                       const ArmViewOptions& options = {});
}  // namespace tech_core
