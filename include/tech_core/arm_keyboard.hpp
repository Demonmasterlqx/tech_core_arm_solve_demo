#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

namespace tech_core {
struct ArmKeyboardInput {
    bool forward = false, backward = false;
    bool left = false, right = false;
    bool up = false, down = false;
};

// World coordinates: forward +X, left +Y, up +Z. Diagonals retain the
// requested speed; opposing keys cancel independently on each axis.
inline std::array<double, 3> arm_keyboard_velocity(const ArmKeyboardInput& keys,
                                                    double speed) {
    if (!std::isfinite(speed) || speed < 0)
        throw std::invalid_argument("Keyboard speed must be finite and nonnegative");
    std::array<double, 3> velocity{
        double(keys.forward) - double(keys.backward),
        double(keys.left) - double(keys.right),
        double(keys.up) - double(keys.down)};
    const double norm = std::sqrt(velocity[0] * velocity[0] +
                                  velocity[1] * velocity[1] +
                                  velocity[2] * velocity[2]);
    if (norm > 0)
        for (double& component : velocity) component *= speed / norm;
    return velocity;
}
}  // namespace tech_core
