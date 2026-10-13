#include "tech_core/arm_kinematics.hpp"
#include <iostream>

int main(int argc, char** argv) {
    try {
        tech_core::ArmKinematics arm(tech_core::robot_model_path(argc > 1 ? argv[1] : "rm26"));
        Eigen::Vector3d position;
        Eigen::Matrix3d rotation;
        tech_core::planning::Jacobian jacobian;
        arm.forward(arm.home(), position, rotation, &jacobian);
        std::cout << arm.robot().id << " joints=" << jacobian.cols() << " TCP=" << position.transpose() << '\n';
        tech_core::planning::TaskTarget target;
        target.position = position; target.direction = rotation.col(2);
        auto seed = arm.home(); seed[0] += .03;
        const auto solved = arm.inverse(target, seed);
        std::cout << solved.reason << " q=" << solved.q.transpose() << '\n';
        return solved.success ? 0 : 2;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
