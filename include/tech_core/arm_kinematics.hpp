#pragma once

#include "tech_core/planning_ik.hpp"
#include "tech_core/robot_model.hpp"
#include <memory>

namespace tech_core {
// Standalone FK/IK, with no assembly scene or payload requirement.
// Owns its MuJoCo workspace; use a separate instance per worker.
class ArmKinematics {
public:
    explicit ArmKinematics(const std::string& model_path = robot_model_path());
    void forward(const planning::JointVector& q, Eigen::Vector3d& position,
                 Eigen::Matrix3d& rotation, planning::Jacobian* jacobian = nullptr);
    planning::IKResult inverse(const planning::TaskTarget& target,
                               const planning::JointVector& seed,
                               const planning::IKOptions& options = {});
    const RobotModel& robot() const { return robot_; }
    const planning::JointLimits& limits() const { return limits_; }
    const planning::JointVector& home() const { return home_; }
    mjModel* model() { return model_.get(); }
    mjData* data() { return data_.get(); }
private:
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr, mj_deleteData};
    RobotModel robot_;
    planning::JointLimits limits_{0};
    planning::JointVector home_;
    int tool_ = -1;
    std::vector<mjtNum> translation_, rotation_;
};
} // namespace tech_core
