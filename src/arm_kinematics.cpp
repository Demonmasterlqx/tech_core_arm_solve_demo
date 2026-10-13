#include "tech_core/arm_kinematics.hpp"
#include <stdexcept>

namespace tech_core {
ArmKinematics::ArmKinematics(const std::string& path) {
    char error[2048]{};
    model_.reset(mj_loadXML(path.c_str(), nullptr, error, sizeof(error)));
    if (!model_) throw std::runtime_error(error);
    robot_ = RobotModel::read(model_.get());
    data_.reset(mj_makeData(model_.get()));
    if (!data_) throw std::runtime_error("Cannot allocate kinematics data");
    const int home = mj_name2id(model_.get(), mjOBJ_KEY, "home");
    if (home < 0) throw std::invalid_argument("Missing home keyframe");
    mj_resetDataKeyframe(model_.get(), data_.get(), home);
    tool_ = mj_name2id(model_.get(), mjOBJ_SITE, robot_.tool_site.c_str());
    const int n = robot_.joints.size();
    limits_ = planning::JointLimits(n);
    home_.resize(n);
    for (int i = 0; i < n; ++i) {
        const int j = robot_.joints[i];
        limits_.lower[i] = model_->jnt_range[2*j];
        limits_.upper[i] = model_->jnt_range[2*j+1];
        limits_.angular[i] = model_->jnt_type[j] == mjJNT_HINGE;
        home_[i] = data_->qpos[model_->jnt_qposadr[j]];
    }
    if (!home_.allFinite() || (home_.array()<limits_.lower.array()).any() ||
        (home_.array()>limits_.upper.array()).any()) throw std::invalid_argument("Invalid home keyframe");
    translation_.resize(3*model_->nv); rotation_.resize(3*model_->nv);
}
void ArmKinematics::forward(const planning::JointVector& q, Eigen::Vector3d& p,
                            Eigen::Matrix3d& r, planning::Jacobian* jacobian) {
    if (q.size() != static_cast<int>(robot_.joints.size()) || !q.allFinite())
        throw std::invalid_argument("Invalid kinematics joint vector");
    for (int i = 0; i < q.size(); ++i) data_->qpos[model_->jnt_qposadr[robot_.joints[i]]] = q[i];
    mj_kinematics(model_.get(), data_.get());
    p = Eigen::Map<const Eigen::Vector3d>(data_->site_xpos+3*tool_);
    r = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(data_->site_xmat+9*tool_);
    if (jacobian) {
        mj_comPos(model_.get(), data_.get());
        mj_jacSite(model_.get(), data_.get(), translation_.data(), rotation_.data(), tool_);
        jacobian->resize(6, q.size());
        for (int i = 0; i < q.size(); ++i) for (int row = 0; row < 3; ++row) {
            const int dof = model_->jnt_dofadr[robot_.joints[i]];
            (*jacobian)(row,i) = translation_[row*model_->nv+dof];
            (*jacobian)(row+3,i) = rotation_[row*model_->nv+dof];
        }
    }
}
planning::IKResult ArmKinematics::inverse(const planning::TaskTarget& target,
                                         const planning::JointVector& seed,
                                         const planning::IKOptions& options) {
    return planning::solve_ik([&](const auto& q, auto& p, auto& r, auto* j){ forward(q,p,r,j); },
                              target, seed, limits_, options);
}
} // namespace tech_core
