#ifndef AXIS_FREE_ROTATE_IK_HPP
#define AXIS_FREE_ROTATE_IK_HPP

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace axis_free_rotate_ik
{


struct IKResult
{
    bool success;
    Eigen::VectorXd q; // Joint configuration solution
    std::string message; // Optional message for debugging or error information
};

struct PointSolverOptions
{
    std::optional<int> max_iterations;
    std::optional<double> position_tolerance;
    std::optional<double> axis_tolerance;
    std::optional<double> damping;
    std::optional<double> alpha;
    std::optional<Eigen::VectorXd> delta_max_q;
};

struct IKRequest
{
    // Desired end-effector pose in base link frame
    Eigen::Vector3d desired_position;

    // Desired end-effector orientation as a rotation matrix (3x3) in base link frame
    std::optional<Eigen::Matrix3d> desired_orientation;

    // Desired end-effector direction vector in base link frame
    std::optional<Eigen::Vector3d> desired_direction;

    // Local end-effector axis to align with desired_direction. Expressed in end-effector frame.
    std::optional<Eigen::Vector3d> desired_up_vector;

    // Optional initial guess for the chain joint configuration, ordered by joint_names().
    std::optional<Eigen::VectorXd> initial_guess;

    // The maximum delta q in each iteration of the IK solver, used to prevent large jumps in joint space
    std::optional<Eigen::VectorXd> delta_max_q;

    // Optional parameters for the IK solver. Fixed-orientation IK uses tolerance for both
    // position and orientation; axis-free IK uses it as the fallback when the split
    // tolerances below are not set.
    int max_iterations = 300;
    double tolerance = 1e-4;

    // Axis-free IK only: position error tolerance in meters.
    std::optional<double> position_tolerance;

    // Axis-free IK only: axis angular error tolerance in radians.
    std::optional<double> axis_tolerance;
    double damping = 1e-4;

    // the step size for the IK solver
    double alpha = 0.2;
};

struct IKTrajectoryPoint
{
    // Desired end-effector position in base link frame.
    Eigen::Vector3d desired_position;

    // Desired end-effector axis direction in base link frame.
    Eigen::Vector3d desired_direction;

    // Local end-effector axis to align with desired_direction.
    Eigen::Vector3d local_axis{Eigen::Vector3d::UnitZ()};
    std::optional<PointSolverOptions> solver_options;
};

struct IKTrajectoryRequest
{
    std::vector<IKTrajectoryPoint> points;

    // Optional initial guess for the first trajectory point, ordered by joint_names().
    std::optional<Eigen::VectorXd> initial_guess;

    // The maximum delta q in each IK iteration.
    std::optional<Eigen::VectorXd> delta_max_q;

    // Optional maximum tangent-space joint step between adjacent solved trajectory points.
    // Ordered by joint_names(), with size model.nv; checked with pinocchio::difference().
    std::optional<Eigen::VectorXd> max_joint_step_between_points;

    int max_iterations = 300;
    double tolerance = 1e-4;

    // Axis-free trajectory IK only: position error tolerance in meters.
    std::optional<double> position_tolerance;

    // Axis-free trajectory IK only: axis angular error tolerance in radians.
    std::optional<double> axis_tolerance;
    double damping = 1e-4;
    double alpha = 0.2;
};

struct IKTrajectoryPointResult
{
    bool success;
    size_t point_index;

    // Solved chain configuration, ordered by joint_names(). On IK failure this is the last
    // configuration returned by the point solver.
    Eigen::VectorXd q;

    // Final position norm error and axis angular error in radians. These are infinity when
    // no finite final configuration is available.
    double position_error;
    double axis_error;
    std::string message;
};

struct IKTrajectoryResult
{
    bool success;

    // Contains every completed point and, when solving starts then fails, the failed point.
    // Validation failures happen before solving and therefore leave this vector empty.
    std::vector<IKTrajectoryPointResult> points;

    // Set to the first invalid or failed point index; nullopt only when all points succeed.
    std::optional<size_t> first_failed_index;
    std::string message;
};

/**
 * @brief A class for performing inverse kinematics with an axis-free rotation constraint.
 */
class AxisFreeRotateIK
{

public:
    AxisFreeRotateIK(
        const std::string & robot_description,
        const std::string & base_link,
        const std::string & end_effector_link
        /* const collision_check */
        );

    IKResult solve(const IKRequest & request);

    IKTrajectoryResult solveTrajectory(const IKTrajectoryRequest & request);

    const std::vector<std::string> & joint_names() const;

private:
    struct SolverWorkspace
    {
        SolverWorkspace(Eigen::Index nq, Eigen::Index nv)
            : q(nq),
            delta_q(nv),
            q_next(nq),
            jacobian_world(6, nv),
            jacobian_base(6, nv),
            task_jacobian(6, nv)
        {
        }

        Eigen::VectorXd q;
        Eigen::VectorXd delta_q;
        Eigen::VectorXd q_next;

        Eigen::Matrix<double, 6, 1> task_error;
        Eigen::Matrix<double, 6, 1> task_step;

        Eigen::MatrixXd jacobian_world;
        Eigen::MatrixXd jacobian_base;
        Eigen::MatrixXd task_jacobian;

        Eigen::Matrix<double, 6, 6> normal_matrix;
    };

    struct InternalIKResult
    {
        bool success;
        Eigen::VectorXd q;
        double position_error;
        double axis_error;
        std::string message;
    };

    std::string base_link_;
    std::string end_effector_link_;
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex base_frame_id_{0};
    pinocchio::FrameIndex end_effector_frame_id_{0};

    std::vector<std::string> joints_ordered_list_;
    std::vector<Eigen::Index> limited_q_indices_;
    Eigen::VectorXd lower_limits_;
    Eigen::VectorXd upper_limits_;

    void cache_model_limits();

    void clamp_to_model_limits(Eigen::VectorXd & q) const;

    void compute_frame_jacobian_in_base(
        const Eigen::VectorXd & q,
        SolverWorkspace & ws);

    void solve_damped_least_squares(
        const Eigen::Ref<const Eigen::MatrixXd> & jacobian,
        const Eigen::Matrix<double, 6, 1> & error,
        double damping,
        SolverWorkspace & ws) const;

    IKResult check_request_validity(const IKRequest & request);

    IKResult solve_fix_ik(const IKRequest & request);

    IKResult solve_free_ik(const IKRequest & request);

    InternalIKResult solve_free_ik_internal(const IKRequest & request);


}; // class AxisFreeRotateIK


} // namespace axis_free_rotate_ik

#endif // AXIS_FREE_ROTATE_IK_HPP
