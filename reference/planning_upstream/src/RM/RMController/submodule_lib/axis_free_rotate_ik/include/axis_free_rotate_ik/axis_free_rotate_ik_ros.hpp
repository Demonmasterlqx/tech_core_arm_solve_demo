#ifndef AXIS_FREE_ROTATE_IK_ROS_HPP
#define AXIS_FREE_ROTATE_IK_ROS_HPP

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "axis_free_rotate_ik/axis_free_rotate_ik.hpp"
#include "axis_free_rotate_ik/axis_free_rotate_ik_ros_parameters.hpp"
#include "axis_free_rotate_ik/srv/solve_axis_free_ik.hpp"
#include "axis_free_rotate_ik/srv/solve_fixed_ik.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace axis_free_rotate_ik
{

class AxisFreeRotateIKRos : public rclcpp::Node
{
public:
    explicit AxisFreeRotateIKRos(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    using SolveFixedIK = axis_free_rotate_ik::srv::SolveFixedIK;
    using SolveAxisFreeIK = axis_free_rotate_ik::srv::SolveAxisFreeIK;

    void load_parameters();
    std::string wait_for_robot_description();
    void initialize_solver(const std::string & robot_description);
    void create_publishers();
    void create_services();

    void handle_fixed_ik(
        const std::shared_ptr<SolveFixedIK::Request> request,
        std::shared_ptr<SolveFixedIK::Response> response);
    void handle_axis_free_ik(
        const std::shared_ptr<SolveAxisFreeIK::Request> request,
        std::shared_ptr<SolveAxisFreeIK::Response> response);

    bool transform_pose_to_base(
        const geometry_msgs::msg::PoseStamped & input,
        const double tf_timeout,
        geometry_msgs::msg::PoseStamped & output,
        std::string & error_message) const;

    std::optional<Eigen::VectorXd> vector_to_optional_q(
        const std::vector<double> & values,
        const std::string & field_name,
        std::string & error_message) const;

    bool apply_common_request_fields(
        const std::vector<double> & initial_guess,
        const std::vector<double> & delta_max_q,
        const int32_t max_iterations,
        const double tolerance,
        const double damping,
        const double alpha,
        IKRequest & ik_request,
        std::string & error_message) const;

    bool apply_common_trajectory_request_fields(
        const std::vector<double> & initial_guess,
        const std::vector<double> & delta_max_q,
        const int32_t max_iterations,
        const double tolerance,
        const double damping,
        const double alpha,
        IKTrajectoryRequest & trajectory_request,
        std::string & error_message) const;

    trajectory_msgs::msg::JointTrajectory build_trajectory(
        const std::vector<Eigen::VectorXd> & points,
        double point_time_interval) const;
    void publish_trajectory_if_requested(
        bool publish_requested,
        const trajectory_msgs::msg::JointTrajectory & trajectory) const;

    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    std::string robot_description_topic_;
    double robot_description_timeout_{2.0};
    std::string base_link_;
    std::string end_effector_link_;
    std::string joint_trajectory_topic_;

    std::unique_ptr<AxisFreeRotateIK> solver_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Service<SolveFixedIK>::SharedPtr fixed_ik_service_;
    rclcpp::Service<SolveAxisFreeIK>::SharedPtr axis_free_ik_service_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
};

}  // namespace axis_free_rotate_ik

#endif  // AXIS_FREE_ROTATE_IK_ROS_HPP
