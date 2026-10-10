#include "axis_free_rotate_ik/axis_free_rotate_ik.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

namespace
{

constexpr double kPi = 3.14159265358979323846;

const std::vector<std::string> & chain_joint_names()
{
    static const std::vector<std::string> names = {
        "j1", "j2", "j3", "j4", "j5", "j6", "j7"};
    return names;
}

const std::string & test_urdf()
{
    static const std::string urdf =
        R"(
<robot name="axis_free_rotate_ik_test">
  <link name="world"/>
  <joint name="world_to_base" type="fixed">
    <origin xyz="0.1 -0.2 0.3" rpy="0.1 -0.2 0.3"/>
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
  <link name="base_link"/>

  <link name="side_link"/>
  <joint name="side_joint" type="revolute">
    <origin xyz="0.0 0.25 0.0" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="side_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="10"/>
  </joint>

  <link name="link1"/>
  <joint name="j1" type="revolute">
    <origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.141592653589793" upper="3.141592653589793" effort="10" velocity="10"/>
  </joint>

  <link name="link2"/>
  <joint name="j2" type="revolute">
    <origin xyz="0.18 0.0 0.02" rpy="0 0 0"/>
    <parent link="link1"/>
    <child link="link2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.141592653589793" upper="3.141592653589793" effort="10" velocity="10"/>
  </joint>

  <link name="link3"/>
  <joint name="j3" type="revolute">
    <origin xyz="0.16 0.0 0.03" rpy="0 0 0"/>
    <parent link="link2"/>
    <child link="link3"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.141592653589793" upper="3.141592653589793" effort="10" velocity="10"/>
  </joint>

  <link name="link4"/>
  <joint name="j4" type="revolute">
    <origin xyz="0.14 0.03 0.0" rpy="0 0 0"/>
    <parent link="link3"/>
    <child link="link4"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.141592653589793" upper="3.141592653589793" effort="10" velocity="10"/>
  </joint>

  <link name="link5"/>
  <joint name="j5" type="revolute">
    <origin xyz="0.12 0.0 0.04" rpy="0 0 0"/>
    <parent link="link4"/>
    <child link="link5"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.141592653589793" upper="3.141592653589793" effort="10" velocity="10"/>
  </joint>

  <link name="link6"/>
  <joint name="j6" type="revolute">
    <origin xyz="0.10 -0.02 0.0" rpy="0 0 0"/>
    <parent link="link5"/>
    <child link="link6"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.141592653589793" upper="3.141592653589793" effort="10" velocity="10"/>
  </joint>

  <link name="link7"/>
  <joint name="j7" type="revolute">
    <origin xyz="0.08 0.0 0.02" rpy="0 0 0"/>
    <parent link="link6"/>
    <child link="link7"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.141592653589793" upper="3.141592653589793" effort="10" velocity="10"/>
  </joint>

  <link name="ee_link"/>
  <joint name="ee_fixed" type="fixed">
    <origin xyz="0.09 0.01 0.0" rpy="0.0 0.2 0.0"/>
    <parent link="link7"/>
    <child link="ee_link"/>
  </joint>
</robot>
)";
    return urdf;
}

pinocchio::Model build_full_model()
{
    pinocchio::Model model;
    pinocchio::urdf::buildModelFromXML(test_urdf(), model);
    return model;
}

pinocchio::Model build_reduced_chain_model()
{
    const pinocchio::Model full_model = build_full_model();
    const std::vector<pinocchio::JointIndex> joints_to_lock = {
        full_model.getJointId("side_joint")};
    return pinocchio::buildReducedModel(
        full_model,
        joints_to_lock,
        pinocchio::neutral(full_model));
}

Eigen::VectorXd full_configuration_from_chain(
    const pinocchio::Model & model,
    const Eigen::VectorXd & chain_configuration)
{
    Eigen::VectorXd q = pinocchio::neutral(model);
    const auto & names = chain_joint_names();
    for (size_t i = 0; i < names.size(); ++i) {
        const pinocchio::JointIndex joint_id = model.getJointId(names[i]);
        q(model.joints[joint_id].idx_q()) = chain_configuration(static_cast<Eigen::Index>(i));
    }
    return q;
}

pinocchio::SE3 base_to_end_effector(const Eigen::VectorXd & chain_configuration)
{
    pinocchio::Model model = build_full_model();
    pinocchio::Data data(model);
    const Eigen::VectorXd q = full_configuration_from_chain(model, chain_configuration);

    pinocchio::framesForwardKinematics(model, data, q);
    const pinocchio::FrameIndex base_frame_id = model.getFrameId("base_link");
    const pinocchio::FrameIndex ee_frame_id = model.getFrameId("ee_link");
    return data.oMf[base_frame_id].inverse() * data.oMf[ee_frame_id];
}

double axis_angle(const Eigen::Vector3d & lhs, const Eigen::Vector3d & rhs)
{
    const double dot = std::clamp(lhs.normalized().dot(rhs.normalized()), -1.0, 1.0);
    return std::acos(dot);
}

axis_free_rotate_ik::IKRequest make_default_request()
{
    axis_free_rotate_ik::IKRequest request;
    request.max_iterations = 200;
    request.tolerance = 1e-6;
    request.damping = 1e-5;
    request.alpha = 0.8;
    request.delta_max_q = Eigen::VectorXd::Constant(7, 0.08);
    return request;
}

axis_free_rotate_ik::IKTrajectoryRequest make_default_trajectory_request()
{
    axis_free_rotate_ik::IKTrajectoryRequest request;
    request.max_iterations = 200;
    request.tolerance = 1e-6;
    request.damping = 1e-5;
    request.alpha = 0.8;
    request.delta_max_q = Eigen::VectorXd::Constant(7, 0.08);
    return request;
}

axis_free_rotate_ik::IKTrajectoryPoint trajectory_point_from_q(
    const Eigen::VectorXd & q,
    const Eigen::Vector3d & local_axis = Eigen::Vector3d::UnitZ())
{
    const pinocchio::SE3 pose = base_to_end_effector(q);
    axis_free_rotate_ik::IKTrajectoryPoint point;
    point.desired_position = pose.translation();
    point.desired_direction = pose.rotation() * local_axis;
    point.local_axis = local_axis;
    return point;
}

} // namespace

TEST(AxisFreeRotateIKTest, AcceptsXmlAndFilePathInputs)
{
    axis_free_rotate_ik::AxisFreeRotateIK xml_solver(test_urdf(), "base_link", "ee_link");
    EXPECT_EQ(xml_solver.joint_names(), chain_joint_names());

    const std::filesystem::path urdf_path =
        std::filesystem::temp_directory_path() / "axis_free_rotate_ik_test.urdf";
    {
        std::ofstream output(urdf_path);
        ASSERT_TRUE(output.good());
        output << test_urdf();
    }

    axis_free_rotate_ik::AxisFreeRotateIK file_solver(
        urdf_path.string(), "base_link", "ee_link");
    EXPECT_EQ(file_solver.joint_names(), chain_joint_names());

    std::filesystem::remove(urdf_path);
}

TEST(AxisFreeRotateIKTest, RejectsInvalidFramesAndDisconnectedBase)
{
    EXPECT_THROW(
        axis_free_rotate_ik::AxisFreeRotateIK(test_urdf(), "missing_link", "ee_link"),
        std::runtime_error);
    EXPECT_THROW(
        axis_free_rotate_ik::AxisFreeRotateIK(test_urdf(), "base_link", "missing_link"),
        std::runtime_error);
    EXPECT_THROW(
        axis_free_rotate_ik::AxisFreeRotateIK(test_urdf(), "side_link", "ee_link"),
        std::runtime_error);
}

TEST(AxisFreeRotateIKTest, RejectsInvalidRequests)
{
    axis_free_rotate_ik::AxisFreeRotateIK solver(test_urdf(), "base_link", "ee_link");

    axis_free_rotate_ik::IKRequest request;
    request.desired_position = Eigen::Vector3d::Zero();
    EXPECT_FALSE(solver.solve(request).success);

    request.desired_orientation = Eigen::Matrix3d::Identity();
    request.initial_guess = Eigen::VectorXd::Zero(3);
    EXPECT_FALSE(solver.solve(request).success);

    request.initial_guess = Eigen::VectorXd::Zero(7);
    request.alpha = 0.0;
    EXPECT_FALSE(solver.solve(request).success);

    request.alpha = 0.5;
    request.desired_orientation = 2.0 * Eigen::Matrix3d::Identity();
    EXPECT_FALSE(solver.solve(request).success);

    axis_free_rotate_ik::IKRequest free_request = make_default_request();
    free_request.desired_position = Eigen::Vector3d::Zero();
    free_request.desired_direction = Eigen::Vector3d::UnitZ();
    free_request.desired_up_vector = Eigen::Vector3d::UnitZ();

    free_request.position_tolerance = 0.0;
    EXPECT_FALSE(solver.solve(free_request).success);

    free_request.position_tolerance.reset();
    free_request.axis_tolerance = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(solver.solve(free_request).success);
}

TEST(AxisFreeRotateIKTest, SolvesFixedPoseIkFromNearbySeed)
{
    axis_free_rotate_ik::AxisFreeRotateIK solver(test_urdf(), "base_link", "ee_link");

    Eigen::VectorXd target_q(7);
    target_q << 0.25, -0.35, 0.30, 0.20, -0.25, 0.15, 0.40;
    const pinocchio::SE3 target_pose = base_to_end_effector(target_q);

    axis_free_rotate_ik::IKRequest request = make_default_request();
    request.desired_position = target_pose.translation();
    request.desired_orientation = target_pose.rotation();

    Eigen::VectorXd initial_guess(7);
    initial_guess << 0.30, -0.39, 0.33, 0.17, -0.21, 0.12, 0.42;
    request.initial_guess = initial_guess;

    const auto result = solver.solve(request);
    ASSERT_TRUE(result.success) << result.message;

    const pinocchio::SE3 solved_pose = base_to_end_effector(result.q);
    EXPECT_LT((target_pose.translation() - solved_pose.translation()).norm(), 1e-5);
    EXPECT_LT(
        pinocchio::log3(target_pose.rotation() * solved_pose.rotation().transpose()).norm(),
        1e-5);
}

TEST(AxisFreeRotateIKTest, SolvesAxisFreeIkFromNearbySeed)
{
    axis_free_rotate_ik::AxisFreeRotateIK solver(test_urdf(), "base_link", "ee_link");

    Eigen::VectorXd target_q(7);
    target_q << 0.20, -0.40, 0.35, -0.20, 0.25, -0.10, 0.45;
    const pinocchio::SE3 target_pose = base_to_end_effector(target_q);
    const Eigen::Vector3d local_axis = Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d desired_axis = target_pose.rotation() * local_axis;

    axis_free_rotate_ik::IKRequest request = make_default_request();
    request.desired_position = target_pose.translation();
    request.desired_direction = desired_axis;
    request.desired_up_vector = local_axis;

    Eigen::VectorXd initial_guess(7);
    initial_guess << 0.25, -0.44, 0.38, -0.23, 0.29, -0.13, 0.48;
    request.initial_guess = initial_guess;

    const auto result = solver.solve(request);
    ASSERT_TRUE(result.success) << result.message;

    const pinocchio::SE3 solved_pose = base_to_end_effector(result.q);
    const Eigen::Vector3d solved_axis = solved_pose.rotation() * local_axis;

    EXPECT_LT((target_pose.translation() - solved_pose.translation()).norm(), 1e-5);
    EXPECT_LT(axis_angle(desired_axis, solved_axis), 1e-5);
}

TEST(AxisFreeRotateIKTest, UsesSplitPositionAndAxisTolerancesForAxisFreeIk)
{
    axis_free_rotate_ik::AxisFreeRotateIK solver(test_urdf(), "base_link", "ee_link");

    Eigen::VectorXd initial_guess(7);
    initial_guess << 0.20, -0.40, 0.35, -0.20, 0.25, -0.10, 0.45;
    const pinocchio::SE3 initial_pose = base_to_end_effector(initial_guess);
    const Eigen::Vector3d local_axis = Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d current_axis = initial_pose.rotation() * local_axis;

    Eigen::Vector3d rotation_axis = current_axis.cross(Eigen::Vector3d::UnitX());
    if (rotation_axis.norm() < 1e-6) {
        rotation_axis = current_axis.cross(Eigen::Vector3d::UnitY());
    }
    rotation_axis.normalize();

    axis_free_rotate_ik::IKRequest loose_axis_request = make_default_request();
    loose_axis_request.desired_position = initial_pose.translation();
    loose_axis_request.desired_direction =
        Eigen::AngleAxisd(5e-5, rotation_axis) * current_axis;
    loose_axis_request.desired_up_vector = local_axis;
    loose_axis_request.initial_guess = initial_guess;
    loose_axis_request.max_iterations = 1;
    loose_axis_request.tolerance = 1e-8;
    loose_axis_request.position_tolerance = 1e-8;
    loose_axis_request.axis_tolerance = 1e-4;
    EXPECT_TRUE(solver.solve(loose_axis_request).success);

    axis_free_rotate_ik::IKRequest loose_position_request = make_default_request();
    loose_position_request.desired_position =
        initial_pose.translation() + Eigen::Vector3d(5e-5, 0.0, 0.0);
    loose_position_request.desired_direction = current_axis;
    loose_position_request.desired_up_vector = local_axis;
    loose_position_request.initial_guess = initial_guess;
    loose_position_request.max_iterations = 1;
    loose_position_request.tolerance = 1e-8;
    loose_position_request.position_tolerance = 1e-4;
    loose_position_request.axis_tolerance = 1e-8;
    EXPECT_TRUE(solver.solve(loose_position_request).success);
}

TEST(AxisFreeRotateIKTest, RejectsInvalidTrajectoryRequests)
{
    axis_free_rotate_ik::AxisFreeRotateIK solver(test_urdf(), "base_link", "ee_link");

    axis_free_rotate_ik::IKTrajectoryRequest request = make_default_trajectory_request();
    const auto empty_result = solver.solveTrajectory(request);
    EXPECT_FALSE(empty_result.success);
    EXPECT_FALSE(empty_result.first_failed_index.has_value());

    Eigen::VectorXd target_q(7);
    target_q << 0.10, -0.15, 0.12, -0.08, 0.11, -0.05, 0.09;
    const auto valid_point = trajectory_point_from_q(target_q);

    auto expect_rejected_at = [&solver](
        const axis_free_rotate_ik::IKTrajectoryRequest & invalid_request,
        const size_t failed_index)
        {
            const auto result = solver.solveTrajectory(invalid_request);
            EXPECT_FALSE(result.success);
            ASSERT_TRUE(result.first_failed_index.has_value());
            EXPECT_EQ(*result.first_failed_index, failed_index);
            EXPECT_TRUE(result.points.empty());
        };

    request.points.push_back(valid_point);
    request.initial_guess = Eigen::VectorXd::Zero(3);
    expect_rejected_at(request, 0u);

    request.initial_guess = Eigen::VectorXd::Zero(7);
    request.points[0].desired_direction.setZero();
    expect_rejected_at(request, 0u);

    request.points[0] = valid_point;
    request.points.push_back(valid_point);
    request.points[1].local_axis.setZero();
    expect_rejected_at(request, 1u);

    request.points.resize(1);
    request.max_joint_step_between_points = Eigen::VectorXd::Constant(3, 0.1);
    expect_rejected_at(request, 0u);

    Eigen::VectorXd max_joint_step = Eigen::VectorXd::Constant(7, 0.1);
    max_joint_step(2) = std::numeric_limits<double>::infinity();
    request.max_joint_step_between_points = max_joint_step;
    expect_rejected_at(request, 0u);

    max_joint_step = Eigen::VectorXd::Constant(7, 0.1);
    max_joint_step(2) = -0.1;
    request.max_joint_step_between_points = max_joint_step;
    expect_rejected_at(request, 0u);

    request.max_joint_step_between_points.reset();
    request.delta_max_q = Eigen::VectorXd::Constant(3, 0.1);
    expect_rejected_at(request, 0u);

    request.delta_max_q = Eigen::VectorXd::Constant(7, 0.08);
    request.alpha = 0.0;
    expect_rejected_at(request, 0u);

    request.alpha = 0.8;
    request.tolerance = 0.0;
    expect_rejected_at(request, 0u);

    request.tolerance = 1e-6;
    request.position_tolerance = 0.0;
    expect_rejected_at(request, 0u);

    request.position_tolerance.reset();
    request.axis_tolerance = std::numeric_limits<double>::quiet_NaN();
    expect_rejected_at(request, 0u);

    request.axis_tolerance.reset();
    request.damping = -1.0;
    expect_rejected_at(request, 0u);

    request.damping = 1e-5;
    request.max_iterations = 0;
    expect_rejected_at(request, 0u);
}

TEST(AxisFreeRotateIKTest, SolvesSinglePointTrajectoryLikeAxisFreeSolve)
{
    axis_free_rotate_ik::AxisFreeRotateIK solver(test_urdf(), "base_link", "ee_link");

    Eigen::VectorXd target_q(7);
    target_q << 0.16, -0.22, 0.18, -0.14, 0.19, -0.07, 0.21;
    Eigen::VectorXd initial_guess(7);
    initial_guess << 0.18, -0.24, 0.20, -0.16, 0.21, -0.09, 0.23;

    const auto point = trajectory_point_from_q(target_q);

    axis_free_rotate_ik::IKRequest single_request = make_default_request();
    single_request.desired_position = point.desired_position;
    single_request.desired_direction = point.desired_direction;
    single_request.desired_up_vector = point.local_axis;
    single_request.initial_guess = initial_guess;
    const auto single_result = solver.solve(single_request);
    ASSERT_TRUE(single_result.success) << single_result.message;

    axis_free_rotate_ik::IKTrajectoryRequest trajectory_request = make_default_trajectory_request();
    trajectory_request.points.push_back(point);
    trajectory_request.initial_guess = initial_guess;

    const auto trajectory_result = solver.solveTrajectory(trajectory_request);
    ASSERT_TRUE(trajectory_result.success) << trajectory_result.message;
    ASSERT_EQ(trajectory_result.points.size(), 1u);
    EXPECT_FALSE(trajectory_result.first_failed_index.has_value());
    EXPECT_TRUE(trajectory_result.points[0].success);
    EXPECT_LT(trajectory_result.points[0].position_error, 1e-5);
    EXPECT_LT(trajectory_result.points[0].axis_error, 1e-5);

    const pinocchio::SE3 single_pose = base_to_end_effector(single_result.q);
    const pinocchio::SE3 trajectory_pose = base_to_end_effector(trajectory_result.points[0].q);
    EXPECT_LT((single_pose.translation() - trajectory_pose.translation()).norm(), 1e-5);
    EXPECT_LT(
        axis_angle(single_pose.rotation() * point.local_axis,
        trajectory_pose.rotation() * point.local_axis),
        1e-5);
}

TEST(AxisFreeRotateIKTest, UsesPreviousPointSolutionAsNextSeed)
{
    axis_free_rotate_ik::AxisFreeRotateIK solver(test_urdf(), "base_link", "ee_link");

    Eigen::VectorXd initial_seed(7);
    initial_seed << -0.45, 0.38, -0.33, 0.27, -0.24, 0.19, -0.16;
    Eigen::VectorXd first_target_q(7);
    first_target_q << 0.35, -0.42, 0.31, -0.26, 0.29, -0.18, 0.24;
    Eigen::VectorXd second_target_q(7);
    second_target_q << 0.42, -0.36, 0.28, -0.31, 0.34, -0.21, 0.30;

    const auto first_point = trajectory_point_from_q(first_target_q);
    const auto second_point = trajectory_point_from_q(second_target_q);

    axis_free_rotate_ik::IKRequest first_request = make_default_request();
    first_request.desired_position = first_point.desired_position;
    first_request.desired_direction = first_point.desired_direction;
    first_request.desired_up_vector = first_point.local_axis;
    first_request.initial_guess = initial_seed;
    const auto first_result = solver.solve(first_request);
    ASSERT_TRUE(first_result.success) << first_result.message;

    axis_free_rotate_ik::IKRequest from_previous_request = make_default_request();
    from_previous_request.desired_position = second_point.desired_position;
    from_previous_request.desired_direction = second_point.desired_direction;
    from_previous_request.desired_up_vector = second_point.local_axis;
    from_previous_request.initial_guess = first_result.q;
    const auto expected_from_previous = solver.solve(from_previous_request);
    ASSERT_TRUE(expected_from_previous.success) << expected_from_previous.message;

    axis_free_rotate_ik::IKRequest from_original_request = from_previous_request;
    from_original_request.initial_guess = initial_seed;
    const auto expected_from_original = solver.solve(from_original_request);
    ASSERT_TRUE(expected_from_original.success) << expected_from_original.message;

    const pinocchio::Model reduced_model = build_reduced_chain_model();
    const double seed_sensitive_distance = pinocchio::difference(
        reduced_model, expected_from_previous.q, expected_from_original.q).norm();
    ASSERT_GT(seed_sensitive_distance, 1e-4);

    axis_free_rotate_ik::IKTrajectoryRequest trajectory_request = make_default_trajectory_request();
    trajectory_request.initial_guess = initial_seed;
    trajectory_request.points.push_back(first_point);
    trajectory_request.points.push_back(second_point);

    const auto trajectory_result = solver.solveTrajectory(trajectory_request);
    ASSERT_TRUE(trajectory_result.success) << trajectory_result.message;
    ASSERT_EQ(trajectory_result.points.size(), 2u);

    const double trajectory_to_previous = pinocchio::difference(
        reduced_model, expected_from_previous.q, trajectory_result.points[1].q).norm();
    const double trajectory_to_original = pinocchio::difference(
        reduced_model, expected_from_original.q, trajectory_result.points[1].q).norm();
    EXPECT_LT(trajectory_to_previous, 1e-8);
    EXPECT_GT(trajectory_to_original, 1e-4);
}

TEST(AxisFreeRotateIKTest, SolvesMultiPointTrajectoryAndKeepsContinuity)
{
    axis_free_rotate_ik::AxisFreeRotateIK solver(test_urdf(), "base_link", "ee_link");

    std::vector<Eigen::VectorXd> target_qs;
    Eigen::VectorXd q0(7);
    q0 << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;
    Eigen::VectorXd q1(7);
    q1 << 0.13, -0.20, 0.14, -0.10, 0.16, -0.07, 0.18;
    Eigen::VectorXd q2(7);
    q2 << 0.16, -0.23, 0.17, -0.12, 0.19, -0.08, 0.21;
    target_qs.push_back(q0);
    target_qs.push_back(q1);
    target_qs.push_back(q2);

    axis_free_rotate_ik::IKTrajectoryRequest request = make_default_trajectory_request();
    request.initial_guess = q0;
    request.max_joint_step_between_points = Eigen::VectorXd::Constant(7, 0.2);
    for (const auto & target_q : target_qs) {
        request.points.push_back(trajectory_point_from_q(target_q));
    }

    const auto result = solver.solveTrajectory(request);
    ASSERT_TRUE(result.success) << result.message;
    ASSERT_EQ(result.points.size(), target_qs.size());
    EXPECT_FALSE(result.first_failed_index.has_value());

    const pinocchio::Model reduced_model = build_reduced_chain_model();
    ASSERT_EQ(reduced_model.nq, 7);
    ASSERT_EQ(reduced_model.nv, 7);
    for (size_t i = 0; i < result.points.size(); ++i) {
        EXPECT_TRUE(result.points[i].success);
        EXPECT_EQ(result.points[i].point_index, i);
        EXPECT_LT(result.points[i].position_error, 1e-5);
        EXPECT_LT(result.points[i].axis_error, 1e-5);
        if (i > 0) {
            const Eigen::VectorXd joint_step = pinocchio::difference(
                reduced_model, result.points[i - 1].q, result.points[i].q);
            EXPECT_LT(joint_step.cwiseAbs().maxCoeff(), 0.2);
        }
    }
}

TEST(AxisFreeRotateIKTest, FailsTrajectoryWhenAdjacentJointStepIsTooLarge)
{
    axis_free_rotate_ik::AxisFreeRotateIK solver(test_urdf(), "base_link", "ee_link");

    Eigen::VectorXd q0(7);
    q0 << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;
    Eigen::VectorXd q1(7);
    q1 << 0.16, -0.23, 0.17, -0.12, 0.19, -0.08, 0.21;

    axis_free_rotate_ik::IKTrajectoryRequest request = make_default_trajectory_request();
    request.initial_guess = q0;
    request.points.push_back(trajectory_point_from_q(q0));
    request.points.push_back(trajectory_point_from_q(q1));
    request.max_joint_step_between_points = Eigen::VectorXd::Constant(7, 1e-6);

    const auto result = solver.solveTrajectory(request);
    EXPECT_FALSE(result.success);
    ASSERT_TRUE(result.first_failed_index.has_value());
    EXPECT_EQ(*result.first_failed_index, 1u);
    ASSERT_EQ(result.points.size(), 2u);
    EXPECT_TRUE(result.points[0].success);
    EXPECT_FALSE(result.points[1].success);
}

TEST(AxisFreeRotateIKTest, StopsTrajectoryAtFirstUnconvergedPoint)
{
    axis_free_rotate_ik::AxisFreeRotateIK solver(test_urdf(), "base_link", "ee_link");

    Eigen::VectorXd q0(7);
    q0 << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;

    axis_free_rotate_ik::IKTrajectoryRequest request = make_default_trajectory_request();
    request.initial_guess = q0;
    request.max_iterations = 1;
    request.points.push_back(trajectory_point_from_q(q0));
    axis_free_rotate_ik::IKTrajectoryPoint unreachable_point;
    unreachable_point.desired_position = Eigen::Vector3d(5.0, 5.0, 5.0);
    unreachable_point.desired_direction = Eigen::Vector3d::UnitZ();
    unreachable_point.local_axis = Eigen::Vector3d::UnitZ();
    request.points.push_back(unreachable_point);

    const auto result = solver.solveTrajectory(request);
    EXPECT_FALSE(result.success);
    ASSERT_TRUE(result.first_failed_index.has_value());
    EXPECT_EQ(*result.first_failed_index, 1u);
    ASSERT_EQ(result.points.size(), 2u);
    EXPECT_TRUE(result.points[0].success);
    EXPECT_FALSE(result.points[1].success);
    EXPECT_FALSE(result.points[1].q.size() == 0);
}

// Five new tests for per-point option overrides (red phase expected)
TEST(AxisFreeRotateIKTest, PointSolverOptionsInvalidMaxIterations)
{
    axis_free_rotate_ik::AxisFreeRotateIK solver(test_urdf(), "base_link", "ee_link");

    Eigen::VectorXd target_q(7);
    target_q << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;

    axis_free_rotate_ik::IKTrajectoryRequest request = make_default_trajectory_request();
    auto pt = trajectory_point_from_q(target_q);
    axis_free_rotate_ik::PointSolverOptions opts;
    opts.max_iterations = -1; // invalid per-point value
    pt.solver_options = opts;
    request.points.push_back(pt);

    const auto result = solver.solveTrajectory(request);
    ASSERT_FALSE(result.success);
    ASSERT_TRUE(result.first_failed_index.has_value());
    EXPECT_EQ(0u, *result.first_failed_index);
}

TEST(AxisFreeRotateIKTest, PointSolverOptionsInvalidPositionTolerance)
{
    axis_free_rotate_ik::AxisFreeRotateIK solver(test_urdf(), "base_link", "ee_link");

    Eigen::VectorXd target_q(7);
    target_q << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;

    axis_free_rotate_ik::IKTrajectoryRequest request = make_default_trajectory_request();
    auto pt = trajectory_point_from_q(target_q);
    axis_free_rotate_ik::PointSolverOptions opts;
    opts.position_tolerance = -1.0; // invalid negative tolerance
    pt.solver_options = opts;
    request.points.push_back(pt);

    const auto result = solver.solveTrajectory(request);
    ASSERT_FALSE(result.success);
    ASSERT_TRUE(result.first_failed_index.has_value());
    EXPECT_EQ(0u, *result.first_failed_index);
}

TEST(AxisFreeRotateIKTest, PointSolverOptionsInvalidAlpha)
{
    axis_free_rotate_ik::AxisFreeRotateIK solver(test_urdf(), "base_link", "ee_link");

    Eigen::VectorXd target_q(7);
    target_q << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;

    axis_free_rotate_ik::IKTrajectoryRequest request = make_default_trajectory_request();
    auto pt = trajectory_point_from_q(target_q);
    axis_free_rotate_ik::PointSolverOptions opts;
    opts.alpha = -0.5; // invalid negative alpha
    pt.solver_options = opts;
    request.points.push_back(pt);

    const auto result = solver.solveTrajectory(request);
    ASSERT_FALSE(result.success);
    ASSERT_TRUE(result.first_failed_index.has_value());
    EXPECT_EQ(0u, *result.first_failed_index);
}

TEST(AxisFreeRotateIKTest, PointSolverOptionsAllFieldsOverridden)
{
    axis_free_rotate_ik::AxisFreeRotateIK solver(test_urdf(), "base_link", "ee_link");

    Eigen::VectorXd target_q(7);
    target_q << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;
    const auto point = trajectory_point_from_q(target_q);

    axis_free_rotate_ik::PointSolverOptions opts;
    opts.max_iterations = 50;
    opts.position_tolerance = 1e-3;
    opts.axis_tolerance = 1e-3;
    opts.damping = 1e-3;
    opts.alpha = 0.1;
    opts.delta_max_q = Eigen::VectorXd::Constant(7, 0.5);
    auto point_with_opts = point;
    point_with_opts.solver_options = opts;

    axis_free_rotate_ik::IKTrajectoryRequest request = make_default_trajectory_request();
    request.points.push_back(point_with_opts);

    const auto result = solver.solveTrajectory(request);
    ASSERT_TRUE(result.success) << result.message;
    ASSERT_FALSE(result.points.empty());
    EXPECT_TRUE(result.points[0].success);
}

TEST(AxisFreeRotateIKTest, PointSolverOptionsDeltaMaxQVectorSize)
{
    axis_free_rotate_ik::AxisFreeRotateIK solver(test_urdf(), "base_link", "ee_link");

    Eigen::VectorXd target_q(7);
    target_q << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;
    auto pt = trajectory_point_from_q(target_q);

    axis_free_rotate_ik::PointSolverOptions opts;
    opts.delta_max_q = Eigen::VectorXd::Constant(3, 0.1);
    pt.solver_options = opts;

    axis_free_rotate_ik::IKTrajectoryRequest request = make_default_trajectory_request();
    request.points.push_back(pt);

    const auto result = solver.solveTrajectory(request);
    ASSERT_FALSE(result.success);
    ASSERT_TRUE(result.first_failed_index.has_value());
    ASSERT_EQ(0u, *result.first_failed_index);
}

TEST(AxisFreeRotateIKTest, PointSolverOptionsInvalidDamping)
{
    axis_free_rotate_ik::AxisFreeRotateIK solver(test_urdf(), "base_link", "ee_link");

    Eigen::VectorXd target_q(7);
    target_q << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;

    axis_free_rotate_ik::IKTrajectoryRequest request = make_default_trajectory_request();
    auto pt = trajectory_point_from_q(target_q);
    axis_free_rotate_ik::PointSolverOptions opts;
    opts.damping = -0.1; // invalid negative damping
    pt.solver_options = opts;
    request.points.push_back(pt);

    const auto result = solver.solveTrajectory(request);
    ASSERT_FALSE(result.success);
    ASSERT_TRUE(result.first_failed_index.has_value());
    EXPECT_EQ(0u, *result.first_failed_index);
}

TEST(AxisFreeRotateIKTest, PointSolverOptionsInvalidAxisTolerance)
{
    axis_free_rotate_ik::AxisFreeRotateIK solver(test_urdf(), "base_link", "ee_link");

    Eigen::VectorXd target_q(7);
    target_q << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;

    axis_free_rotate_ik::IKTrajectoryRequest request = make_default_trajectory_request();
    auto pt = trajectory_point_from_q(target_q);
    axis_free_rotate_ik::PointSolverOptions opts;
    opts.axis_tolerance = -1.0; // invalid negative axis tolerance
    pt.solver_options = opts;
    request.points.push_back(pt);

    const auto result = solver.solveTrajectory(request);
    ASSERT_FALSE(result.success);
    ASSERT_TRUE(result.first_failed_index.has_value());
    EXPECT_EQ(0u, *result.first_failed_index);
}

TEST(AxisFreeRotateIKTest, RecordsAxisFreeIkPerformanceBaseline)
{
    axis_free_rotate_ik::AxisFreeRotateIK solver(test_urdf(), "base_link", "ee_link");

    std::vector<Eigen::VectorXd> target_qs;
    std::vector<axis_free_rotate_ik::IKTrajectoryPoint> points;
    target_qs.reserve(100);
    points.reserve(100);

    for (int i = 0; i < 100; ++i) {
        const double t = static_cast<double>(i) / 99.0;
        Eigen::VectorXd q(7);
        q << 0.12 + 0.12 * std::sin(2.0 * kPi * t),
            -0.22 + 0.08 * std::cos(2.0 * kPi * t),
            0.16 + 0.06 * std::sin(4.0 * kPi * t),
            -0.11 + 0.05 * std::cos(4.0 * kPi * t),
            0.18 + 0.07 * std::sin(3.0 * kPi * t),
            -0.08 + 0.04 * std::cos(3.0 * kPi * t),
            0.20 + 0.05 * std::sin(5.0 * kPi * t);
        target_qs.push_back(q);
        points.push_back(trajectory_point_from_q(q));
    }

    std::vector<double> single_point_us;
    single_point_us.reserve(points.size());
    size_t success_count = 0;

    for (size_t i = 0; i < points.size(); ++i) {
        axis_free_rotate_ik::IKRequest request = make_default_request();
        request.desired_position = points[i].desired_position;
        request.desired_direction = points[i].desired_direction;
        request.desired_up_vector = points[i].local_axis;
        request.initial_guess = target_qs[i] + Eigen::VectorXd::Constant(7, 0.015);

        const auto start = std::chrono::steady_clock::now();
        const auto result = solver.solve(request);
        const auto end = std::chrono::steady_clock::now();

        single_point_us.push_back(
            std::chrono::duration<double, std::micro>(end - start).count());
        if (result.success) {
            ++success_count;
        }
    }

    std::vector<double> sorted_single_point_us = single_point_us;
    std::sort(sorted_single_point_us.begin(), sorted_single_point_us.end());
    const double single_point_avg_us =
        std::accumulate(single_point_us.begin(), single_point_us.end(), 0.0) /
        static_cast<double>(single_point_us.size());
    const double single_point_p95_us =
        sorted_single_point_us[static_cast<size_t>(0.95 * (sorted_single_point_us.size() - 1))];

    axis_free_rotate_ik::IKTrajectoryRequest trajectory_request = make_default_trajectory_request();
    trajectory_request.initial_guess = target_qs.front() + Eigen::VectorXd::Constant(7, 0.015);
    trajectory_request.points = points;

    const auto trajectory_start = std::chrono::steady_clock::now();
    const auto trajectory_result = solver.solveTrajectory(trajectory_request);
    const auto trajectory_end = std::chrono::steady_clock::now();
    const double trajectory_100_points_ms =
        std::chrono::duration<double, std::milli>(trajectory_end - trajectory_start).count();

    std::cout << "axis_free_rotate_ik_performance_baseline "
              << "single_point_free_ik_avg_us=" << single_point_avg_us << " "
              << "single_point_free_ik_p95_us=" << single_point_p95_us << " "
              << "trajectory_100_points_avg_ms=" << trajectory_100_points_ms << " "
              << "success_rate=" << static_cast<double>(success_count) /
        static_cast<double>(points.size())
              << std::endl;

    EXPECT_EQ(success_count, points.size());
    ASSERT_TRUE(trajectory_result.success) << trajectory_result.message;
    ASSERT_EQ(trajectory_result.points.size(), points.size());
}
