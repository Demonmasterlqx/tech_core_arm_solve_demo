#include "axis_free_rotate_ik/axis_free_rotate_ik_ros.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace
{

using axis_free_rotate_ik::AxisFreeRotateIKRos;
using axis_free_rotate_ik::srv::SolveAxisFreeIK;
using axis_free_rotate_ik::srv::SolveFixedIK;

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
<robot name="axis_free_rotate_ik_ros_test">
  <link name="world"/>
  <joint name="world_to_base" type="fixed">
    <origin xyz="0.1 -0.2 0.3" rpy="0.1 -0.2 0.3"/>
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
  <link name="base_link"/>

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

pinocchio::SE3 base_to_end_effector(const Eigen::VectorXd & chain_configuration)
{
    pinocchio::Model model;
    pinocchio::urdf::buildModelFromXML(test_urdf(), model);
    pinocchio::Data data(model);
    Eigen::VectorXd q = pinocchio::neutral(model);

    const auto & names = chain_joint_names();
    for (size_t i = 0; i < names.size(); ++i) {
        const pinocchio::JointIndex joint_id = model.getJointId(names[i]);
        q(model.joints[joint_id].idx_q()) = chain_configuration(static_cast<Eigen::Index>(i));
    }

    pinocchio::framesForwardKinematics(model, data, q);
    const pinocchio::FrameIndex base_frame_id = model.getFrameId("base_link");
    const pinocchio::FrameIndex ee_frame_id = model.getFrameId("ee_link");
    return data.oMf[base_frame_id].inverse() * data.oMf[ee_frame_id];
}

geometry_msgs::msg::PoseStamped pose_from_se3(const pinocchio::SE3 & pose, const std::string & frame_id)
{
    const Eigen::Quaterniond quaternion(pose.rotation());

    geometry_msgs::msg::PoseStamped output;
    output.header.frame_id = frame_id;
    output.pose.position.x = pose.translation().x();
    output.pose.position.y = pose.translation().y();
    output.pose.position.z = pose.translation().z();
    output.pose.orientation.x = quaternion.x();
    output.pose.orientation.y = quaternion.y();
    output.pose.orientation.z = quaternion.z();
    output.pose.orientation.w = quaternion.w();
    return output;
}

geometry_msgs::msg::PoseStamped pose_from_q(const Eigen::VectorXd & q)
{
    return pose_from_se3(base_to_end_effector(q), "base_link");
}

std::vector<double> to_std_vector(const Eigen::VectorXd & vector)
{
    std::vector<double> output;
    output.reserve(static_cast<size_t>(vector.size()));
    for (Eigen::Index i = 0; i < vector.size(); ++i) {
        output.push_back(vector(i));
    }
    return output;
}

std::string unique_topic(const std::string & suffix)
{
    static std::atomic<int> counter{0};
    return "/axis_free_rotate_ik_ros_test/" + suffix + "_" + std::to_string(counter.fetch_add(1));
}

rclcpp::NodeOptions node_options_for_topic(
    const std::string & topic,
    const double timeout = 2.0,
    const std::string & joint_trajectory_topic = "")
{
    rclcpp::NodeOptions options;
    options.parameter_overrides({
            rclcpp::Parameter("robot_description_topic", topic),
            rclcpp::Parameter("robot_description_timeout", timeout),
            rclcpp::Parameter("base_link", "base_link"),
            rclcpp::Parameter("end_effector_link", "ee_link"),
            rclcpp::Parameter("joint_trajectory_topic", joint_trajectory_topic)
        });
    return options;
}

std::string unique_node_name(const std::string & prefix)
{
    static std::atomic<int> counter{0};
    return prefix + "_" + std::to_string(counter.fetch_add(1));
}

bool wait_for_publisher(const rclcpp::Node::SharedPtr & node, const std::string & topic)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
        if (node->count_publishers(topic) > 0) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

void expect_trajectories_equal(
    const trajectory_msgs::msg::JointTrajectory & expected,
    const trajectory_msgs::msg::JointTrajectory & actual)
{
    EXPECT_EQ(actual.header.frame_id, expected.header.frame_id);
    EXPECT_EQ(actual.header.stamp.sec, expected.header.stamp.sec);
    EXPECT_EQ(actual.header.stamp.nanosec, expected.header.stamp.nanosec);
    EXPECT_EQ(actual.joint_names, expected.joint_names);
    ASSERT_EQ(actual.points.size(), expected.points.size());
    for (size_t i = 0; i < expected.points.size(); ++i) {
        EXPECT_EQ(actual.points[i].positions, expected.points[i].positions);
        EXPECT_EQ(actual.points[i].velocities, expected.points[i].velocities);
        EXPECT_EQ(actual.points[i].accelerations, expected.points[i].accelerations);
        EXPECT_EQ(actual.points[i].effort, expected.points[i].effort);
        EXPECT_EQ(actual.points[i].time_from_start.sec, expected.points[i].time_from_start.sec);
        EXPECT_EQ(actual.points[i].time_from_start.nanosec, expected.points[i].time_from_start.nanosec);
    }
}

class RobotDescriptionPublisher
{
public:
    explicit RobotDescriptionPublisher(std::string topic)
        : topic_(std::move(topic))
    {
        thread_ = std::thread([this]() {
                auto node = std::make_shared<rclcpp::Node>(
                    "axis_free_rotate_ik_ros_test_robot_description_publisher");
                auto publisher = node->create_publisher<std_msgs::msg::String>(
                    topic_,
                    rclcpp::QoS(1).reliable().transient_local());

                std_msgs::msg::String message;
                message.data = test_urdf();

                rclcpp::WallRate rate(50.0);
                while (!stop_.load() && rclcpp::ok()) {
                    publisher->publish(message);
                    rate.sleep();
                }
            });
    }

    ~RobotDescriptionPublisher()
    {
        stop_.store(true);
        if (thread_.joinable()) {
            thread_.join();
        }
    }

private:
    std::string topic_;
    std::atomic<bool> stop_{false};
    std::thread thread_;
};

class StaticTransformPublisher
{
public:
    StaticTransformPublisher(const std::string & parent_frame, const std::string & child_frame)
        : node_(std::make_shared<rclcpp::Node>("axis_free_rotate_ik_ros_test_static_tf_publisher")),
        broadcaster_(std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_))
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = node_->now();
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = child_frame;
        transform.transform.translation.x = 0.03;
        transform.transform.translation.y = -0.02;
        transform.transform.translation.z = 0.04;
        const Eigen::Quaterniond rotation(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));
        transform.transform.rotation.x = rotation.x();
        transform.transform.rotation.y = rotation.y();
        transform.transform.rotation.z = rotation.z();
        transform.transform.rotation.w = rotation.w();
        broadcaster_->sendTransform(transform);
        base_to_child_ = pinocchio::SE3(
            rotation.toRotationMatrix(),
            Eigen::Vector3d(
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z));
    }

    const pinocchio::SE3 & base_to_child() const
    {
        return base_to_child_;
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
    pinocchio::SE3 base_to_child_{pinocchio::SE3::Identity()};
};

class TrajectoryRecorder
{
public:
    explicit TrajectoryRecorder(const std::string & topic)
        : node_(std::make_shared<rclcpp::Node>(
                unique_node_name("axis_free_rotate_ik_ros_test_trajectory_recorder")))
    {
        subscription_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            topic,
            10,
            [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr message) {
                messages_.push_back(*message);
            });
    }

    const rclcpp::Node::SharedPtr & node() const
    {
        return node_;
    }

    const std::vector<trajectory_msgs::msg::JointTrajectory> & messages() const
    {
        return messages_;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
    std::vector<trajectory_msgs::msg::JointTrajectory> messages_;
};

std::shared_ptr<AxisFreeRotateIKRos> make_initialized_wrapper(
    const std::string & topic,
    const std::string & joint_trajectory_topic = "")
{
    RobotDescriptionPublisher publisher(topic);
    return std::make_shared<AxisFreeRotateIKRos>(
        node_options_for_topic(topic, 2.0, joint_trajectory_topic));
}

template<typename ServiceT>
typename ServiceT::Response::SharedPtr call_service(
    const std::shared_ptr<AxisFreeRotateIKRos> & wrapper,
    const std::string & service_suffix,
    const typename ServiceT::Request & request,
    const std::vector<rclcpp::Node::SharedPtr> & extra_nodes = {},
    const std::chrono::milliseconds post_response_spin = std::chrono::milliseconds(0))
{
    auto client_node = std::make_shared<rclcpp::Node>("axis_free_rotate_ik_ros_test_client");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(wrapper);
    executor.add_node(client_node);
    for (const auto & extra_node : extra_nodes) {
        executor.add_node(extra_node);
    }

    const std::string service_name = std::string(wrapper->get_fully_qualified_name()) + service_suffix;
    auto client = client_node->create_client<ServiceT>(service_name);
    if (!client->wait_for_service(std::chrono::seconds(1))) {
        return nullptr;
    }

    auto request_ptr = std::make_shared<typename ServiceT::Request>(request);
    auto future = client->async_send_request(request_ptr);
    if (executor.spin_until_future_complete(future, std::chrono::seconds(2)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        return nullptr;
    }

    auto response = future.get();
    if (post_response_spin > std::chrono::milliseconds(0)) {
        const auto deadline = std::chrono::steady_clock::now() + post_response_spin;
        while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
            executor.spin_some(std::chrono::milliseconds(10));
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    return response;
}

SolveFixedIK::Request make_valid_fixed_request(
    const Eigen::VectorXd & q0,
    const Eigen::VectorXd & q1,
    const double point_time_interval)
{
    SolveFixedIK::Request request;
    request.target_poses = {pose_from_q(q0), pose_from_q(q1)};
    request.point_time_interval = point_time_interval;
    request.tf_timeout = 1.0;
    request.initial_guess = to_std_vector(q0);
    request.max_iterations = 200;
    request.tolerance = 1e-6;
    request.damping = 1e-5;
    request.alpha = 0.8;
    return request;
}

SolveAxisFreeIK::Request make_valid_axis_free_request(
    const Eigen::VectorXd & q0,
    const Eigen::VectorXd & q1,
    const double point_time_interval)
{
    SolveAxisFreeIK::Request request;
    request.target_poses = {pose_from_q(q0), pose_from_q(q1)};
    request.local_axis.z = 1.0;
    request.point_time_interval = point_time_interval;
    request.tf_timeout = 1.0;
    request.initial_guess = to_std_vector(q0);
    request.max_joint_step_between_points = std::vector<double>(7, 0.2);
    request.max_iterations = 200;
    request.tolerance = 1e-6;
    request.damping = 1e-5;
    request.alpha = 0.8;
    return request;
}

class AxisFreeRotateIKRosTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        if (!rclcpp::ok()) {
            int argc = 0;
            char ** argv = nullptr;
            rclcpp::init(argc, argv);
        }
    }

    static void TearDownTestSuite()
    {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }
};

}  // namespace

TEST_F(AxisFreeRotateIKRosTest, InitializesFromTransientLocalRobotDescription)
{
    const auto wrapper = make_initialized_wrapper(unique_topic("robot_description"));
    ASSERT_NE(wrapper, nullptr);
}

TEST_F(AxisFreeRotateIKRosTest, ThrowsWhenRobotDescriptionTimesOut)
{
    EXPECT_THROW(
        std::make_shared<AxisFreeRotateIKRos>(
            node_options_for_topic(unique_topic("missing_robot_description"), 0.02)),
        std::runtime_error);
}

TEST_F(AxisFreeRotateIKRosTest, FixedServiceReturnsJointTrajectory)
{
    const auto wrapper = make_initialized_wrapper(unique_topic("fixed"));
    StaticTransformPublisher static_tf("base_link", "fixed_target_frame");

    Eigen::VectorXd q0(7);
    q0 << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;
    Eigen::VectorXd q1(7);
    q1 << 0.13, -0.20, 0.14, -0.10, 0.16, -0.07, 0.18;

    SolveFixedIK::Request request;
    request.target_poses = {
        pose_from_se3(static_tf.base_to_child().inverse() * base_to_end_effector(q0), "fixed_target_frame"),
        pose_from_se3(static_tf.base_to_child().inverse() * base_to_end_effector(q1), "fixed_target_frame")};
    request.point_time_interval = 0.2;
    request.tf_timeout = 1.0;
    request.initial_guess = to_std_vector(q0);
    request.max_iterations = 200;
    request.tolerance = 1e-6;
    request.damping = 1e-5;
    request.alpha = 0.8;

    const auto response = call_service<SolveFixedIK>(wrapper, "/solve_fixed_ik", request);
    ASSERT_NE(response, nullptr);
    EXPECT_EQ(response->trajectory.joint_names, chain_joint_names());
    ASSERT_EQ(response->trajectory.points.size(), 2u) << response->message;
    EXPECT_NE(response->message.find("succeeded"), std::string::npos);
    EXPECT_EQ(response->trajectory.points[0].positions.size(), 7u);
    EXPECT_EQ(response->trajectory.points[0].velocities.size(), 7u);
    EXPECT_NEAR(rclcpp::Duration(response->trajectory.points[0].time_from_start).seconds(), 0.2, 1e-9);
    EXPECT_NEAR(rclcpp::Duration(response->trajectory.points[1].time_from_start).seconds(), 0.4, 1e-9);
}

TEST_F(AxisFreeRotateIKRosTest, AxisFreeServiceReturnsJointTrajectory)
{
    const auto wrapper = make_initialized_wrapper(unique_topic("axis_free"));
    StaticTransformPublisher static_tf("base_link", "axis_free_target_frame");

    Eigen::VectorXd q0(7);
    q0 << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;
    Eigen::VectorXd q1(7);
    q1 << 0.13, -0.20, 0.14, -0.10, 0.16, -0.07, 0.18;

    SolveAxisFreeIK::Request request;
    request.target_poses = {
        pose_from_se3(static_tf.base_to_child().inverse() * base_to_end_effector(q0), "axis_free_target_frame"),
        pose_from_se3(static_tf.base_to_child().inverse() * base_to_end_effector(q1), "axis_free_target_frame")};
    request.local_axis.z = 1.0;
    request.point_time_interval = 0.15;
    request.tf_timeout = 1.0;
    request.initial_guess = to_std_vector(q0);
    request.max_joint_step_between_points = std::vector<double>(7, 0.2);
    request.max_iterations = 200;
    request.tolerance = 1e-6;
    request.damping = 1e-5;
    request.alpha = 0.8;

    const auto response = call_service<SolveAxisFreeIK>(wrapper, "/solve_axis_free_ik", request);
    ASSERT_NE(response, nullptr);
    EXPECT_EQ(response->trajectory.joint_names, chain_joint_names());
    ASSERT_EQ(response->trajectory.points.size(), 2u) << response->message;
    EXPECT_NE(response->message.find("succeeded"), std::string::npos);
    EXPECT_EQ(response->trajectory.points[0].positions.size(), 7u);
    EXPECT_EQ(response->trajectory.points[0].velocities.size(), 7u);
    EXPECT_NEAR(rclcpp::Duration(response->trajectory.points[0].time_from_start).seconds(), 0.15, 1e-9);
    EXPECT_NEAR(rclcpp::Duration(response->trajectory.points[1].time_from_start).seconds(), 0.30, 1e-9);
}

TEST_F(AxisFreeRotateIKRosTest, PublishRequestIsIgnoredWhenTrajectoryTopicIsEmpty)
{
    const auto wrapper = make_initialized_wrapper(unique_topic("publish_empty_topic"));

    Eigen::VectorXd q0(7);
    q0 << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;
    Eigen::VectorXd q1(7);
    q1 << 0.13, -0.20, 0.14, -0.10, 0.16, -0.07, 0.18;

    auto request = make_valid_fixed_request(q0, q1, 0.2);
    request.publish_trajectory = true;

    const auto response = call_service<SolveFixedIK>(wrapper, "/solve_fixed_ik", request);
    ASSERT_NE(response, nullptr);
    EXPECT_EQ(response->trajectory.joint_names, chain_joint_names());
    ASSERT_EQ(response->trajectory.points.size(), 2u) << response->message;
    EXPECT_NE(response->message.find("succeeded"), std::string::npos);
}

TEST_F(AxisFreeRotateIKRosTest, DoesNotPublishWhenRequestFlagIsFalse)
{
    const std::string trajectory_topic = unique_topic("trajectory_publish_disabled");
    const auto wrapper = make_initialized_wrapper(unique_topic("publish_false"), trajectory_topic);
    TrajectoryRecorder recorder(trajectory_topic);
    ASSERT_TRUE(wait_for_publisher(recorder.node(), trajectory_topic));

    Eigen::VectorXd q0(7);
    q0 << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;
    Eigen::VectorXd q1(7);
    q1 << 0.13, -0.20, 0.14, -0.10, 0.16, -0.07, 0.18;

    auto request = make_valid_fixed_request(q0, q1, 0.2);
    request.publish_trajectory = false;

    const auto response = call_service<SolveFixedIK>(
        wrapper,
        "/solve_fixed_ik",
        request,
        {recorder.node()},
        std::chrono::milliseconds(200));
    ASSERT_NE(response, nullptr);
    ASSERT_EQ(response->trajectory.points.size(), 2u) << response->message;
    EXPECT_TRUE(recorder.messages().empty());
}

TEST_F(AxisFreeRotateIKRosTest, FixedServicePublishesRequestedTrajectory)
{
    const std::string trajectory_topic = unique_topic("fixed_trajectory_publish");
    const auto wrapper = make_initialized_wrapper(unique_topic("fixed_publish"), trajectory_topic);
    TrajectoryRecorder recorder(trajectory_topic);
    ASSERT_TRUE(wait_for_publisher(recorder.node(), trajectory_topic));

    Eigen::VectorXd q0(7);
    q0 << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;
    Eigen::VectorXd q1(7);
    q1 << 0.13, -0.20, 0.14, -0.10, 0.16, -0.07, 0.18;

    auto request = make_valid_fixed_request(q0, q1, 0.2);
    request.publish_trajectory = true;

    const auto response = call_service<SolveFixedIK>(
        wrapper,
        "/solve_fixed_ik",
        request,
        {recorder.node()},
        std::chrono::milliseconds(300));
    ASSERT_NE(response, nullptr);
    ASSERT_EQ(response->trajectory.points.size(), 2u) << response->message;
    ASSERT_EQ(recorder.messages().size(), 1u);
    expect_trajectories_equal(response->trajectory, recorder.messages().front());
}

TEST_F(AxisFreeRotateIKRosTest, AxisFreeServicePublishesRequestedTrajectory)
{
    const std::string trajectory_topic = unique_topic("axis_free_trajectory_publish");
    const auto wrapper = make_initialized_wrapper(unique_topic("axis_free_publish"), trajectory_topic);
    TrajectoryRecorder recorder(trajectory_topic);
    ASSERT_TRUE(wait_for_publisher(recorder.node(), trajectory_topic));

    Eigen::VectorXd q0(7);
    q0 << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;
    Eigen::VectorXd q1(7);
    q1 << 0.13, -0.20, 0.14, -0.10, 0.16, -0.07, 0.18;

    auto request = make_valid_axis_free_request(q0, q1, 0.15);
    request.publish_trajectory = true;

    const auto response = call_service<SolveAxisFreeIK>(
        wrapper,
        "/solve_axis_free_ik",
        request,
        {recorder.node()},
        std::chrono::milliseconds(300));
    ASSERT_NE(response, nullptr);
    ASSERT_EQ(response->trajectory.points.size(), 2u) << response->message;
    ASSERT_EQ(recorder.messages().size(), 1u);
    expect_trajectories_equal(response->trajectory, recorder.messages().front());
}

TEST_F(AxisFreeRotateIKRosTest, ServicesRejectInvalidRequests)
{
    const auto wrapper = make_initialized_wrapper(unique_topic("invalid"));

    SolveFixedIK::Request empty_fixed_request;
    empty_fixed_request.point_time_interval = 0.1;
    const auto empty_fixed_response =
        call_service<SolveFixedIK>(wrapper, "/solve_fixed_ik", empty_fixed_request);
    ASSERT_NE(empty_fixed_response, nullptr);
    EXPECT_TRUE(empty_fixed_response->trajectory.points.empty());
    EXPECT_NE(empty_fixed_response->message.find("target_poses cannot be empty"), std::string::npos);

    Eigen::VectorXd q0(7);
    q0 << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;

    SolveFixedIK::Request bad_frame_request;
    bad_frame_request.target_poses = {pose_from_q(q0)};
    bad_frame_request.target_poses[0].header.frame_id.clear();
    bad_frame_request.point_time_interval = 0.1;
    const auto bad_frame_response =
        call_service<SolveFixedIK>(wrapper, "/solve_fixed_ik", bad_frame_request);
    ASSERT_NE(bad_frame_response, nullptr);
    EXPECT_TRUE(bad_frame_response->trajectory.points.empty());
    EXPECT_NE(bad_frame_response->message.find("frame_id cannot be empty"), std::string::npos);

    SolveAxisFreeIK::Request bad_axis_request;
    bad_axis_request.target_poses = {pose_from_q(q0)};
    bad_axis_request.point_time_interval = 0.1;
    const auto bad_axis_response =
        call_service<SolveAxisFreeIK>(wrapper, "/solve_axis_free_ik", bad_axis_request);
    ASSERT_NE(bad_axis_response, nullptr);
    EXPECT_TRUE(bad_axis_response->trajectory.points.empty());
    EXPECT_NE(bad_axis_response->message.find("local_axis"), std::string::npos);

    SolveAxisFreeIK::Request bad_seed_request;
    bad_seed_request.target_poses = {pose_from_q(q0)};
    bad_seed_request.local_axis.z = 1.0;
    bad_seed_request.point_time_interval = 0.1;
    bad_seed_request.initial_guess = {0.0, 0.0, 0.0};
    const auto bad_seed_response =
        call_service<SolveAxisFreeIK>(wrapper, "/solve_axis_free_ik", bad_seed_request);
    ASSERT_NE(bad_seed_response, nullptr);
    EXPECT_TRUE(bad_seed_response->trajectory.points.empty());
    EXPECT_NE(bad_seed_response->message.find("initial_guess size"), std::string::npos);

    SolveFixedIK::Request bad_scalar_request;
    bad_scalar_request.target_poses = {pose_from_q(q0)};
    bad_scalar_request.point_time_interval = 0.1;
    bad_scalar_request.tolerance = std::numeric_limits<double>::quiet_NaN();
    const auto bad_scalar_response =
        call_service<SolveFixedIK>(wrapper, "/solve_fixed_ik", bad_scalar_request);
    ASSERT_NE(bad_scalar_response, nullptr);
    EXPECT_TRUE(bad_scalar_response->trajectory.points.empty());
    EXPECT_NE(bad_scalar_response->message.find("tolerance must be finite"), std::string::npos);

    SolveFixedIK::Request bad_alpha_request;
    bad_alpha_request.target_poses = {pose_from_q(q0)};
    bad_alpha_request.point_time_interval = 0.1;
    bad_alpha_request.alpha = 1.5;
    const auto bad_alpha_response =
        call_service<SolveFixedIK>(wrapper, "/solve_fixed_ik", bad_alpha_request);
    ASSERT_NE(bad_alpha_response, nullptr);
    EXPECT_TRUE(bad_alpha_response->trajectory.points.empty());
    EXPECT_NE(bad_alpha_response->message.find("alpha must be in the range"), std::string::npos);

    SolveFixedIK::Request bad_delta_request;
    bad_delta_request.target_poses = {pose_from_q(q0)};
    bad_delta_request.point_time_interval = 0.1;
    bad_delta_request.delta_max_q = std::vector<double>(7, 0.1);
    bad_delta_request.delta_max_q[3] = -0.1;
    const auto bad_delta_response =
        call_service<SolveFixedIK>(wrapper, "/solve_fixed_ik", bad_delta_request);
    ASSERT_NE(bad_delta_response, nullptr);
    EXPECT_TRUE(bad_delta_response->trajectory.points.empty());
    EXPECT_NE(bad_delta_response->message.find("delta_max_q must be non-negative"), std::string::npos);

    SolveAxisFreeIK::Request bad_axis_alpha_request;
    bad_axis_alpha_request.target_poses = {pose_from_q(q0)};
    bad_axis_alpha_request.local_axis.z = 1.0;
    bad_axis_alpha_request.point_time_interval = 0.1;
    bad_axis_alpha_request.alpha = 1.5;
    const auto bad_axis_alpha_response =
        call_service<SolveAxisFreeIK>(wrapper, "/solve_axis_free_ik", bad_axis_alpha_request);
    ASSERT_NE(bad_axis_alpha_response, nullptr);
    EXPECT_TRUE(bad_axis_alpha_response->trajectory.points.empty());
    EXPECT_NE(bad_axis_alpha_response->message.find("alpha must be in the range"), std::string::npos);

    SolveAxisFreeIK::Request bad_axis_delta_request;
    bad_axis_delta_request.target_poses = {pose_from_q(q0)};
    bad_axis_delta_request.local_axis.z = 1.0;
    bad_axis_delta_request.point_time_interval = 0.1;
    bad_axis_delta_request.delta_max_q = std::vector<double>(7, 0.1);
    bad_axis_delta_request.delta_max_q[3] = -0.1;
    const auto bad_axis_delta_response =
        call_service<SolveAxisFreeIK>(wrapper, "/solve_axis_free_ik", bad_axis_delta_request);
    ASSERT_NE(bad_axis_delta_response, nullptr);
    EXPECT_TRUE(bad_axis_delta_response->trajectory.points.empty());
    EXPECT_NE(bad_axis_delta_response->message.find("delta_max_q must be non-negative"), std::string::npos);

    SolveAxisFreeIK::Request bad_axis_step_request;
    bad_axis_step_request.target_poses = {pose_from_q(q0)};
    bad_axis_step_request.local_axis.z = 1.0;
    bad_axis_step_request.point_time_interval = 0.1;
    bad_axis_step_request.max_joint_step_between_points = std::vector<double>(7, 0.1);
    bad_axis_step_request.max_joint_step_between_points[3] = -0.1;
    const auto bad_axis_step_response =
        call_service<SolveAxisFreeIK>(wrapper, "/solve_axis_free_ik", bad_axis_step_request);
    ASSERT_NE(bad_axis_step_response, nullptr);
    EXPECT_TRUE(bad_axis_step_response->trajectory.points.empty());
    EXPECT_NE(
        bad_axis_step_response->message.find("max_joint_step_between_points must be non-negative"),
        std::string::npos);

    SolveAxisFreeIK::Request bad_tf_timeout_request;
    bad_tf_timeout_request.target_poses = {pose_from_q(q0)};
    bad_tf_timeout_request.local_axis.z = 1.0;
    bad_tf_timeout_request.point_time_interval = 0.1;
    bad_tf_timeout_request.tf_timeout = std::numeric_limits<double>::infinity();
    const auto bad_tf_timeout_response =
        call_service<SolveAxisFreeIK>(wrapper, "/solve_axis_free_ik", bad_tf_timeout_request);
    ASSERT_NE(bad_tf_timeout_response, nullptr);
    EXPECT_TRUE(bad_tf_timeout_response->trajectory.points.empty());
    EXPECT_NE(bad_tf_timeout_response->message.find("tf_timeout must be finite"), std::string::npos);
}

TEST_F(AxisFreeRotateIKRosTest, FixedServiceValidatesAllPointsBeforeSolving)
{
    const auto wrapper = make_initialized_wrapper(unique_topic("fixed_validate_first"));

    geometry_msgs::msg::PoseStamped unreachable_first_target;
    unreachable_first_target.header.frame_id = "base_link";
    unreachable_first_target.pose.position.x = 5.0;
    unreachable_first_target.pose.position.y = 5.0;
    unreachable_first_target.pose.position.z = 5.0;
    unreachable_first_target.pose.orientation.w = 1.0;

    Eigen::VectorXd q0(7);
    q0 << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;
    auto invalid_second_target = pose_from_q(q0);
    invalid_second_target.header.frame_id.clear();

    SolveFixedIK::Request request;
    request.target_poses = {unreachable_first_target, invalid_second_target};
    request.point_time_interval = 0.1;
    request.max_iterations = 1;

    const auto response = call_service<SolveFixedIK>(wrapper, "/solve_fixed_ik", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->trajectory.points.empty());
    EXPECT_NE(response->message.find("point 1"), std::string::npos);
    EXPECT_NE(response->message.find("frame_id cannot be empty"), std::string::npos);
}

TEST_F(AxisFreeRotateIKRosTest, FixedServiceValidatesAllTfBeforeSolving)
{
    const auto wrapper = make_initialized_wrapper(unique_topic("fixed_validate_tf_first"));

    geometry_msgs::msg::PoseStamped unreachable_first_target;
    unreachable_first_target.header.frame_id = "base_link";
    unreachable_first_target.pose.position.x = 5.0;
    unreachable_first_target.pose.position.y = 5.0;
    unreachable_first_target.pose.position.z = 5.0;
    unreachable_first_target.pose.orientation.w = 1.0;

    Eigen::VectorXd q0(7);
    q0 << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;
    auto missing_tf_second_target = pose_from_q(q0);
    missing_tf_second_target.header.frame_id = "missing_fixed_target_frame";

    SolveFixedIK::Request request;
    request.target_poses = {unreachable_first_target, missing_tf_second_target};
    request.point_time_interval = 0.1;
    request.max_iterations = 1;

    const auto response = call_service<SolveFixedIK>(wrapper, "/solve_fixed_ik", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->trajectory.points.empty());
    EXPECT_NE(response->message.find("point 1"), std::string::npos);
    EXPECT_NE(response->message.find("Failed to transform pose"), std::string::npos);
}

TEST_F(AxisFreeRotateIKRosTest, FixedServiceValidatesAllPoseContentBeforeSolving)
{
    const auto wrapper = make_initialized_wrapper(unique_topic("fixed_validate_pose_first"));

    geometry_msgs::msg::PoseStamped unreachable_first_target;
    unreachable_first_target.header.frame_id = "base_link";
    unreachable_first_target.pose.position.x = 5.0;
    unreachable_first_target.pose.position.y = 5.0;
    unreachable_first_target.pose.position.z = 5.0;
    unreachable_first_target.pose.orientation.w = 1.0;

    Eigen::VectorXd q0(7);
    q0 << 0.10, -0.18, 0.12, -0.09, 0.14, -0.06, 0.16;
    auto invalid_orientation_second_target = pose_from_q(q0);
    invalid_orientation_second_target.pose.orientation.x = 0.0;
    invalid_orientation_second_target.pose.orientation.y = 0.0;
    invalid_orientation_second_target.pose.orientation.z = 0.0;
    invalid_orientation_second_target.pose.orientation.w = 0.0;

    SolveFixedIK::Request request;
    request.target_poses = {unreachable_first_target, invalid_orientation_second_target};
    request.point_time_interval = 0.1;
    request.max_iterations = 1;

    const auto response = call_service<SolveFixedIK>(wrapper, "/solve_fixed_ik", request);
    ASSERT_NE(response, nullptr);
    EXPECT_TRUE(response->trajectory.points.empty());
    EXPECT_NE(response->message.find("point 1"), std::string::npos);
    EXPECT_NE(response->message.find("orientation quaternion is zero"), std::string::npos);
}
