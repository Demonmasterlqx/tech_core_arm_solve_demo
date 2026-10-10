// Copyright 2026 pnx
// Licensed under the MIT License.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "handle_arm/trajectory_loop_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<handle_arm::TrajectoryLoopNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
