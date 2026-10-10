#include "rm_connecting_rod_gripper_controller/rm_connecting_rod_gripper_controller.hpp"

namespace RM_hardware_interface
{

bool RMConnectingRodGripperController::read_motor_state_()
{
    const auto motor_position = state_interfaces_[READ_MOTOR_POSITION_INDEX].get_optional();
    const auto motor_velocity = state_interfaces_[READ_MOTOR_VELOCITY_INDEX].get_optional();
    const auto motor_effort = state_interfaces_[READ_MOTOR_EFFORT_INDEX].get_optional();

    if (!motor_position.has_value() || !motor_velocity.has_value() || !motor_effort.has_value()) {
        RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(),
            *get_node()->get_clock(),
            1000,
            "Failed to read motor state interfaces");
        return false;
    }

    motor_position_ = motor_position.value();
    motor_velocity_ = motor_velocity.value();
    motor_effort_ = motor_effort.value();
    return true;
}

void RMConnectingRodGripperController::update_virtual_state_()
{
    left_joint_position_ = motor_position_;
    left_joint_velocity_ = motor_velocity_;
    right_joint_position_ = motor_position_;
    right_joint_velocity_ = motor_velocity_;
}

controller_interface::return_type RMConnectingRodGripperController::update_and_write_commands(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & period)
{
    if (!read_motor_state_()) {
        return controller_interface::return_type::OK;
    }

    update_virtual_state_();

    const auto result =
        core_->update(motor_position_, motor_velocity_, period.seconds(), current_pid_config_());

    target_position_ = result.target_position;
    command_effort_ = result.command_effort;
    pid_debug_ = result.pid_debug;

    if (!command_interfaces_[WRITE_MOTOR_EFFORT_INDEX].set_value(command_effort_)) {
        RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(),
            *get_node()->get_clock(),
            1000,
            "Failed to write gripper motor effort command");
    }

    publish_state_();
    if (params_.debug) {
        publish_debug_();
    }

    return controller_interface::return_type::OK;
}

void RMConnectingRodGripperController::publish_state_()
{
    if (state_publisher_ && state_publisher_->trylock()) {
        state_publisher_->msg_.data = static_cast<int8_t>(core_->state());
        state_publisher_->unlockAndPublish();
    }
}

void RMConnectingRodGripperController::publish_debug_()
{
    publish_float_(debug_publishers_.target_position, target_position_);
    publish_float_(debug_publishers_.command_effort, command_effort_);
    publish_float_(debug_publishers_.motor_position, motor_position_);
    publish_float_(debug_publishers_.motor_velocity, motor_velocity_);
    publish_float_(debug_publishers_.motor_effort, motor_effort_);
    publish_float_(debug_publishers_.left_joint_position, left_joint_position_);
    publish_float_(debug_publishers_.left_joint_velocity, left_joint_velocity_);
    publish_float_(debug_publishers_.right_joint_position, right_joint_position_);
    publish_float_(debug_publishers_.right_joint_velocity, right_joint_velocity_);

    publish_float_(debug_publishers_.pid_kp, kp_.load(std::memory_order_relaxed));
    publish_float_(debug_publishers_.pid_ki, ki_.load(std::memory_order_relaxed));
    publish_float_(debug_publishers_.pid_kd, kd_.load(std::memory_order_relaxed));
    publish_float_(debug_publishers_.pid_max_output, max_output_.load(std::memory_order_relaxed));
    publish_float_(
        debug_publishers_.pid_max_integral, max_integral_.load(std::memory_order_relaxed));
    publish_float_(
        debug_publishers_.pid_static_friction, static_friction_.load(std::memory_order_relaxed));
    publish_float_(debug_publishers_.pid_error, pid_debug_.error);
    publish_float_(debug_publishers_.pid_p_term, pid_debug_.p_term);
    publish_float_(debug_publishers_.pid_i_term, pid_debug_.i_term);
    publish_float_(debug_publishers_.pid_d_term, pid_debug_.d_term);
    publish_float_(debug_publishers_.pid_friction_term, pid_debug_.friction_term);
    publish_float_(debug_publishers_.pid_output, pid_debug_.output);
    publish_float_(debug_publishers_.pid_integral_error, pid_debug_.integral_error);
}

}  // namespace RM_hardware_interface
