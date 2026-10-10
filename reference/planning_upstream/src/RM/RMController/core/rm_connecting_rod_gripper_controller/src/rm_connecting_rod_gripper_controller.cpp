#include "rm_connecting_rod_gripper_controller/rm_connecting_rod_gripper_controller.hpp"

#include <utility>

namespace RM_hardware_interface
{

RMConnectingRodGripperController::RMConnectingRodGripperController()
: controller_interface::ChainableControllerInterface()
{
}

controller_interface::CallbackReturn RMConnectingRodGripperController::on_init()
{
    try {
        param_listener_ =
            std::make_shared<::rm_connecting_rod_gripper_controller::ParamListener>(get_node());
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to create parameter listener: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RMConnectingRodGripperController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    try {
        params_ = param_listener_->get_params();
        rm_connecting_rod_gripper_controller::validate_params(params_);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid parameters: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    kp_.store(params_.kp, std::memory_order_relaxed);
    ki_.store(params_.ki, std::memory_order_relaxed);
    kd_.store(params_.kd, std::memory_order_relaxed);
    max_output_.store(params_.max_output, std::memory_order_relaxed);
    max_integral_.store(params_.max_integral, std::memory_order_relaxed);
    static_friction_.store(params_.static_friction, std::memory_order_relaxed);

    core_ = std::make_unique<rm_connecting_rod_gripper_controller::ConnectingRodGripperCore>(
        rm_connecting_rod_gripper_controller::CoreConfig{
            params_.open_position,
            params_.close_position,
            params_.hold_effort,
            params_.position_threshold,
            params_.position_reach_frames});
    core_->reset_for_activation();

    open_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
        std::string(get_node()->get_name()) + "/open",
        1,
        std::bind(&RMConnectingRodGripperController::open_callback_, this, std::placeholders::_1));
    close_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
        std::string(get_node()->get_name()) + "/close",
        1,
        std::bind(&RMConnectingRodGripperController::close_callback_, this, std::placeholders::_1));
    toggle_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
        std::string(get_node()->get_name()) + "/toggle",
        1,
        std::bind(&RMConnectingRodGripperController::toggle_callback, this, std::placeholders::_1));

    kp_sub_ = get_node()->create_subscription<std_msgs::msg::Float32>(
        std::string(get_node()->get_name()) + "/pid/kp",
        1,
        std::bind(&RMConnectingRodGripperController::kp_callback_, this, std::placeholders::_1));
    ki_sub_ = get_node()->create_subscription<std_msgs::msg::Float32>(
        std::string(get_node()->get_name()) + "/pid/ki",
        1,
        std::bind(&RMConnectingRodGripperController::ki_callback_, this, std::placeholders::_1));
    kd_sub_ = get_node()->create_subscription<std_msgs::msg::Float32>(
        std::string(get_node()->get_name()) + "/pid/kd",
        1,
        std::bind(&RMConnectingRodGripperController::kd_callback_, this, std::placeholders::_1));
    max_output_sub_ = get_node()->create_subscription<std_msgs::msg::Float32>(
        std::string(get_node()->get_name()) + "/pid/max_output",
        1,
        std::bind(
            &RMConnectingRodGripperController::max_output_callback_, this, std::placeholders::_1));
    max_integral_sub_ = get_node()->create_subscription<std_msgs::msg::Float32>(
        std::string(get_node()->get_name()) + "/pid/max_integral",
        1,
        std::bind(
            &RMConnectingRodGripperController::max_integral_callback_, this,
            std::placeholders::_1));
    static_friction_sub_ = get_node()->create_subscription<std_msgs::msg::Float32>(
        std::string(get_node()->get_name()) + "/pid/static_friction",
        1,
        std::bind(
            &RMConnectingRodGripperController::static_friction_callback_, this,
            std::placeholders::_1));

    state_raw_publisher_ = get_node()->create_publisher<std_msgs::msg::Int8>(
        std::string(get_node()->get_name()) + "/state", 1);
    state_publisher_ = std::make_shared<RtPublisher<std_msgs::msg::Int8>>(state_raw_publisher_);

    if (params_.debug) {
        debug_publishers_.target_position = create_debug_publisher_("target_position");
        debug_publishers_.command_effort = create_debug_publisher_("command_effort");
        debug_publishers_.motor_position = create_debug_publisher_("motor_position");
        debug_publishers_.motor_velocity = create_debug_publisher_("motor_velocity");
        debug_publishers_.motor_effort = create_debug_publisher_("motor_effort");
        debug_publishers_.left_joint_position = create_debug_publisher_("left_joint_position");
        debug_publishers_.left_joint_velocity = create_debug_publisher_("left_joint_velocity");
        debug_publishers_.right_joint_position = create_debug_publisher_("right_joint_position");
        debug_publishers_.right_joint_velocity = create_debug_publisher_("right_joint_velocity");
        debug_publishers_.pid_kp = create_debug_publisher_("pid/kp");
        debug_publishers_.pid_ki = create_debug_publisher_("pid/ki");
        debug_publishers_.pid_kd = create_debug_publisher_("pid/kd");
        debug_publishers_.pid_max_output = create_debug_publisher_("pid/max_output");
        debug_publishers_.pid_max_integral = create_debug_publisher_("pid/max_integral");
        debug_publishers_.pid_static_friction = create_debug_publisher_("pid/static_friction");
        debug_publishers_.pid_error = create_debug_publisher_("pid/error");
        debug_publishers_.pid_p_term = create_debug_publisher_("pid/p_term");
        debug_publishers_.pid_i_term = create_debug_publisher_("pid/i_term");
        debug_publishers_.pid_d_term = create_debug_publisher_("pid/d_term");
        debug_publishers_.pid_friction_term = create_debug_publisher_("pid/friction_term");
        debug_publishers_.pid_output = create_debug_publisher_("pid/output");
        debug_publishers_.pid_integral_error = create_debug_publisher_("pid/integral_error");
    }

    // log no_pid_config is enabled or not
    RCLCPP_INFO(
        get_node()->get_logger(),
        "No PID config is %s",
        params_.no_pid_config.enable ? "enabled" : "disabled");

    RCLCPP_INFO(
        get_node()->get_logger(),
        "Configured rm_connecting_rod_gripper_controller for motor %s",
        params_.gripper_motor.c_str());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RMConnectingRodGripperController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (command_interfaces_.size() != 1U || state_interfaces_.size() != 3U) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Expected 1 command interface and 3 state interfaces, got %zu command / %zu state",
            command_interfaces_.size(),
            state_interfaces_.size());
        return controller_interface::CallbackReturn::ERROR;
    }

    core_->reset_for_activation();
    target_position_ = params_.open_position;
    command_effort_ = 0.0;
    pid_debug_ = {};

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RMConnectingRodGripperController::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    open_sub_.reset();
    close_sub_.reset();
    kp_sub_.reset();
    ki_sub_.reset();
    kd_sub_.reset();
    max_output_sub_.reset();
    max_integral_sub_.reset();
    static_friction_sub_.reset();
    state_publisher_.reset();
    state_raw_publisher_.reset();
    reset_debug_publishers_();
    core_.reset();
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RMConnectingRodGripperController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    command_effort_ = 0.0;
    if (!command_interfaces_.empty()) {
        (void)command_interfaces_[WRITE_MOTOR_EFFORT_INDEX].set_value(0.0);
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
RMConnectingRodGripperController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.push_back(params_.gripper_motor + "/effort");
    return config;
}

controller_interface::InterfaceConfiguration
RMConnectingRodGripperController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.push_back(params_.gripper_motor + "/position");
    config.names.push_back(params_.gripper_motor + "/velocity");
    config.names.push_back(params_.gripper_motor + "/effort");
    return config;
}

std::vector<hardware_interface::CommandInterface>
RMConnectingRodGripperController::on_export_reference_interfaces()
{
    return {};
}

std::vector<hardware_interface::StateInterface>
RMConnectingRodGripperController::on_export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.emplace_back(
        std::string(get_node()->get_name()) + "/" + params_.left_connecting_rod_joint,
        "position",
        &left_joint_position_);
    interfaces.emplace_back(
        std::string(get_node()->get_name()) + "/" + params_.left_connecting_rod_joint,
        "velocity",
        &left_joint_velocity_);
    interfaces.emplace_back(
        std::string(get_node()->get_name()) + "/" + params_.right_connecting_rod_joint,
        "position",
        &right_joint_position_);
    interfaces.emplace_back(
        std::string(get_node()->get_name()) + "/" + params_.right_connecting_rod_joint,
        "velocity",
        &right_joint_velocity_);
    return interfaces;
}

controller_interface::return_type
RMConnectingRodGripperController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/)
{
    return controller_interface::return_type::OK;
}

bool RMConnectingRodGripperController::on_set_chained_mode(bool chained_mode)
{
    return !chained_mode;
}

std::shared_ptr<RMConnectingRodGripperController::RtPublisher<std_msgs::msg::Float32>>
RMConnectingRodGripperController::create_debug_publisher_(const std::string & suffix)
{
    return std::make_shared<RtPublisher<std_msgs::msg::Float32>>(
        get_node()->create_publisher<std_msgs::msg::Float32>(
            std::string(get_node()->get_name()) + "/debug/" + suffix, 1));
}

void RMConnectingRodGripperController::reset_debug_publishers_()
{
    debug_publishers_ = {};
}

RMConnectingRodGripperController::PidConfig
RMConnectingRodGripperController::current_pid_config_() const
{
    return PidConfig{
        kp_.load(std::memory_order_relaxed),
        ki_.load(std::memory_order_relaxed),
        kd_.load(std::memory_order_relaxed),
        max_output_.load(std::memory_order_relaxed),
        max_integral_.load(std::memory_order_relaxed),
        static_friction_.load(std::memory_order_relaxed),
        deadband_.load(std::memory_order_relaxed),
        params_.no_pid_config.enable,
        params_.no_pid_config.open_effort,
        params_.no_pid_config.close_effort
    };
}

void RMConnectingRodGripperController::publish_float_(
    const std::shared_ptr<RtPublisher<std_msgs::msg::Float32>> & publisher,
    double value)
{
    if (publisher && publisher->trylock()) {
        publisher->msg_.data = static_cast<float>(value);
        publisher->unlockAndPublish();
    }
}

}  // namespace RM_hardware_interface
