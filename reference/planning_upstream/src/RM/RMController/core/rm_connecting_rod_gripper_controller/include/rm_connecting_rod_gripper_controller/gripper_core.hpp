#ifndef RM_CONNECTING_ROD_GRIPPER_CONTROLLER__GRIPPER_CORE_HPP_
#define RM_CONNECTING_ROD_GRIPPER_CONTROLLER__GRIPPER_CORE_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "rm_connecting_rod_gripper_controller/rm_connecting_rod_gripper_controller_parameters.hpp"

namespace RM_hardware_interface::rm_connecting_rod_gripper_controller
{

enum class GripperState : int8_t
{
    OPENED = 0,
    OPENING = 1,
    CLOSED = 2,
    CLOSING = 3
};

struct PidConfig
{
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double max_output = 0.0;
    double max_integral = 0.0;
    double static_friction = 0.0;
    double deadband = 0.0;
    bool no_pid_control = false;
    double open_effort = 0.0;
    double close_effort = 0.0;
};

struct PidDebugData
{
    double error = 0.0;
    double p_term = 0.0;
    double i_term = 0.0;
    double d_term = 0.0;
    double friction_term = 0.0;
    double output = 0.0;
    double integral_error = 0.0;
};

struct CoreConfig
{
    double open_position = 0.0;
    double close_position = 0.0;
    double hold_effort = 0.0;
    double position_threshold = 0.0;
    int64_t position_reach_frames = 1;
};

struct CoreUpdateResult
{
    GripperState state = GripperState::OPENING;
    double target_position = 0.0;
    double command_effort = 0.0;
    PidDebugData pid_debug;
};

class PositionPid
{
public:
    void reset()
    {
        integral_error_ = 0.0;
        debug_ = {};
    }

    CoreUpdateResult compute(
        const double target_position,
        const double current_position,
        const double measured_velocity,
        const double dt,
        const PidConfig & config)
    {
        const double error = std::abs(target_position - current_position) < config.deadband ? 0.0 : (target_position - current_position);
        const double safe_dt = std::max(dt, 1e-9);

        integral_error_ += error * safe_dt;
        integral_error_ = std::clamp(integral_error_, -config.max_integral, config.max_integral);

        const double p_term = config.kp * error;
        const double i_term = config.ki * integral_error_;
        const double d_term = -config.kd * measured_velocity;
        const double friction_term = (std::abs(error) >
            0.0) ? config.static_friction * sign(error) : 0.0;
        const double raw_output = p_term + i_term + d_term + friction_term;

        debug_.error = error;
        debug_.p_term = p_term;
        debug_.i_term = i_term;
        debug_.d_term = d_term;
        debug_.friction_term = friction_term;
        debug_.integral_error = integral_error_;
        debug_.output = std::clamp(raw_output, -config.max_output, config.max_output);

        CoreUpdateResult result;
        result.pid_debug = debug_;
        result.command_effort = debug_.output;
        result.target_position = target_position;
        return result;
    }

    const PidDebugData & debug_data() const
    {
        return debug_;
    }

private:
    static double sign(const double value)
    {
        if (value > 0.0) {
            return 1.0;
        }
        if (value < 0.0) {
            return -1.0;
        }
        return 0.0;
    }

    double integral_error_ = 0.0;
    PidDebugData debug_{};
};

class ConnectingRodGripperCore
{
public:
    explicit ConnectingRodGripperCore(const CoreConfig & config)
    : config_(config)
    {
    }

    void reset_for_activation()
    {
        pid_.reset();
        state_ = GripperState::OPENING;
        pending_pid_reset_ = false;
        threshold_frame_counter_ = 0;
    }

    void command_open()
    {
        state_ = GripperState::OPENING;
        pending_pid_reset_ = true;
        threshold_frame_counter_ = 0;
    }

    void command_close()
    {
        state_ = GripperState::CLOSING;
        pending_pid_reset_ = true;
        threshold_frame_counter_ = 0;
    }

    CoreUpdateResult pid_control(
        const double current_position,
        const double current_velocity,
        const double dt,
        const PidConfig & pid_config
    ){
        if (pending_pid_reset_) {
            pid_.reset();
            pending_pid_reset_ = false;
            threshold_frame_counter_ = 0;
        }

        const double target_position =
            (state_ == GripperState::OPENING || state_ == GripperState::OPENED) ?
            config_.open_position : config_.close_position;

        if (state_ == GripperState::OPENING) {
            const bool reached =
                std::abs(current_position - config_.open_position) <= config_.position_threshold;
            threshold_frame_counter_ = reached ? (threshold_frame_counter_ + 1) : 0;
            if (threshold_frame_counter_ >= config_.position_reach_frames) {
                state_ = GripperState::OPENED;
                pid_.reset();
                threshold_frame_counter_ = 0;
            }
        } else if (state_ == GripperState::CLOSING) {
            const bool reached =
                std::abs(current_position - config_.close_position) <= config_.position_threshold;
            threshold_frame_counter_ = reached ? (threshold_frame_counter_ + 1) : 0;
            if (threshold_frame_counter_ >= config_.position_reach_frames) {
                state_ = GripperState::CLOSED;
                pid_.reset();
                threshold_frame_counter_ = 0;
            }
        } else {
            threshold_frame_counter_ = 0;
        }

        CoreUpdateResult result;
        result.state = state_;
        result.target_position = target_position;

        switch (state_) {
            case GripperState::OPENED:
                result.command_effort = 0.0;
                result.pid_debug = {};
                break;
            case GripperState::CLOSED:
                result.command_effort = config_.hold_effort;
                result.pid_debug = {};
                break;
            case GripperState::OPENING:
            case GripperState::CLOSING:
            default:
                result = pid_.compute(target_position, current_position, current_velocity, dt, pid_config);
                result.state = state_;
                result.target_position = target_position;
                break;
        }

        return result;
    }

    CoreUpdateResult no_pid_control(
        const double current_position,
        const double current_velocity,
        const double dt,
        const PidConfig & pid_config
    ){

        (void)dt;  // unused
        (void)current_velocity;  // unused

        double command_effort = 0.0;
        double target_position = [this]() {
            if(state_ == GripperState::OPENING || state_ == GripperState::OPENED){
                return config_.open_position;
            }
            else{
                return config_.close_position;
            }
        }();
        CoreUpdateResult result{};

        // state switching logic
        if(state_ == GripperState::OPENING){
            command_effort = pid_config.open_effort;
            if(std::abs(current_position - config_.open_position) <= config_.position_threshold){
                state_ = GripperState::OPENED;
            }
        }
        else if(state_ == GripperState::OPENED){
            command_effort = 0.0;
            if(std::abs(current_position - config_.open_position) >= config_.position_threshold){
                state_ = GripperState::OPENING;
            }
        }
        else if(state_ == GripperState::CLOSING){
            command_effort = pid_config.close_effort;
            if(current_position >= config_.close_position - config_.position_threshold){
                state_ = GripperState::CLOSED;
            }
        }
        else if(state_ == GripperState::CLOSED){

            command_effort = pid_config.close_effort;
            if(current_position <= config_.close_position - config_.position_threshold){
                state_ = GripperState::CLOSING;
            }
        }

        result.command_effort = command_effort;
        result.state = state_;
        result.target_position = target_position;
        return result;

    }

    CoreUpdateResult update(
        const double current_position,
        const double current_velocity,
        const double dt,
        const PidConfig & pid_config)
    {

        CoreUpdateResult result;

        if(!pid_config.no_pid_control){
            result = pid_control(current_position, current_velocity, dt, pid_config);
        }
        else{
            result = no_pid_control(current_position, current_velocity, dt, pid_config);
        }

        return result;
    }

    GripperState state() const
    {
        return state_;
    }

private:
    CoreConfig config_;
    GripperState state_ = GripperState::OPENING;
    bool pending_pid_reset_ = false;
    int64_t threshold_frame_counter_ = 0;
    PositionPid pid_{};
};

inline void validate_params(const ::rm_connecting_rod_gripper_controller::Params & params)
{
    auto require_non_empty = [](const std::string & value, const char * name) {
            if (value.empty()) {
                throw std::invalid_argument(std::string(name) + " must not be empty");
            }
        };

    require_non_empty(params.gripper_motor, "gripper_motor");
    require_non_empty(params.left_connecting_rod_joint, "left_connecting_rod_joint");
    require_non_empty(params.right_connecting_rod_joint, "right_connecting_rod_joint");

    if (!(params.hold_effort > 0.0) || !std::isfinite(params.hold_effort)) {
        throw std::invalid_argument("hold_effort must be finite and positive");
    }
    if (!(params.position_threshold > 0.0) || !std::isfinite(params.position_threshold)) {
        throw std::invalid_argument("position_threshold must be finite and positive");
    }
    if (params.position_reach_frames < 1) {
        throw std::invalid_argument("position_reach_frames must be greater than or equal to 1");
    }
    if (!(params.max_output > 0.0) || !std::isfinite(params.max_output)) {
        throw std::invalid_argument("max_output must be finite and positive");
    }
    if (params.max_integral < 0.0 || !std::isfinite(params.max_integral)) {
        throw std::invalid_argument("max_integral must be finite and non-negative");
    }
    if (!(params.close_position > params.open_position) ||
        !std::isfinite(params.open_position) || !std::isfinite(params.close_position))
    {
        throw std::invalid_argument("close_position must be finite and greater than open_position");
    }
    if (!std::isfinite(params.kp) || !std::isfinite(params.ki) || !std::isfinite(params.kd) ||
        !std::isfinite(params.static_friction))
    {
        throw std::invalid_argument("PID parameters must be finite");
    }
}

}  // namespace RM_hardware_interface::rm_connecting_rod_gripper_controller

#endif  // RM_CONNECTING_ROD_GRIPPER_CONTROLLER__GRIPPER_CORE_HPP_
