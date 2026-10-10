#include "tech_core/rules.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace tech_core {
namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double epsilon = 1e-9;
bool close_to(double value, double target, double tolerance) noexcept {
    return std::abs(value - target) <= tolerance + epsilon;
}
RulesConfig with_dwell(double dwell) {
    RulesConfig config;
    config.alignment_dwell = dwell;
    return config;
}
} // namespace

std::string_view stage_name(Stage stage) noexcept {
    switch (stage) {
    case Stage::Moving: return "moving";
    case Stage::Ready: return "ready";
    case Stage::Inserted: return "inserted";
    case Stage::Lifted: return "lifted";
    case Stage::Flipped: return "flipped";
    case Stage::Aligned: return "aligned";
    case Stage::Confirmed: return "confirmed";
    case Stage::Failed: return "failed";
    }
    return "unknown";
}

AssemblyRules::AssemblyRules(int level, std::array<double, 2> target_rad, double dwell)
    : AssemblyRules(level, target_rad, with_dwell(dwell)) {}

AssemblyRules::AssemblyRules(int level, std::array<double, 2> target_rad, RulesConfig config)
    : level_(level), targets_(target_rad), config_(config) {
    if (level < 1 || level > 4) throw std::invalid_argument("difficulty must be 1..4");
    for (std::size_t i = 0; i < core_count(); ++i) {
        if (!std::isfinite(targets_[i]) || std::abs(targets_[i]) > pi / 2 + epsilon)
            throw std::invalid_argument("Q target must be within +/- 90 degrees");
    }
    if (!std::isfinite(config.alignment_dwell) || config.alignment_dwell < 0.3 ||
        config.alignment_dwell > 0.5 || !std::isfinite(config.linear_tolerance) ||
        config.linear_tolerance <= 0 || config.linear_tolerance >= 0.05 ||
        !std::isfinite(config.flip_tolerance) || config.flip_tolerance <= 0 ||
        config.flip_tolerance >= pi / 4 || !std::isfinite(config.presence_threshold) ||
        config.presence_threshold < 0 || config.presence_threshold >= 0.05 ||
        !std::isfinite(config.twist_tolerance) || config.twist_tolerance <= 0 ||
        config.twist_tolerance > 5 * pi / 180 + epsilon)
        throw std::invalid_argument("invalid rule tolerances or display dwell");
}

void AssemblyRules::check_core(std::size_t core) const {
    if (core >= core_count()) throw std::out_of_range("inactive technology core");
}

Stage AssemblyRules::stage(std::size_t core) const { check_core(core); return cores_[core].stage; }
bool AssemblyRules::insertion_blocked(std::size_t core) const {
    check_core(core); return cores_[core].early_insertion;
}
double AssemblyRules::confirm_available_at(std::size_t core) const {
    check_core(core); return cores_[core].confirm_after;
}
double AssemblyRules::target(std::size_t core) const { check_core(core); return targets_[core]; }

bool AssemblyRules::inserted(const Observation& obs) const noexcept {
    return close_to(obs.insertion, 0.1, config_.linear_tolerance);
}
bool AssemblyRules::lifted(const Observation& obs) const noexcept {
    return close_to(obs.lift, 0.1, config_.linear_tolerance);
}
bool AssemblyRules::flipped(const Observation& obs) const noexcept {
    return close_to(obs.flip, pi / 2, config_.flip_tolerance);
}
bool AssemblyRules::correct_pose(std::size_t core) const noexcept {
    const auto& obs = observations_[core];
    return obs.station_ready && inserted(obs) && (level_ < 2 || lifted(obs)) &&
           (level_ < 3 || (flipped(obs) &&
             close_to(obs.twist, targets_[core], config_.twist_tolerance)));
}

void AssemblyRules::fail(std::string reason, double time) {
    if (failed_ || successful_) return;
    failed_ = true;
    failure_reason_ = std::move(reason);
    if (level_ == 4) retry_after_ = time + 90.0;
    for (std::size_t i = 0; i < core_count(); ++i) cores_[i].stage = Stage::Failed;
}

void AssemblyRules::check_pair_deadlines(double time) {
    if (level_ != 4 || failed_ || successful_) return;
    constexpr std::array<int, 4> steps = {2, 3, 5, 6};
    for (std::size_t step = 0; step < steps.size(); ++step) {
        const double a = cores_[0].completed_at[step];
        const double b = cores_[1].completed_at[step];
        if ((std::isfinite(a) && std::isfinite(b) && std::abs(a - b) > 5 + epsilon) ||
            (std::isfinite(a) && !std::isfinite(b) && time - a > 5 + epsilon) ||
            (std::isfinite(b) && !std::isfinite(a) && time - b > 5 + epsilon)) {
            fail("level 4 pair exceeded 5 seconds at step " + std::to_string(steps[step]), time);
            return;
        }
    }
}

void AssemblyRules::update(double time, const std::array<Observation, 2>& observations) {
    if (!std::isfinite(time) || (std::isfinite(last_time_) && time < last_time_))
        throw std::invalid_argument("simulation time must be finite and monotonic");
    for (std::size_t i = 0; i < core_count(); ++i) {
        const auto& obs = observations[i];
        if (!std::isfinite(obs.insertion) || !std::isfinite(obs.lift) ||
            !std::isfinite(obs.flip) || !std::isfinite(obs.twist))
            throw std::invalid_argument("joint observations must be finite");
    }
    last_time_ = time;
    observations_ = observations;
    if (failed_ || successful_) return;

    bool all_ready = true;
    for (std::size_t i = 0; i < core_count(); ++i) all_ready &= observations[i].station_ready;
    if (all_ready && !std::isfinite(arrival_time_)) arrival_time_ = time;
    if (level_ == 4 && std::isfinite(arrival_time_) && time - arrival_time_ > 45 + epsilon) {
        fail("level 4 exceeded 45 seconds after arrival", time);
        return;
    }

    for (std::size_t i = 0; i < core_count(); ++i) {
        auto& state = cores_[i];
        const auto& obs = observations[i];
        if (!obs.robot_alive) { fail("engineer robot died during assembly", time); return; }
        if (obs.card_present) state.card_absent_since = unset;
        else if (!std::isfinite(state.card_absent_since)) state.card_absent_since = time;
        if (std::isfinite(state.card_absent_since) &&
            time - state.card_absent_since >= 15 - epsilon) {
            fail("assembly interaction card absent for 15 seconds", time); return;
        }
        if (state.inserted_once && obs.insertion <= config_.presence_threshold) {
            fail("energy unit removed after insertion", time); return;
        }
        if (state.stage == Stage::Confirmed) {
            if (!correct_pose(i)) { fail("confirmed energy unit moved out of its required pose", time); return; }
            continue;
        }
        if (!obs.station_ready) {
            if (obs.insertion > config_.presence_threshold && !state.inserted_once)
                state.early_insertion = true;
            state.aligned_since = unset;
            if (!state.inserted_once) state.stage = Stage::Moving;
            else if (state.stage == Stage::Aligned) state.stage = Stage::Flipped;
            continue;
        }
        if (state.stage == Stage::Moving) state.stage = Stage::Ready;
        if (state.early_insertion) {
            if (obs.insertion <= config_.presence_threshold) state.early_insertion = false;
            else continue;
        }

        // Each physical operation must follow its predecessor. One sample may
        // complete at most one physical operation; dwell is sampled separately.
        if (!state.inserted_once) {
            if (inserted(obs) && close_to(obs.lift, 0, config_.linear_tolerance) &&
                close_to(obs.flip, 0, config_.flip_tolerance) &&
                (level_ < 3 || close_to(obs.twist, 0, config_.flip_tolerance))) {
                state.inserted_once = true;
                state.stage = Stage::Inserted;
                state.completed_at[0] = time;
            }
        } else if (level_ >= 2 && !state.lifted_once) {
            if (inserted(obs) && lifted(obs) && close_to(obs.flip, 0, config_.flip_tolerance) &&
                (level_ < 3 || close_to(obs.twist, 0, config_.flip_tolerance))) {
                state.lifted_once = true;
                state.stage = Stage::Lifted;
                state.completed_at[1] = time;
            }
        } else if (level_ >= 3 && !state.flipped_once) {
            if (inserted(obs) && lifted(obs) && flipped(obs) &&
                close_to(obs.twist, 0, config_.flip_tolerance)) {
                state.flipped_once = true;
                state.stage = Stage::Flipped;
            }
        }

        if (level_ >= 3 && state.flipped_once) {
            if (correct_pose(i)) {
                if (!std::isfinite(state.aligned_since)) state.aligned_since = time;
                if (time - state.aligned_since >= config_.alignment_dwell - epsilon) {
                    state.stage = Stage::Aligned;
                    if (!std::isfinite(state.completed_at[2])) state.completed_at[2] = time;
                }
            } else {
                state.aligned_since = unset;
                state.stage = Stage::Flipped;
            }
        }
    }
    check_pair_deadlines(time);
}

bool AssemblyRules::confirm(std::size_t core, double time) {
    check_core(core);
    if (!std::isfinite(last_time_)) throw std::logic_error("update observations before confirmation");
    update(time, observations_);
    if (failed_ || successful_) return false;
    auto& state = cores_[core];
    if (state.stage == Stage::Confirmed || time + epsilon < state.confirm_after) return false;
    const Stage required = level_ == 1 ? Stage::Inserted : level_ == 2 ? Stage::Lifted : Stage::Aligned;
    if (state.stage != required || !correct_pose(core)) {
        state.confirm_after = time + 1.0;
        return false;
    }
    state.stage = Stage::Confirmed;
    state.completed_at[3] = time;
    check_pair_deadlines(time);
    if (failed_) return false;
    bool all_confirmed = true;
    for (std::size_t i = 0; i < core_count(); ++i)
        all_confirmed &= cores_[i].stage == Stage::Confirmed && correct_pose(i);
    successful_ = all_confirmed;
    return true;
}

bool AssemblyRules::safety_locked(std::size_t core) const {
    check_core(core);
    return !failed_ && level_ >= 3 && correct_pose(core) &&
           std::isfinite(cores_[core].aligned_since) &&
           last_time_ - cores_[core].aligned_since > 2 + epsilon;
}

bool AssemblyRules::difficulty_available(int level, double match_seconds,
                                       const std::array<bool, 4>& completed) noexcept {
    if (level < 1 || level > 4 || !std::isfinite(match_seconds) || match_seconds < 0)
        return false;
    return match_seconds >= 60.0 * (level - 1) &&
           (level == 1 || completed[static_cast<std::size_t>(level - 2)]) &&
           (level != 4 || !completed[3]);
}

} // namespace tech_core
