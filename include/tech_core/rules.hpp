#pragma once

#include <array>
#include <cstddef>
#include <limits>
#include <string>
#include <string_view>

namespace tech_core {

enum class Stage { Moving, Ready, Inserted, Lifted, Flipped, Aligned, Confirmed, Failed };
std::string_view stage_name(Stage stage) noexcept;

// Coordinates are assembly mechanism joint displacements, in metres and radians.
// Positive insertion is motion along local -X; positive lift is local +Z.
struct Observation {
    double insertion = 0.0;
    double lift = 0.0;
    double flip = 0.0;
    double twist = 0.0;
    bool station_ready = true;
    bool card_present = true;
    bool robot_alive = true;
};

struct RulesConfig {
    // The manual specifies travel endpoints, but not sensor tolerances. These
    // three tolerances are explicit simulation assumptions, not official data.
    double linear_tolerance = 0.003;
    // 2 degrees at the P endpoint, also used as the neutral-Q tolerance while
    // completing steps 2/3/4. This enforces the ordered P-then-Q procedure.
    double flip_tolerance = 0.03490658503988659;
    double presence_threshold = 0.005;
    // The manual says "with 5-degree precision"; this simulation interprets
    // that as absolute target error <= 5 degrees (see docs/rules.md), not a
    // separately published sensor tolerance. The display delay is somewhere
    // in [0.3, 0.5] s; this simulation chooses 0.4 s by default.
    double twist_tolerance = 0.08726646259971647;
    double alignment_dwell = 0.4;
};

// One selected assembly attempt. Times are monotonically increasing simulation
// seconds. Referee economy, two-team arbitration, and manual recovery are outside
// this component. Early insertion requires full withdrawal before another try;
// this explicit simulator retry policy resolves an unspecified manual detail.
class AssemblyRules {
public:
    explicit AssemblyRules(int level, std::array<double, 2> target_rad = {},
                           double dwell = 0.4);
    AssemblyRules(int level, std::array<double, 2> target_rad, RulesConfig config);

    void update(double time, const std::array<Observation, 2>& observations);
    // Call update with current measured poses before confirm. Time advancement
    // here treats the last observation as held constant, as in a sampled sensor.
    bool confirm(std::size_t core, double time);

    Stage stage(std::size_t core) const;
    bool successful() const noexcept { return successful_; }
    bool failed() const noexcept { return failed_; }
    const std::string& failure_reason() const noexcept { return failure_reason_; }
    bool safety_locked(std::size_t core) const;
    bool insertion_blocked(std::size_t core) const;
    double confirm_available_at(std::size_t core) const;
    // Absolute earliest retry time after level-4 failure; -infinity otherwise.
    double retry_after() const noexcept { return retry_after_; }
    double arrival_time() const noexcept { return arrival_time_; }
    int level() const noexcept { return level_; }
    std::size_t core_count() const noexcept { return level_ == 4 ? 2 : 1; }
    double target(std::size_t core) const;

    // completed[0] represents difficulty 1. Difficulty 4 is one completion/game.
    static bool difficulty_available(int level, double match_seconds,
                                     const std::array<bool, 4>& completed) noexcept;

private:
    static constexpr double unset = std::numeric_limits<double>::quiet_NaN();
    struct CoreState {
        Stage stage = Stage::Moving;
        bool early_insertion = false;
        bool inserted_once = false;
        bool lifted_once = false;
        bool flipped_once = false;
        double aligned_since = unset;
        double card_absent_since = unset;
        double confirm_after = -std::numeric_limits<double>::infinity();
        // First completion timestamps for manual steps 2, 3, 5, and 6.
        std::array<double, 4> completed_at = {unset, unset, unset, unset};
    };

    bool inserted(const Observation& obs) const noexcept;
    bool lifted(const Observation& obs) const noexcept;
    bool flipped(const Observation& obs) const noexcept;
    bool correct_pose(std::size_t core) const noexcept;
    void check_core(std::size_t core) const;
    void fail(std::string reason, double time);
    void check_pair_deadlines(double time);

    int level_;
    std::array<double, 2> targets_;
    RulesConfig config_;
    std::array<CoreState, 2> cores_{};
    std::array<Observation, 2> observations_{};
    double last_time_ = unset;
    double arrival_time_ = unset;
    double retry_after_ = -std::numeric_limits<double>::infinity();
    bool failed_ = false;
    bool successful_ = false;
    std::string failure_reason_;
};

} // namespace tech_core
