#pragma once

#include "tech_core/planning.hpp"
#include <cstdint>
#include <future>

namespace tech_core::planning {

enum class DemoState { Editing, Planning, PreviewReady, PreviewFailed, Executing, Paused, Complete };
const char* demo_state_name(DemoState state);

struct DemoPreview {
    PlanResult result;
    // Desired Cartesian task and actual FK of the returned candidate are
    // deliberately separate: an unsuccessful planner may return zero knots.
    std::vector<Eigen::Vector3d> requested_path;
    std::vector<Eigen::Vector3d> candidate_path;
    // A visualization-only copy, retimed even when a failed IK prefix has no
    // timestamps. A zero-knot rejection holds one ghost at the requested start.
    // Execution MUST continue to use result.knots and the success gate.
    std::vector<Knot> ghost_knots;
};
using DemoPlanner = std::function<DemoPreview(const Scenario&, const PlanOptions&)>;
PlanOptions demo_plan_options();
DemoPreview make_demo_preview(const Scenario& request, const PlanOptions& options);
Knot sample_demo_trajectory(const std::vector<Knot>& knots, double seconds);

// Owns the displayed state. The asynchronous planner receives value snapshots
// and owns a different MuJoCo model/data; previewing never advances this scene.
class PlanningDemo {
public:
    explicit PlanningDemo(Scenario initial_scenario = scenario_suite().front(),
                          PlanOptions options = demo_plan_options(),
                          DemoPlanner planner = make_demo_preview);
    ~PlanningDemo();
    PlanningDemo(const PlanningDemo&) = delete;
    PlanningDemo& operator=(const PlanningDemo&) = delete;

    void enter();
    void tick(double seconds);
    void home();                      // Seven zero joint angles + mechanism reset; preserve core pose/backend.
    void reset();                     // Entire initial scene; preserve backend.
    void toggle_pause();
    void focus_lost();
    void set_backend(Method method);
    void jog_arm(const Eigen::Vector3d& world_velocity, double seconds);
    void stop_jog();
    void jog_core(const Eigen::Vector3d& world_velocity,
                  const Eigen::Vector3d& rpy_degrees_per_second, double seconds);

    DemoState state() const { return state_; }
    const Scenario& scenario() const { return scenario_; }
    const JointVector& joints() const { return q_; }
    const MechanismState& mechanism() const { return mechanism_; }
    const PlanOptions& options() const { return options_; }
    const std::optional<DemoPreview>& preview() const { return preview_; }
    const std::string& message() const { return message_; }
    std::uint64_t revision() const { return revision_; }
    double execution_time() const { return execution_time_; }
    bool worker_pending() const { return worker_.valid(); }
    Eigen::Vector3d tool_position() const;
    Eigen::Vector3d core_position() const;
    PlanningScene& scene() { return scene_; }

private:
    bool editing_allowed();
    void invalidate(const std::string& message);
    void display_state();
    void poll();
    void advance(double seconds);

    PlanningScene scene_;
    Scenario initial_, scenario_;
    PlanOptions options_;
    DemoPlanner planner_;
    JointVector q_;
    MechanismState mechanism_;
    DemoState state_ = DemoState::Editing;
    std::string message_ = "Enter: plan and preview";
    std::optional<DemoPreview> preview_;
    std::optional<Eigen::Matrix3d> jog_orientation_;
    std::future<DemoPreview> worker_;
    std::uint64_t revision_ = 0, worker_revision_ = 0;
    double execution_time_ = 0;
    int tool_site_ = -1;
};

} // namespace tech_core::planning
