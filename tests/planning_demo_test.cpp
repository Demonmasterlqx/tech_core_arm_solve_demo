#include "tech_core/planning_demo.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace tech_core;
using namespace tech_core::planning;

namespace {
void require(bool ok, const std::string& message) {
    if (!ok) throw std::runtime_error(message);
}
void finish_request(PlanningDemo& demo) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (demo.worker_pending() && std::chrono::steady_clock::now() < end) {
        demo.tick(.01);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(!demo.worker_pending(), "Planner did not finish");
}
bool same(const JointVector& a, const JointVector& b) { return (a - b).norm() < 1e-12; }

void two_stage_contract(const DemoPreview& solved) {
    PlanningDemo demo(scenario_suite().front(), demo_plan_options(),
        [solved](const Scenario&, const PlanOptions&) { return solved; });
    const auto initial = demo.joints();
    const auto tcp = demo.tool_position();
    demo.enter();
    demo.enter();
    require(demo.state() == DemoState::Planning, "An extra Enter during planning must not queue execution");
    finish_request(demo);
    require(demo.state() == DemoState::PreviewReady, "Successful plan must stop at preview");
    for (int i = 0; i < 100; ++i) demo.tick(.05);
    require(same(initial, demo.joints()) && (demo.tool_position() - tcp).norm() < 1e-12,
            "Planning/preview mutated displayed robot state");
    require(demo.execution_time() == 0 && demo.mechanism().lift == 0, "Preview advanced mechanism or execution clock");
    PlanningScene ghost;
    ghost.configure(demo.scenario());
    for (int i = 0; i <= 10; ++i) {
        const auto frame = sample_demo_trajectory(solved.ghost_knots, solved.ghost_knots.back().time * i / 10.);
        ghost.set_state(frame.q, frame.mechanism);
        mj_forward(ghost.model(), ghost.data());
    }
    require(same(initial, demo.joints()) && (demo.tool_position() - tcp).norm() < 1e-12 &&
            demo.mechanism().lift == 0, "Ghost playback changed the opaque robot/mechanism");

    demo.enter();
    require(demo.state() == DemoState::Executing && same(initial, demo.joints()),
            "Second Enter should arm execution without teleporting");
    // Compare the actual displayed joints to the planner's cubic contract at
    // an interior time, not just its endpoint.
    demo.tick(.05);
    const auto& knots = solved.result.knots;
    const auto it = std::lower_bound(knots.begin(), knots.end(), .05,
        [](const Knot& k, double t) { return k.time < t; });
    require(it != knots.begin() && it != knots.end(), "Need an interior interpolation sample");
    const auto& a = *(it - 1);
    const double u = (.05 - a.time) / (it->time - a.time);
    const double s = u * u * (3 - 2 * u);
    require((demo.joints() - (a.q + s * (it->q - a.q))).norm() < 1e-12, "Execution differs from validated cubic interpolation");
    require(!same(initial, demo.joints()), "Execution must move after second Enter");
    demo.focus_lost();
    require(demo.state() == DemoState::Paused, "Focus loss must pause execution");
    const auto paused = demo.joints();
    const double paused_time = demo.execution_time();
    demo.enter();
    for (int i = 0; i < 10; ++i) demo.tick(.05);
    require(same(paused, demo.joints()) && paused_time == demo.execution_time(), "Pause failed to hold");
    demo.toggle_pause();
    for (int i = 0; i < 2000 && demo.state() != DemoState::Complete; ++i) demo.tick(.05);
    require(demo.state() == DemoState::Complete && same(demo.joints(), knots.back().q), "Execution did not reach exact final knot");
    require(std::abs(demo.mechanism().twist - radians(35)) < 1e-12, "Core mechanism not synchronized with arm");
    demo.home();
    require(demo.state() == DemoState::Editing && same(initial, demo.joints()) && !demo.preview(), "Home did not reset arm and invalidate preview");
    require(demo.mechanism().flip == 0 && demo.execution_time() == 0, "Home did not reset assembly");
    std::cout << "PASS preview hold, double Enter, interpolation, focus pause, completion and home\n";
}

void invalidation_contract(const DemoPreview& solved) {
    PlanningDemo demo(scenario_suite().front(), demo_plan_options(),
        [solved](const Scenario&, const PlanOptions&) { return solved; });
    demo.enter(); finish_request(demo);
    demo.set_backend(Method::Baseline);
    require(demo.state() == DemoState::Editing && !demo.preview(), "Backend must invalidate a ready preview");
    demo.enter(); finish_request(demo);
    demo.jog_arm({.02, 0, 0}, .05);
    require(demo.state() == DemoState::Editing && !demo.preview(), "Arm jog must invalidate a ready preview");
    demo.home(); demo.enter(); finish_request(demo);
    demo.jog_core({-.01, 0, 0}, {0, 0, 0}, .05);
    require(demo.state() == DemoState::Editing && !demo.preview(), "Core jog must invalidate a ready preview");
    const auto core = demo.core_position();
    demo.home();
    require((core - demo.core_position()).norm() < 1e-12, "Backspace must preserve edited core pose");

    // Hold a worker across a reset, then release its old successful answer.
    // The gate is released before any assertion/destructor can wait on it.
    std::promise<void> gate;
    auto ready = gate.get_future().share();
    PlanningDemo stale(scenario_suite().front(), demo_plan_options(),
        [solved, ready](const Scenario&, const PlanOptions&) { ready.wait(); return solved; });
    stale.enter();
    stale.jog_core({-.01, 0, 0}, {0, -5, 0}, .05);
    stale.home();
    gate.set_value();
    finish_request(stale);
    require(stale.state() == DemoState::Editing && !stale.preview(), "Stale asynchronous result resurrected after reset");
    require(stale.execution_time() == 0, "Stale result executed");
    std::cout << "PASS arm/core/backend invalidation and stale worker disposal\n";
}

void jogging_contract() {
    PlanningDemo demo;
    const auto initial = demo.joints();
    for (int axis = 0; axis < 3; ++axis) for (int sign : {-1, 1}) {
        demo.home();
        const auto before = demo.tool_position();
        for (int i = 0; i < 20; ++i) demo.jog_arm(.04 * sign * Eigen::Vector3d::Unit(axis), .01);
        const auto delta = (demo.tool_position() - before).eval();
        require(delta[axis] * sign > .004, "Cartesian jog failed axis/sign " + std::to_string(axis) + "/" + std::to_string(sign));
        for (int other = 0; other < 3; ++other) if (other != axis)
            require(std::abs(delta[other]) < .002, "Cartesian jog drifted on another axis");
        demo.stop_jog();
        const auto released = demo.joints();
        demo.tick(.1);
        require(same(released, demo.joints()), "Jog continued after release");
    }
    demo.home();
    require(same(initial, demo.joints()), "Arm home differs after jogging");
    const auto before = demo.core_position();
    demo.jog_core({-.04, .04, .04}, {10, -10, 10}, .1);
    const auto delta = (demo.core_position() - before).eval();
    require(delta.x() < -.002 && delta.y() > .002 && delta.z() > .002, "Core world axes differ from arm world axes");
    require(demo.scenario().core.alpha == 1 && demo.scenario().core.phi == 89 && demo.scenario().core.theta == 1,
            "Core RPY controls use wrong parameter mapping");
    for (int i = 0; i < 100; ++i) demo.jog_core({.1, .1, .1}, {30, 30, 30}, .1);
    validate_pose(demo.scenario().core, 3);
    std::cout << "PASS six arm directions, release hold, core world XYZ and RPY limits\n";
}

void failure_contract() {
    auto request = scenario_suite().front();
    request.blocking_obstacle = true;
    const auto failed = make_demo_preview(request, demo_plan_options());
    require(!failed.result.success && !failed.requested_path.empty(), "Rejected plan lost its desired task visualization");
    PlanningDemo demo(request, demo_plan_options(), [failed](const Scenario&, const PlanOptions&) { return failed; });
    const auto initial = demo.joints();
    demo.enter(); finish_request(demo);
    require(demo.state() == DemoState::PreviewFailed && demo.preview(), "Failed plan was not retained for display");
    demo.enter(); demo.tick(.1);
    require(demo.state() == DemoState::PreviewFailed && same(initial, demo.joints()), "Failed trajectory executed");

    // Front-plane rejection produces no knots at all; still show intent.
    const auto suite = scenario_suite();
    request = *std::find_if(suite.begin(), suite.end(), [](const auto& s) { return s.id == "reject_front_plane"; });
    const auto empty = make_demo_preview(request, demo_plan_options());
    require(!empty.result.success && empty.result.knots.empty() && !empty.requested_path.empty(), "Zero-knot failure lacks a preview");
    require(empty.ghost_knots.size() == 1 && same(empty.ghost_knots.front().q, request.start),
            "Zero-knot failure must show a stationary start ghost, not invent a path");
    request = *std::find_if(suite.begin(), suite.end(), [](const auto& s) { return s.id == "confirm_top"; });
    const auto prefix = make_demo_preview(request, demo_plan_options());
    require(!prefix.result.success && prefix.result.knots.size() > 1 && prefix.result.knots.back().time == 0,
            "Expected an untimed failed IK prefix fixture");
    require(prefix.ghost_knots.back().time > 0 && prefix.ghost_knots.size() == prefix.result.knots.size(),
            "Failed prefix needs independent visualization timestamps");
    const auto halfway = sample_demo_trajectory(prefix.ghost_knots, .5 * prefix.ghost_knots.back().time);
    require(!same(halfway.q, request.start) && prefix.result.knots.back().time == 0,
            "Failed ghost must animate without rewriting the planner result");
    request = scenario_suite().front();
    auto timeout = demo_plan_options(); timeout.timeout_seconds = 1e-9;
    const auto timed = make_demo_preview(request, timeout);
    require(!timed.result.success && !timed.requested_path.empty(), "Timeout lost the requested path");
    std::cout << "PASS collision/zero-knot/timeout visualization, failed ghost retiming and execution gates\n";
}
}

int main() {
    try {
        const auto request = scenario_suite().front();
        DemoPreview accelerated;
        for (const auto method : {Method::Baseline, Method::RigidCache}) {
            auto options = demo_plan_options(); options.method = method;
            auto result = make_demo_preview(request, options);
            require(result.result.success && result.result.validation.rule_success, std::string("Nominal backend failed: ") + method_name(method) + "/" + result.result.reason);
            require(result.candidate_path.size() == result.result.knots.size() && !result.requested_path.empty(), "Incomplete preview data");
            require(result.ghost_knots.size() == result.result.knots.size() &&
                    result.ghost_knots.back().time == result.result.knots.back().time, "Successful ghost changed trajectory timing");
            std::cout << "PASS real " << method_name(method) << " plan, " << result.result.knots.size() << " knots\n";
            if (method == Method::RigidCache) accelerated = std::move(result);
        }
        two_stage_contract(accelerated);
        invalidation_contract(accelerated);
        jogging_contract();
        failure_contract();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
