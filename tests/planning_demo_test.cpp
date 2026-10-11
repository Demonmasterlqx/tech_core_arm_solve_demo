#include "tech_core/planning_demo.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
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
void require_zeroed(PlanningDemo& demo) {
    require(demo.state() == DemoState::Editing && demo.joints().isZero(0) && !demo.preview(),
            "Backspace must zero all seven joints and clear the preview");
    const auto& mechanism = demo.mechanism();
    require(mechanism.insertion == 0 && mechanism.lift == 0 && mechanism.flip == 0 && mechanism.twist == 0,
            "Backspace did not reset the assembly mechanism");
    auto& scene = demo.scene();
    require(demo.execution_time() == 0 && scene.data()->time == 0, "Backspace did not reset the execution clock");
    for (const char* name : {"j0", "j1", "j2", "j3", "j4",
                            "end_differential_controller/j5", "end_differential_controller/j6"}) {
        const int joint = mj_name2id(scene.model(), mjOBJ_JOINT, name);
        require(joint >= 0 && scene.data()->qpos[scene.model()->jnt_qposadr[joint]] == 0,
                "Displayed joint differs from the requested zero");
    }
    for (int i = 0; i < scene.model()->nv; ++i)
        require(scene.data()->qvel[i] == 0, "Backspace left a nonzero model velocity");
}

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
    require_zeroed(demo);
    demo.reset();
    require(same(initial, demo.joints()) && !demo.joints().isZero(0), "R must restore the initial carrying posture");
    std::cout << "PASS preview hold, double Enter, interpolation, focus pause, completion, joint zero and scene reset\n";
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
    demo.reset(); demo.enter(); finish_request(demo);
    demo.jog_core({-.01, 0, 0}, {0, 0, 0}, .05);
    require(demo.state() == DemoState::Editing && !demo.preview(), "Core jog must invalidate a ready preview");
    const auto core = demo.core_position();
    demo.home();
    require_zeroed(demo);
    require((core - demo.core_position()).norm() < 1e-12, "Backspace must preserve edited core pose");
    require(demo.options().method == Method::Baseline, "Backspace must preserve the selected backend");

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
    require_zeroed(stale);
    std::cout << "PASS arm/core/backend invalidation and stale worker disposal\n";
}

void zero_start_contract() {
    std::promise<Scenario> captured;
    auto request = captured.get_future();
    PlanningDemo demo(scenario_suite().front(), demo_plan_options(),
        [&captured](const Scenario& scenario, const PlanOptions&) {
            captured.set_value(scenario);
            return DemoPreview{};
        });
    demo.jog_core({-.2, .2, -.2}, {60, 60, 60}, .1);
    const auto position = demo.core_position();
    const auto pose = demo.scenario().core;
    demo.home();
    require_zeroed(demo);
    require((demo.core_position() - position).norm() < 1e-12 && demo.scenario().core.alpha == pose.alpha &&
            demo.scenario().core.phi == pose.phi && demo.scenario().core.theta == pose.theta,
            "Backspace changed the core XYZ/RPY");
    demo.enter(); finish_request(demo);
    const auto actual = request.get();
    require(actual.start.isZero(0), "Planning after Backspace must start at seven zero joints");
    require(!actual.enforce_core_pose_limits && actual.core.z == pose.z && actual.core.phi == pose.phi,
            "Planner snapshot lost the freely edited core pose");
    require(demo.preview() && demo.preview()->ghost_knots.front().q.isZero(0),
            "Empty candidate ghost must start at the zero joint pose");
    std::cout << "PASS exact zero joints passed to planner, core pose preserved and zero-start ghost\n";
}

void jogging_contract() {
    PlanningDemo demo;
    const auto initial = demo.joints();
    for (int axis = 0; axis < 3; ++axis) for (int sign : {-1, 1}) {
        demo.reset();
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
    require_zeroed(demo);
    demo.reset();
    require(same(initial, demo.joints()), "R did not restore the initial posture after jogging and zeroing");
    const auto before = demo.core_position();
    demo.jog_core({-.04, .04, .04}, {10, -10, 10}, .1);
    const auto delta = (demo.core_position() - before).eval();
    require(delta.x() < -.002 && delta.y() > .002 && delta.z() > .002, "Core world axes differ from arm world axes");
    require(demo.scenario().core.alpha == 1 && demo.scenario().core.phi == 89 && demo.scenario().core.theta == 1,
            "Core RPY controls use wrong parameter mapping");
    std::cout << "PASS six arm directions, release hold and core world XYZ/RPY mapping\n";
}

void free_core_contract() {
    PlanningDemo demo;
    for (int axis = 0; axis < 3; ++axis) for (int sign : {-1, 1}) {
        demo.reset();
        const auto before = demo.core_position();
        for (int i = 0; i < 20; ++i) demo.jog_core(.2 * sign * Eigen::Vector3d::Unit(axis), {0, 0, 0}, .1);
        require((demo.core_position() - before - .4 * sign * Eigen::Vector3d::Unit(axis)).norm() < 1e-12,
                "Core translation was clipped or slowed outside the former bounds");
        const int core = mj_name2id(demo.scene().model(), mjOBJ_BODY, "core0_base");
        require((Eigen::Map<const Eigen::Vector3d>(demo.scene().data()->xpos + 3 * core) -
                 demo.core_position()).norm() < 1e-12, "Rendered core differs from the unbounded pose");
        demo.reset();
        const auto initial = demo.scenario().core;
        for (int i = 0; i < 20; ++i) demo.jog_core({0, 0, 0}, 60 * sign * Eigen::Vector3d::Unit(axis), .1);
        const auto& pose = demo.scenario().core;
        const Eigen::Vector3d delta(pose.alpha - initial.alpha, pose.phi - initial.phi, pose.theta - initial.theta);
        require((delta - 120 * sign * Eigen::Vector3d::Unit(axis)).norm() < 1e-12,
                "Core rotation was clipped or slowed outside the former bounds");
    }

    PlanningScene strict;
    auto outside = scenario_suite().front(); outside.core.x = .2;
    bool rejected = false;
    try { strict.configure(outside); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Non-demo scenes must retain the default rule-table bounds");
    for (auto field : {&Pose::x, &Pose::y, &Pose::z, &Pose::alpha, &Pose::phi, &Pose::theta}) {
        auto invalid = demo.scenario(); invalid.core.*field = std::numeric_limits<double>::quiet_NaN();
        rejected = false;
        try { strict.configure(invalid); } catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "Unbounded scenes must still reject nonfinite core poses");
    }

    demo.reset();
    for (int i = 0; i < 20; ++i) demo.jog_core({-.2, 0, 0}, {0, 0, 0}, .1);
    const auto edited = demo.core_position();
    for (auto method : {Method::Baseline, Method::RigidCache}) {
        demo.set_backend(method);
        demo.enter(); finish_request(demo);
        require(demo.state() == DemoState::PreviewFailed && demo.preview()->result.reason == "invalid_station_front_plane",
                "Out-of-range core pose must reach the planner and retain its rejection reason");
        require(!demo.preview()->requested_path.empty() && !demo.preview()->ghost_knots.empty(),
                "Out-of-range failed plan lost the requested path or start ghost");
        demo.enter(); demo.tick(.1);
        require(demo.state() == DemoState::PreviewFailed && (demo.core_position() - edited).norm() < 1e-12,
                "Rejected out-of-range request executed or changed the core pose");
    }
    std::cout << "PASS unbounded core XYZ/RPY at increased speeds, strict defaults and failed preview in both backends\n";
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
        zero_start_contract();
        jogging_contract();
        free_core_contract();
        failure_contract();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
