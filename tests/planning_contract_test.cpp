#include "tech_core/planning.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace tech_core;
using namespace tech_core::planning;

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
bool contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

// This is deliberately NOT a successful trajectory fixture. It is finite,
// ordered input for testing structural preconditions before expensive geometry.
// All joints stay home; its fabricated later stages must never be accepted as
// physical assembly merely because their mechanism labels look complete.
std::vector<Knot> structural_fixture(PlanningScene& scene) {
    Eigen::Vector3d p;
    Eigen::Matrix3d r;
    scene.forward(scene.scenario().start, p, r);
    const std::array<Phase, 6> phase{Phase::Approach, Phase::Insert, Phase::Lift,
                                   Phase::Flip, Phase::Twist, Phase::Dwell};
    const double q = radians(scene.scenario().target_degrees);
    const std::array<MechanismState, 6> state{{{}, {.1, 0, 0, 0}, {.1, .1, 0, 0},
        {.1, .1, pi/2, 0}, {.1, .1, pi/2, q}, {.1, .1, pi/2, q}}};
    std::vector<Knot> knots;
    for (std::size_t i = 0; i < phase.size(); ++i) {
        Knot k;
        k.q = scene.scenario().start;
        k.time = static_cast<double>(i);
        k.phase = phase[i];
        k.mechanism = state[i];
        k.target = i == 0 ? TaskTarget{p, r.col(2), Eigen::Vector3d::UnitZ()} : scene.mechanism_target(state[i]);
        knots.push_back(k);
    }
    return knots;
}

void invalid_trajectory_tests(PlanningScene& scene) {
    PlanOptions options;
    auto result = validate_trajectory(scene, {}, options);
    require(!result.valid && contains(result.reason, "incomplete"), "empty trajectory must fail structurally");

    auto knots = structural_fixture(scene);
    knots.front().q[0] += .02;
    result = validate_trajectory(scene, knots, options);
    require(!result.valid && contains(result.reason, "start"), "wrong initial joint state must be rejected");

    knots = structural_fixture(scene);
    knots[1].time = knots[0].time;
    result = validate_trajectory(scene, knots, options);
    require(!result.valid && (contains(result.reason, "timestamp") || contains(result.reason, "time")),
            "non-increasing timestamp must fail before an unrelated geometric failure: " + result.reason);

    // Preserve six points while deleting the lift phase, avoiding the trivial
    // minimum-size rejection and ensuring actual phase coverage is checked.
    knots = structural_fixture(scene);
    knots[2].phase = Phase::Insert;
    result = validate_trajectory(scene, knots, options);
    require(!result.valid && contains(result.reason, "phase"),
            "dropped lift phase must fail phase contract before geometry: " + result.reason);

    knots = structural_fixture(scene);
    knots.front().target.position.x() += 1;
    result = validate_trajectory(scene, knots, options);
    require(!result.valid && (contains(result.reason, "tracking") || contains(result.reason, "position") ||
                              contains(result.reason, "target")),
            "one metre target error must not be accepted or hidden by a collision: " + result.reason);
    std::cout << "contract structural/phase/position negative tests passed\n";
}

void nonfinite_tests(PlanningScene& scene) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    PlanOptions options;
    for (int field = 0; field < 5; ++field) {
        auto knots = structural_fixture(scene);
        // Place corruption in a later point: validators must preflight the
        // complete request, not fail on unrelated earlier collision geometry.
        switch (field) {
            case 0: knots.back().q[3] = nan; break;
            case 1: knots.back().target.position[0] = nan; break;
            case 2: knots.back().target.direction[1] = nan; break;
            case 3: knots.back().mechanism.lift = nan; break;
            case 4: knots.back().time = nan; break;
        }
        bool rejected = false;
        try {
            const auto result = validate_trajectory(scene, knots, options);
            rejected = !result.valid && (contains(result.reason, "invalid") ||
                contains(result.reason, "finite") || contains(result.reason, "NaN"));
            require(rejected, "NaN field " + std::to_string(field) +
                " was not explicitly rejected before geometry: " + result.reason);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "nonfinite trajectory accepted");
    }
    std::cout << "contract NaN preflight tests passed\n";
}

int arm_cluster(const std::string& name) {
    for (int i = 0; i < 7; ++i)
        if (name == "j" + std::to_string(i) + "_link_visual") return i;
    if (name == "camera_link_visual") return 2;
    if (name == "end_left_link_visual" || name == "end_right_link_visual") return 6;
    return -1;
}

void nonadjacent_collision_test(PlanningScene& scene) {
    // Deterministic joint-limited folded poses, independent of task generation
    // and IK. Require a genuine nonadjacent arm pair, and independently query
    // MuJoCo to establish overlap rather than accepting an arbitrary rejection.
    int attempts = 0;
    for (double shoulder : {.02, .4, 1.1, 2.1, 3.1}) {
        for (double elbow : {3.1, 2.7, 2., 1., 0.}) {
            for (double wrist : {0., -2.8, 2.8}) {
                JointVector q = scene.scenario().start;
                q[1] = shoulder; q[2] = elbow; q[4] = wrist;
                ++attempts;
                const auto result = scene.collision(q, {});
                const auto delimiter = result.pair.find('/');
                if (result.valid || delimiter == std::string::npos) continue;
                const auto left = result.pair.substr(0, delimiter);
                const auto right = result.pair.substr(delimiter + 1);
                const int a = arm_cluster(left), b = arm_cluster(right);
                if (a < 0 || b < 0 || std::abs(a-b) <= 1) continue;
                const int ga = mj_name2id(scene.model(), mjOBJ_GEOM, left.c_str());
                const int gb = mj_name2id(scene.model(), mjOBJ_GEOM, right.c_str());
                require(ga >= 0 && gb >= 0, "reported collision pair must exist");
                const double distance = mj_geomDistance(scene.model(), scene.data(), ga, gb, .01, nullptr);
                if (distance >= -1e-5) continue; // Require actual overlap, not clearance alone.
                require(std::isfinite(distance), "independent distance must be finite");
                std::cout << "contract nonadjacent collision detected: " << result.pair
                          << " distance=" << distance << " attempts=" << attempts << '\n';
                return;
            }
        }
    }
    throw std::runtime_error("No independently confirmed nonadjacent arm overlap was detected in 75 folded fixtures");
}

void complete_plan_test(PlanningScene& scene) {
    auto request=scenario_suite().front();
    PlanOptions options;
    const auto result=plan(scene,request,options);
    require(result.success && result.validation.rule_success,
            "complete nominal baseline regression: "+result.reason);
    options.method=Method::RigidCache;
    const auto cached=plan(scene,request,options);
    require(cached.success && cached.counters.rigid_pair_cache_hits>0,
            "complete cached planning regression: "+cached.reason);
    auto fast_options=options;fast_options.motion_speed_scale=4;
    const auto fast=plan(scene,request,fast_options);
    require(fast.success && fast.validation.rule_success,"4x motion must retain full rule completion: "+fast.reason);
    require(fast.knots.size()==cached.knots.size(),"Retiming must not drop collision/path samples");
    for(std::size_t i=1;i<fast.knots.size();++i) {
        require((fast.knots[i].q-cached.knots[i].q).norm()<1e-12,"Retiming must preserve the joint path");
        const double before=cached.knots[i].time-cached.knots[i-1].time;
        const double after=fast.knots[i].time-fast.knots[i-1].time;
        require(std::abs(after-(fast.knots[i].phase==Phase::Dwell?.5:before/4))<1e-10,
                "Motion must be 4x faster while rule dwell remains 0.5 s");
    }
    require(std::abs(fast.validation.max_joint_speed-4*cached.validation.max_joint_speed)<1e-9,
            "Time scaling must scale peak speed by four");
    require(std::abs(fast.validation.max_joint_acceleration-16*cached.validation.max_joint_acceleration)<1e-8,
            "Time scaling must scale peak acceleration by sixteen");
    scene.configure(request);
    require(validate_trajectory(scene,fast.knots,options).reason=="rate_limit",
            "Fast trajectory must not silently pass the original motion limits");
    for(double scale:{0.,-1.,std::numeric_limits<double>::quiet_NaN()}) {
        auto invalid=options;invalid.motion_speed_scale=scale;
        require(plan(scene,request,invalid).reason=="invalid_motion_speed_scale","Reject invalid speed scale before IK");
    }
    // Revalidate exactly the same solved path after adding a real box. This
    // checks the exported trajectory against changed geometry, not IK failure.
    request.blocking_obstacle=true;
    scene.configure(request);
    const auto blocked=validate_trajectory(scene,cached.knots,options);
    require(!blocked.valid && contains(blocked.reason,"collision"),
            "inserted obstacle must invalidate a previously valid complete plan");
    std::cout << "complete baseline and obstacle revalidation passed\n";
}

void q_sweep_contract_test(PlanningScene& scene) {
    auto request=scenario_suite().front();request.target_degrees=-90;request.q_sweep=true;
    scene.configure(request);
    const auto task=make_task(scene,PlanOptions{});
    std::vector<Knot> knots;
    for(const auto& point:task) {Knot k;static_cast<TaskPoint&>(k)=point;k.q=request.start;knots.push_back(k);}
    parameterize(knots);
    auto peak=std::find_if(knots.begin(),knots.end(),[](const Knot& k){
        return k.phase==Phase::Twist && std::abs(k.mechanism.twist-pi/2)<1e-12;
    });
    require(peak!=knots.end(),"Sweep must explicitly reach positive 90 degrees");
    require(std::abs(knots.back().mechanism.twist+pi/2)<1e-12,"Sweep must end at negative 90 degrees");
    // Structural fixtures intentionally do not solve IK: omission/order must
    // be rejected before their unrelated FK/collision failure.
    auto missing=knots;
    missing.erase(missing.begin()+(peak-knots.begin())+1,missing.end()-1);
    require(validate_trajectory(scene,missing,PlanOptions{}).reason=="q_sweep_missing_endpoint",
            "A jump from +90 directly into final dwell must not count as the return leg");
    auto reversed=knots;
    auto first=std::find_if(reversed.begin(),reversed.end(),[](const Knot& k){return k.phase==Phase::Twist;});
    first->mechanism.twist=-radians(1);first->target=scene.mechanism_target(first->mechanism);
    require(validate_trajectory(scene,reversed,PlanOptions{}).reason=="q_sweep_order",
            "Negative Q before visiting +90 must be rejected");
    bool rejected=false;
    request.target_degrees=35;
    try {scene.configure(request);} catch(const std::invalid_argument&) {rejected=true;}
    require(rejected,"Sweep and final target must agree");
    request.core={-.09,.2,.5,0,80,10};request.target_degrees=-90;
    request.arm_base_z=-.1;request.assembly_translation.x()=.2;
    request.start<<-.58298870078787879,1.4014363481415861,1.4956286729240151,
        .51538441396359136,-1.5264021873645037,-.58813609807777278,2.7857137959190528;
    PlanOptions options;options.method=Method::RigidCache;options.motion_speed_scale=4;
    const auto complete=plan(scene,request,options);
    require(complete.success && complete.validation.rule_success,
            "Lowered tilted-core Q sweep must complete with collision validation: "+complete.reason);
    std::cout << "Q sweep endpoint and order contract tests passed\n";
}
} // namespace

int main() {
    try {
        PlanningScene scene;
        Scenario scenario;
        scenario.id = "contract_negative_fixture";
        scenario.assembly_translation.x() = .1;
        scenario.core.z = .5;
        scene.configure(scenario);
        invalid_trajectory_tests(scene);
        nonfinite_tests(scene);
        nonadjacent_collision_test(scene);
        complete_plan_test(scene);
        q_sweep_contract_test(scene);
        std::cout << "planning contract negative tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "planning contract test: " << error.what() << '\n';
        return 1;
    }
}
