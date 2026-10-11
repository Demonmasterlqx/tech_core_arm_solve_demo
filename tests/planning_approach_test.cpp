#include "tech_core/planning.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace tech_core;
using namespace tech_core::planning;

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void audit(PlanningScene& scene, const std::vector<JointVector>& path) {
    require(!path.empty(), "Missing approach path");
    require(scene.collision(path.front(), {}).valid, "Invalid approach start");
    for (std::size_t i = 1; i < path.size(); ++i) {
        // An independent, finer uniform grid, without either collision cache.
        const int steps = std::max(1, static_cast<int>(std::ceil(
            (path[i]-path[i-1]).cwiseAbs().maxCoeff()/.002)));
        for (int j = 1; j <= steps; ++j) {
            const auto collision = scene.collision(path[i-1]+(double(j)/steps)*(path[i]-path[i-1]), {});
            require(collision.valid, "Optimized edge failed dense recheck: "+collision.pair);
        }
    }
}
}

int main() {
    try {
        PlanningScene scene;
        auto request = scenario_suite().front(); request.start.setZero();
        scene.configure(request);
        auto target = scene.mechanism_target({}); target.position += .04*target.direction;
        const ForwardKinematics fk = [&](const JointVector& q, Eigen::Vector3d& p, Eigen::Matrix3d& r, Jacobian* j) {
            scene.forward(q,p,r,j);
        };
        auto ik = solve_ik(fk,target,request.start,scene.limits());
        require(ik.success, "The screenshot's Baseline pre-insertion IK must converge");
        const auto original_goal = scene.collision(ik.q,{});
        std::cout << "Original zero-seed IK goal valid=" << original_goal.valid << " pair=" << original_goal.pair << std::endl;
        require(!original_goal.valid, "Original goal must reproduce its self-collision");
        // This goal uses a distinct IK branch at exactly the same target pose.
        ik = solve_ik(fk,target,scenario_suite().front().start,scene.limits());
        require(ik.success && scene.collision(ik.q,{}).valid, "Fixture needs a clear alternative IK goal");
        for (auto method : {Method::Baseline,Method::RigidCache}) {
            PlanOptions options; options.method=method; options.timeout_seconds=5; options.motion_speed_scale=4;
            const auto result=plan(scene,request,options);
            std::cout << "zero-start " << method_name(method) << " success=" << result.success << " reason=" << result.reason
                      << " IK attempts=" << result.preinsert_ik_attempts << " approach seconds=" << result.approach_seconds
                      << " total seconds=" << result.planning_seconds << std::endl;
            require(result.success && result.preinsert_ik_attempts>=2 && result.knots.front().q.isZero(0),
                    "A collision-free pre-insertion branch must allow the zero-start request: "+result.reason);
            scene.configure(request);
            require(validate_trajectory(scene,result.knots,options).valid, "Zero-start plan failed uncached full validation");
        }
        Eigen::Vector3d obstruction; Eigen::Matrix3d rotation;
        scene.forward(.5*ik.q,obstruction,rotation);
        const int obstacle=mj_name2id(scene.model(),mjOBJ_GEOM,"collision_block_fixture");
        auto obstruct=[&] {
            Eigen::Map<Eigen::Vector3d>(scene.model()->geom_pos+3*obstacle)=obstruction;
        };
        obstruct();
        require(scene.collision(request.start,{}).valid && scene.collision(ik.q,{}).valid,
                "Mid-path obstacle must leave both endpoints clear");
        bool straight_blocked = false;
        for (int i = 1; i < 300; ++i)
            if (!scene.collision((double(i)/300)*ik.q,{}).valid) { straight_blocked = true; break; }
        require(straight_blocked, "Mid-path obstacle must block the straight connection");

        for (bool cached : {false,true}) {
            scene.configure(request); obstruct(); scene.enable_rigid_pair_cache(cached);
            PlanOptions options; options.timeout_seconds = 3;
            const auto begin = std::chrono::steady_clock::now();
            const auto result = optimize_approach(scene,request.start,ik.q,options);
            const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
            std::cout << "approach cached=" << cached << " success=" << result.success << " reason=" << result.reason
                      << " iterations=" << result.iterations << " shortcuts=" << result.shortcuts
                      << " length=" << result.initial_length << " -> " << result.optimized_length
                      << " seconds=" << elapsed << std::endl;
            require(result.success, "Collision-aware approach failed: "+result.reason);
            require((result.path.front()-request.start).norm() < 1e-12 && (result.path.back()-ik.q).norm() < 1e-12,
                    "Optimization changed a fixed endpoint");
            require(result.iterations > 0 && result.path.size() > 2 && result.shortcuts > 0 &&
                    result.optimized_length <= result.initial_length+1e-10,
                    "Blocked straight line must be rerouted and shortened");
            scene.configure(request); obstruct();
            audit(scene,result.path);
        }
        PlanOptions options;
        scene.configure(request); obstruct();
        int polls=0;
        const auto cancelled=optimize_approach(scene,request.start,ik.q,options,[&]{return ++polls>=600;});
        require(!cancelled.success && cancelled.reason=="deadline_during_approach" && !cancelled.path.empty() &&
                cancelled.path.front().isZero(0) && (cancelled.path.back()-ik.q).norm()>1e-6,
                "Interrupted search must return only a connected start-side prefix");
        scene.configure(request); obstruct();
        audit(scene,cancelled.path);
        const auto expired = optimize_approach(scene,request.start,ik.q,options,[]{return true;});
        require(!expired.success && expired.reason == "deadline_during_approach", "Expired request must not succeed");
        auto invalid = ik.q; invalid[0] = scene.limits().upper[0]+1;
        const auto outside = optimize_approach(scene,request.start,invalid,options);
        require(!outside.success && outside.reason == "approach_goal_collision_joint_limits" &&
                outside.path.size() == 1 && outside.path.front().isZero(0),
                "Invalid goal must retain only the physical start, not an unchecked connection");
        std::cout << "PASS zero-start planning, obstacle detour, dense uncached recheck, endpoint and deadline contracts\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n'; return 1;
    }
}
