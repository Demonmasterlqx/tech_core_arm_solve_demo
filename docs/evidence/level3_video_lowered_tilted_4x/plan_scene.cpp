#include "tech_core/planning.hpp"

#include <algorithm>
#include <random>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {
using namespace tech_core::planning;
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}
std::string csv(const std::string& text) {
    std::string result = "\"";
    for (char c : text) { if (c == '"') result += '"'; result += c; }
    return result + '"';
}
std::string filename(std::string text) {
    for (char& c : text)
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '-' && c != '_') c = '_';
    return text;
}
std::ofstream output_file(const std::filesystem::path& path) {
    std::ofstream stream(path);
    if (!stream) throw std::runtime_error("Cannot write " + path.string());
    stream.exceptions(std::ios::badbit | std::ios::failbit);
    stream << std::setprecision(17);
    return stream;
}
void write_scenarios(const std::filesystem::path& path, const std::vector<Scenario>& suite) {
    auto out = output_file(path);
    out << "case,split,core_x,core_y,core_z,core_theta_degrees,core_phi_degrees,core_alpha_degrees,target_degrees,start_q0,start_q1,start_q2,start_q3,start_q4,start_q5,start_q6,assembly_translation_x,assembly_translation_y,assembly_translation_z,assembly_yaw_radians,blocking_obstacle,arm_base_z\n";
    for (const auto& scenario : suite) {
        const auto& p = scenario.core;
        out << csv(scenario.id) << ',' << csv(scenario.split) << ',' << p.x << ',' << p.y << ',' << p.z
            << ',' << p.theta << ',' << p.phi << ',' << p.alpha << ',' << scenario.target_degrees;
        for (int k = 0; k < 7; ++k) out << ',' << scenario.start[k];
        for (int k = 0; k < 3; ++k) out << ',' << scenario.assembly_translation[k];
        out << ',' << scenario.assembly_yaw << ',' << scenario.blocking_obstacle << ',' << scenario.arm_base_z << '\n';
    }
}
PlanResult safe_plan(PlanningScene& scene, const Scenario& scenario, const PlanOptions& options,
                     const std::vector<AtlasEntry>* atlas = nullptr) {
    const auto start = Clock::now();
    try { return plan(scene, scenario, options, atlas); }
    catch (const std::exception& error) {
        PlanResult result;
        result.method = options.method;
        result.planning_seconds = elapsed(start);
        result.reason = std::string("exception: ") + error.what();
        return result;
    }
}
void write_trajectory(const std::filesystem::path& path, const PlanResult& result) {
    auto out = output_file(path);
    out << "knot,time,phase,insertion,lift,flip,twist,q0,q1,q2,q3,q4,q5,q6,target_x,target_y,target_z,direction_x,direction_y,direction_z\n";
    for (std::size_t i = 0; i < result.knots.size(); ++i) {
        const auto& knot = result.knots[i];
        out << i << ',' << knot.time << ',' << phase_name(knot.phase) << ','
            << knot.mechanism.insertion << ',' << knot.mechanism.lift << ','
            << knot.mechanism.flip << ',' << knot.mechanism.twist;
        for (int k = 0; k < 7; ++k) out << ',' << knot.q[k];
        for (int k = 0; k < 3; ++k) out << ',' << knot.target.position[k];
        for (int k = 0; k < 3; ++k) out << ',' << knot.target.direction[k];
        out << '\n';
    }
}
void usage() {
    std::cout << "rm26_plan [--method NAME|all] [--case ID|all] [--repeat N]\n"
                 "          [--split development|confirmation|negative|all] [--timeout SECONDS]\n"
                 "          [--output DIRECTORY] [--dense-check] [--motion-speed-scale S] [--list]\n";
}
} // namespace


int main(int argc, char** argv) {
    try {
        if(argc!=2)throw std::runtime_error("Need output directory");
        const std::filesystem::path output=argv[1];
        if(std::filesystem::exists(output/"runs.csv"))throw std::runtime_error("Output already contains results");
        std::filesystem::create_directories(output/"trajectories");
        PlanningScene scene;
        auto nominal=scenario_suite().front();
        nominal.assembly_translation.x()=.20;nominal.arm_base_z=-.10;
        nominal.core.phi=80;nominal.core.alpha=10;
        nominal.start<<-.53563707316736642,1.5610949790455431,1.1846530704622904,.41477115146925325,-1.7293000544221846,.26270487558236871,2.5049445646440422;
        std::vector<Scenario> scenarios;
        auto runs=output_file(output/"runs.csv");
        runs << "method,case,split,repeat,success,plan_success,reason,planning_seconds,ik_seconds,validation_seconds,dense_recheck_seconds,dense_recheck_performed,dense_recheck_valid,dense_recheck_reason,knots,ik_iterations,fk_evals,collision_states,narrowphase_calls,broadphase_skips,validation_samples,max_position_error,max_axis_error,max_joint_speed,max_joint_acceleration,rule_success,dense_recheck_samples,trajectory,atlas_source,rigid_pair_cache_hits,motion_speed_scale\n";
        auto scan=output_file(output/"ready_search.csv");
        scan<<"attempt,ready_ik_success,start_clear,planning_success,dense_valid,planning_seconds,reason\n";
        std::mt19937 generator(20261012);std::normal_distribution<double> normal(0,1);
        bool accepted=false;
        for(int attempt=0;attempt<256 && !accepted;++attempt) {
            Scenario scenario=nominal;scenario.id="lowered_tilted_4x_s"+std::to_string(attempt);scenario.split="video";
            scene.configure(scenario);
            if(!scene.core_within_front_plane())throw std::runtime_error("Tilted core violates initial front plane");
            auto target=scene.mechanism_target({});target.position+=.06*target.direction;
            const ForwardKinematics fk=[&](const JointVector& q,Eigen::Vector3d& p,Eigen::Matrix3d& r,Jacobian* j){scene.forward(q,p,r,j);};
            IKOptions ik;ik.method=IKMethod::FullStep;ik.max_iterations=600;ik.position_tolerance=1e-7;ik.axis_tolerance=1e-7;
            JointVector seed=nominal.start;
            const double sigma=attempt<128?.35:.9;
            if(attempt)for(int j=0;j<7;++j)seed[j]+=sigma*normal(generator);
            auto ready=solve_ik(fk,target,seed,scene.limits(),ik);
            scenario.start=ready.q;scene.configure(scenario);
            const auto start=scene.collision(scenario.start,{});
            if(!ready.success || !start.valid) {
                scan<<attempt<<','<<ready.success<<','<<start.valid<<",0,0,0,"<<csv(ready.success?start.pair:ready.reason)<<'\n';continue;
            }
            PlanOptions options;options.method=Method::RigidCache;options.motion_speed_scale=4;
            const auto result=safe_plan(scene,scenario,options);
            const auto method=Method::RigidCache;const int repeat=0;
            ValidationResult recheck;bool rechecked=false;double recheck_seconds=0;
            if(result.success) {
                auto dense=options;dense.validation_joint_step*=.5;dense.validation_min_substeps*=2;
                scene.configure(scenario);const auto begin=Clock::now();rechecked=true;
                recheck=validate_trajectory(scene,result.knots,dense);recheck_seconds=elapsed(begin);
            }
            const bool success=result.success && rechecked && recheck.valid;accepted=success;
            const std::string reason=result.success && !recheck.valid?recheck.reason:result.reason;
            const std::string relative="trajectories/rigid_cache_"+scenario.id+"_r0.csv";
            write_trajectory(output/relative,result);scenarios.push_back(scenario);
            const auto& c=result.counters;const auto& v=result.validation;
                                runs << csv(method_name(method)) << ',' << csv(scenario.id) << ',' << csv(scenario.split)
                         << ',' << repeat << ',' << success << ',' << result.success << ',' << csv(reason)
                         << ',' << result.planning_seconds << ',' << result.ik_seconds << ',' << result.validation_seconds
                         << ',' << recheck_seconds << ',' << rechecked << ',' << (rechecked && recheck.valid) << ',' << csv(recheck.reason)
                         << ',' << result.knots.size() << ',' << c.ik_iterations << ',' << c.fk_evals
                         << ',' << c.collision_states << ',' << c.narrowphase_calls << ',' << c.broadphase_skips
                         << ',' << v.samples << ',' << v.max_position_error << ',' << v.max_axis_error
                         << ',' << v.max_joint_speed << ',' << v.max_joint_acceleration << ',' << v.rule_success
                         << ',' << recheck.samples << ',' << csv(relative) << ',' << csv(result.atlas_source)
                         << ',' << c.rigid_pair_cache_hits << ',' << options.motion_speed_scale << '\n';

            scan<<attempt<<','<<ready.success<<','<<start.valid<<','<<result.success<<','<<recheck.valid<<','<<result.planning_seconds<<','<<csv(reason)<<'\n';
            std::cout<<scenario.id<<" success="<<success<<" seconds="<<result.planning_seconds<<" reason="<<reason<<std::endl;
            if(success)std::cout<<"EXECUTION_SECONDS "<<result.knots.back().time<<" MAX_V "<<v.max_joint_speed<<" MAX_A "<<v.max_joint_acceleration<<std::endl;
        }
        write_scenarios(output/"scenarios.csv",scenarios);
        return accepted?0:2;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
