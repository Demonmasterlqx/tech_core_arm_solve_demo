#include "tech_core/planning.hpp"

#include <algorithm>
#include <random>
#include <sstream>
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
    out << "case,split,core_x,core_y,core_z,core_theta_degrees,core_phi_degrees,core_alpha_degrees,target_degrees,start_q0,start_q1,start_q2,start_q3,start_q4,start_q5,start_q6,assembly_translation_x,assembly_translation_y,assembly_translation_z,assembly_yaw_radians,blocking_obstacle,arm_base_z,q_sweep,q_best_effort\n";
    for (const auto& scenario : suite) {
        const auto& p = scenario.core;
        out << csv(scenario.id) << ',' << csv(scenario.split) << ',' << p.x << ',' << p.y << ',' << p.z
            << ',' << p.theta << ',' << p.phi << ',' << p.alpha << ',' << scenario.target_degrees;
        for (int k = 0; k < 7; ++k) out << ',' << scenario.start[k];
        for (int k = 0; k < 3; ++k) out << ',' << scenario.assembly_translation[k];
        out << ',' << scenario.assembly_yaw << ',' << scenario.blocking_obstacle << ',' << scenario.arm_base_z << ',' << scenario.q_sweep << ',' << scenario.q_best_effort << '\n';
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

// Explicit trial logging keeps ready search outside selected-request latency.
const char* run_header = "method,case,split,repeat,success,plan_success,reason,planning_seconds,ik_seconds,validation_seconds,dense_recheck_seconds,dense_recheck_performed,dense_recheck_valid,dense_recheck_reason,knots,ik_iterations,fk_evals,collision_states,narrowphase_calls,broadphase_skips,validation_samples,max_position_error,max_axis_error,max_joint_speed,max_joint_acceleration,rule_success,dense_recheck_samples,trajectory,atlas_source,rigid_pair_cache_hits,motion_speed_scale,planning_mode,validation_joint_step,validation_min_substeps,q_positive_degrees,q_final_degrees,full_q_sweep,q_positive_stop,q_negative_stop,exact_state_cache_hits\n";
void write_run(std::ostream& out,const Scenario& s,const PlanResult& r,const ValidationResult& dense,
               bool rechecked,double dense_seconds,const std::string& trajectory,bool online=false) {
    const auto& c=r.counters;const auto& v=r.validation;
    const bool success=r.success && rechecked && dense.valid;
    out<<csv("rigid_cache")<<','<<csv(s.id)<<','<<csv(s.split)<<",0,"<<success<<','<<r.success<<','
       <<csv(r.success&&rechecked&&!dense.valid?dense.reason:r.reason)<<','<<r.planning_seconds<<','<<r.ik_seconds<<','<<r.validation_seconds
       <<','<<dense_seconds<<','<<rechecked<<','<<(rechecked&&dense.valid)<<','<<csv(dense.reason)<<','<<r.knots.size()
       <<','<<c.ik_iterations<<','<<c.fk_evals<<','<<c.collision_states<<','<<c.narrowphase_calls<<','<<c.broadphase_skips
       <<','<<v.samples<<','<<v.max_position_error<<','<<v.max_axis_error<<','<<v.max_joint_speed<<','<<v.max_joint_acceleration
       <<','<<v.rule_success<<','<<dense.samples<<','<<csv(trajectory)<<','<<csv(r.atlas_source)<<','<<c.rigid_pair_cache_hits
       <<",4,"<<(online?"online_fixed_start,0.004,4":"offline_ready_search,0.008,2")
       <<','<<r.q_positive_degrees<<','<<r.q_final_degrees<<','<<r.full_q_sweep
       <<','<<csv(r.q_positive_stop)<<','<<csv(r.q_negative_stop)<<','<<c.exact_state_cache_hits<<'\n';
    out.flush();
}

void run_online(const std::filesystem::path& output,const std::vector<Scenario>& requested,bool retry) {
    // The same physical input configuration is retained in every request.
    // plan() solves the approach from this measured-state surrogate; there is
    // no trajectory lookup or cross-request warm start. Optional retries may
    // select a waypoint, but plan() connects it to this unchanged physical start.
    const auto init=Clock::now();
    PlanningScene scene;
    auto initialization=output_file(output/"initialization.csv");
    initialization<<"model_load_seconds\n"<<elapsed(init)<<'\n';
    auto runs=output_file(output/"runs.csv"),summary=output_file(output/"case_summary.csv");
    std::filesystem::create_directories(output/"trials/trajectories");
    auto trial_runs=output_file(output/"trials/runs.csv"),scan=output_file(output/"online_attempts.csv");
    runs<<run_header;
    trial_runs<<run_header;
    scan<<"case,attempt,kind,ready_ik_seconds,ready_success,ready_clear,plan_called,reason,seed0,seed1,seed2,seed3,seed4,seed5,seed6,waypoint0,waypoint1,waypoint2,waypoint3,waypoint4,waypoint5,waypoint6\n";
    summary<<"case,success,online_planning_seconds,ik_seconds,validation_seconds,uncached_audit_seconds,execution_seconds,planning_attempts,ready_attempts,selected_attempt,reason,search_stop,q_positive_degrees,q_final_degrees,full_q_sweep\n";
    write_scenarios(output/"scenarios.csv",requested);
    std::vector<Scenario> trial_scenarios;
    struct Attempt {
        int number=0;bool called=false;double ready_seconds=0;
        JointVector seed=JointVector::Zero(7);IKResult ready;CollisionResult clearance;
        PlanResult result;
    };
    auto add_counters=[](Counters& a,const Counters& b) {
        a.ik_iterations+=b.ik_iterations;a.fk_evals+=b.fk_evals;a.collision_states+=b.collision_states;
        a.narrowphase_calls+=b.narrowphase_calls;a.broadphase_skips+=b.broadphase_skips;
        a.rigid_pair_cache_hits+=b.rigid_pair_cache_hits;a.exact_state_cache_hits+=b.exact_state_cache_hits;
    };
    for(std::size_t index=0;index<requested.size();++index) {
        const auto& s=requested[index];
        PlanOptions options;options.method=Method::RigidCache;options.motion_speed_scale=4;
        options.timeout_seconds=3;options.validation_joint_step=.004;options.validation_min_substeps=4;
        const auto begin=Clock::now();
        const double search_limit=s.q_best_effort?2.94:3.;
        const auto stop=[&](){return elapsed(begin)>=search_limit;};
        PlanResult result;result.method=Method::RigidCache;result.reason="online_search_exhausted";
        std::vector<Attempt> attempts;
        std::mt19937 generator(20261013+index);std::normal_distribution<double> normal(0,1);
        Counters total;double total_ik=0,total_validation=0;int plans=0,ready_count=0,selected=-1;
        for(int candidate=0;candidate<(retry?97:1) && plans<24 && !stop();++candidate) {
            Attempt attempt;attempt.number=candidate;attempt.seed=s.start;
            if(candidate) {
                ++ready_count;scene.configure(s);scene.enable_rigid_pair_cache(true);
                auto target=scene.mechanism_target({});target.position+=.06*target.direction;
                const ForwardKinematics fk=[&](const JointVector& q,Eigen::Vector3d& p,Eigen::Matrix3d& r,Jacobian* j){scene.forward(q,p,r,j);};
                IKOptions ik;ik.method=IKMethod::FullStep;ik.max_iterations=600;
                ik.position_tolerance=1e-7;ik.axis_tolerance=1e-7;ik.should_stop=stop;
                if(candidate>=2)for(int j=0;j<7;++j)attempt.seed[j]+=(candidate<32?.35:candidate<64?.7:1.2)*normal(generator);
                const auto ik_start=Clock::now();
                attempt.ready=solve_ik(fk,target,attempt.seed,scene.limits(),ik);
                attempt.ready_seconds=elapsed(ik_start);total_ik+=attempt.ready_seconds;
                scene.counters.ik_iterations+=attempt.ready.iterations;
                if(attempt.ready.success && !stop())attempt.clearance=scene.collision(attempt.ready.q,{});
                else attempt.clearance.valid=false;
                add_counters(total,scene.counters);
                if(!attempt.ready.success || !attempt.clearance.valid || stop()) {
                    attempts.push_back(std::move(attempt));continue;
                }
                options.approach_joint_waypoint=attempt.ready.q;
            }
            options.timeout_seconds=search_limit-elapsed(begin);
            if(options.timeout_seconds<=0)break;
            attempt.called=true;++plans;
            attempt.result=safe_plan(scene,s,options);
            total_ik+=attempt.result.ik_seconds;total_validation+=attempt.result.validation_seconds;
            add_counters(total,attempt.result.counters);
            const auto score=[](const PlanResult& r){return 2*r.q_positive_degrees-r.q_final_degrees;};
            if(!result.success || (attempt.result.success && score(attempt.result)>score(result)+1e-8)) {
                result=attempt.result;selected=candidate;
            }
            const bool fatal=attempt.result.reason.rfind("start_collision",0)==0 ||
                             attempt.result.reason=="invalid_station_front_plane";
            attempts.push_back(std::move(attempt));
            if((result.success && (!s.q_best_effort || result.full_q_sweep)) || fatal)break;
        }
        const std::string search_stop=stop()?"online_budget":
            result.success && (!s.q_best_effort || result.full_q_sweep)?"full_path":
            !retry?"single_attempt":"attempt_limit";
        result.planning_seconds=elapsed(begin);
        result.ik_seconds=total_ik;result.validation_seconds=total_validation;result.counters=total;
        if(result.planning_seconds>=3) {
            result.success=false;result.reason="online_deadline";
        }
        ValidationResult audit;double audit_seconds=0;bool rechecked=false;
        if(result.success) {
            // Verification of the cache implementation, not extra online path
            // acceptance: identical density/tolerances, cache disabled. Abort
            // on any disagreement instead of reporting a late success.
            const auto audit_start=Clock::now();
            scene.configure(s);
            audit=validate_trajectory(scene,result.knots,options);
            audit_seconds=elapsed(audit_start);rechecked=true;
            if(!audit.valid || audit.samples!=result.validation.samples)
                throw std::runtime_error(s.id+" online/uncached validation mismatch: "+audit.reason);
        }
        const auto path="trajectories/rigid_cache_"+s.id+"_r0.csv";
        write_trajectory(output/path,result);
        write_run(runs,s,result,audit,rechecked,audit_seconds,path,true);
        for(const auto& attempt:attempts) {
            const auto reason=attempt.called?attempt.result.reason:
                (attempt.ready.success?attempt.clearance.pair:attempt.ready.reason);
            scan<<csv(s.id)<<','<<attempt.number<<','<<(attempt.number?"waypoint":"direct")<<','<<attempt.ready_seconds
                <<','<<(attempt.number && attempt.ready.success)<<','<<(attempt.number && attempt.clearance.valid)
                <<','<<attempt.called<<','<<csv(reason);
            for(int j=0;j<7;++j)scan<<','<<attempt.seed[j];
            for(int j=0;j<7;++j)scan<<','<<(attempt.number?attempt.ready.q[j]:s.start[j]);
            scan<<'\n';
            if(attempt.called) {
                auto trial=s;trial.id+="_try"+std::to_string(attempt.number);trial_scenarios.push_back(trial);
                const auto trial_path="trajectories/rigid_cache_"+trial.id+"_r0.csv";
                write_trajectory(output/"trials"/trial_path,attempt.result);
                const bool audited=result.success && attempt.number==selected;
                write_run(trial_runs,trial,attempt.result,audited?audit:ValidationResult{},audited,
                          audited?audit_seconds:0,trial_path,true);
            }
        }
        scan.flush();
        summary<<csv(s.id)<<','<<result.success<<','<<result.planning_seconds<<','<<result.ik_seconds
               <<','<<result.validation_seconds<<','<<audit_seconds<<','
               <<(result.success?result.knots.back().time:0)<<','<<plans<<','<<ready_count<<','<<selected
               <<','<<csv(result.reason)<<','<<csv(search_stop)<<','<<result.q_positive_degrees<<','<<result.q_final_degrees
               <<','<<result.full_q_sweep<<'\n';summary.flush();
        std::cout<<"CASE_COMPLETE "<<index+1<<'/'<<requested.size()<<' '<<s.id<<" success="<<result.success
                 <<" online_s="<<result.planning_seconds<<" audit_s="<<audit_seconds<<" reason="<<result.reason<<std::endl;
    }
    write_scenarios(output/"trials/scenarios.csv",trial_scenarios);
}
} // namespace
int main(int argc,char** argv) {
  try {
    if(argc<2 || argc>6)throw std::runtime_error("Usage: plan_pose_sweep OUTPUT [MAX_READY=96] [MAX_PLANS=24] [--online|--online-retry] [--best-effort-q]");
    const bool online=argc>=5;
    const bool best_effort=argc==6 && std::string(argv[5])=="--best-effort-q";
    if(argc==6 && !best_effort)throw std::runtime_error("Unknown Q mode");
    const bool retry=online && std::string(argv[4])=="--online-retry";
    if(online && std::string(argv[4])!="--online" && !retry)throw std::runtime_error("Unknown planning mode");
    const std::filesystem::path output=argv[1];
    const int max_ready=argc>2?std::stoi(argv[2]):96,max_plans=argc>3?std::stoi(argv[3]):24;
    if(max_ready<1||max_plans<1)throw std::runtime_error("Search limits must be positive");
    if(std::filesystem::exists(output/"runs.csv"))throw std::runtime_error("Output already has runs.csv");
    std::ifstream input(output/"requested_poses.csv");if(!input)throw std::runtime_error("Missing requested_poses.csv");
    std::string line;std::getline(input,line);std::vector<Scenario> requested;
    while(std::getline(input,line)) {
      if(line.empty())continue;
      std::replace(line.begin(),line.end(),',',' ');std::istringstream row(line);
      Scenario s;row>>s.id>>s.core.x>>s.core.y>>s.core.z>>s.core.theta>>s.core.phi>>s.core.alpha;
      if(!row)throw std::runtime_error("Invalid pose row");
      s.split="video40";s.target_degrees=-90;s.q_sweep=true;s.q_best_effort=best_effort;s.arm_base_z=-.10;s.assembly_translation.x()=.20;
      s.start<<-.58298870078787879,1.4014363481415861,1.4956286729240151,.51538441396359136,-1.5264021873645037,-.58813609807777278,2.7857137959190528;
      requested.push_back(s);
    }
    if(requested.empty())throw std::runtime_error("No requested poses");
    std::filesystem::create_directories(output/"trajectories");
    if(online) {run_online(output,requested,retry);return 0;}
    std::filesystem::create_directories(output/"trials/trajectories");
    auto runs=output_file(output/"runs.csv"),trials=output_file(output/"trials/runs.csv"),scan=output_file(output/"ready_search.csv"),summary=output_file(output/"case_summary.csv");
    runs<<run_header;trials<<run_header;
    scan<<"case,attempt,ready_ik_success,start_clear,reason,seed0,seed1,seed2,seed3,seed4,seed5,seed6,q0,q1,q2,q3,q4,q5,q6\n";
    summary<<"case,success,ready_attempts,planning_attempts,selected_attempt,search_wall_seconds,selected_planning_seconds,execution_seconds,reason\n";
    PlanningScene scene;std::vector<Scenario> chosen,trial_scenarios;
    JointVector warm=requested.front().start;
    for(std::size_t index=0;index<requested.size();++index) {
      const auto begin=Clock::now();auto selected=requested[index];
      PlanResult result;result.method=Method::RigidCache;result.reason="ready_search_exhausted";
      ValidationResult dense;bool rechecked=false,accepted=false;double dense_seconds=0;
      int ready_count=0,planning_count=0,selected_attempt=-1;
      std::mt19937 generator(20261013+index);std::normal_distribution<double> normal(0,1);
      scene.configure(selected);
      const bool front=scene.core_within_front_plane();
      if(!front)result.reason="invalid_station_front_plane";
      for(int attempt=0;front && attempt<max_ready && planning_count<max_plans && !accepted;++attempt) {
        auto s=requested[index];scene.configure(s);
        auto target=scene.mechanism_target({});target.position+=.06*target.direction;
        const ForwardKinematics fk=[&](const JointVector& q,Eigen::Vector3d& p,Eigen::Matrix3d& r,Jacobian* j){scene.forward(q,p,r,j);};
        IKOptions ik;ik.method=IKMethod::FullStep;ik.max_iterations=600;ik.position_tolerance=1e-7;ik.axis_tolerance=1e-7;
        JointVector seed=attempt%2==0?s.start:warm;
        if(attempt>=2)for(int j=0;j<7;++j)seed[j]+=(attempt<32?.35:attempt<64?.7:1.2)*normal(generator);
        const auto ready=solve_ik(fk,target,seed,scene.limits(),ik);s.start=ready.q;scene.configure(s);
        const auto clearance=scene.collision(s.start,{});++ready_count;
        scan<<csv(s.id)<<','<<attempt<<','<<ready.success<<','<<clearance.valid<<','<<csv(ready.success?clearance.pair:ready.reason);
        for(int j=0;j<7;++j)scan<<','<<seed[j];for(int j=0;j<7;++j)scan<<','<<ready.q[j];scan<<'\n';scan.flush();
        if(!ready.success || !clearance.valid)continue;
        ++planning_count;PlanOptions options;options.method=Method::RigidCache;options.motion_speed_scale=4;options.timeout_seconds=3;
        result=safe_plan(scene,s,options);dense={};dense_seconds=0;rechecked=false;
        if(result.success) {
          auto extra=options;extra.validation_joint_step*=.5;extra.validation_min_substeps*=2;
          scene.configure(s);const auto d=Clock::now();rechecked=true;dense=validate_trajectory(scene,result.knots,extra);dense_seconds=elapsed(d);
        }
        accepted=result.success && rechecked && dense.valid;selected=s;selected_attempt=attempt;
        auto trial=s;trial.id+="_try"+std::to_string(attempt);trial_scenarios.push_back(trial);
        const auto path="trajectories/rigid_cache_"+trial.id+"_r0.csv";
        write_trajectory(output/"trials"/path,result);write_run(trials,trial,result,dense,rechecked,dense_seconds,path);
        std::cout<<s.id<<" attempt="<<attempt<<" success="<<accepted<<" plan_s="<<result.planning_seconds<<" reason="<<(result.success?dense.reason:result.reason)<<std::endl;
      }
      chosen.push_back(selected);if(accepted)warm=selected.start;
      const auto path="trajectories/rigid_cache_"+selected.id+"_r0.csv";write_trajectory(output/path,result);
      write_run(runs,selected,result,dense,rechecked,dense_seconds,path);
      summary<<csv(selected.id)<<','<<accepted<<','<<ready_count<<','<<planning_count<<','<<selected_attempt<<','<<elapsed(begin)<<','<<result.planning_seconds<<','
             <<(accepted?result.knots.back().time:0)<<','<<csv(result.success?dense.reason:result.reason)<<'\n';summary.flush();
      write_scenarios(output/"scenarios.csv",chosen);write_scenarios(output/"trials/scenarios.csv",trial_scenarios);
      std::cout<<"CASE_COMPLETE "<<index+1<<'/'<<requested.size()<<" "<<selected.id<<" success="<<accepted<<" ready="<<ready_count<<" plans="<<planning_count<<std::endl;
    }
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
