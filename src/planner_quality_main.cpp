#include "tech_core/planning.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace tech_core;
using namespace tech_core::planning;
namespace {
std::string quote(const std::string& text) {
    std::string out="\"";
    for(char c:text) {if(c=='"')out+='"';out+=c;}
    return out+'"';
}
std::ofstream file(const std::filesystem::path& path) {
    std::ofstream out(path);
    if(!out)throw std::runtime_error("Cannot write "+path.string());
    out.exceptions(std::ios::failbit|std::ios::badbit);
    out<<std::setprecision(17);return out;
}
void trajectory(const std::filesystem::path& path,const PlanResult& result) {
    auto out=file(path);
    out<<"knot,time,phase,insertion,lift,flip,twist,q0,q1,q2,q3,q4,q5,q6,target_x,target_y,target_z,direction_x,direction_y,direction_z\n";
    for(std::size_t i=0;i<result.knots.size();++i) {
        const auto& k=result.knots[i];
        out<<i<<','<<k.time<<','<<phase_name(k.phase)<<','<<k.mechanism.insertion<<','<<k.mechanism.lift
           <<','<<k.mechanism.flip<<','<<k.mechanism.twist;
        for(double q:k.q)out<<','<<q;
        for(double x:k.target.position)out<<','<<x;
        for(double x:k.target.direction)out<<','<<x;
        out<<'\n';
    }
}
}
int main(int argc,char** argv) {
    try {
        if(argc!=6)throw std::runtime_error("Usage: benchmark_planner_quality SUITE.csv OUTPUT METHOD SPLIT TIMEOUT");
        const auto method=parse_method(argv[3]);const std::string split=argv[4];
        const std::filesystem::path output=argv[2];
        if(std::filesystem::exists(output))throw std::runtime_error("Output already exists");
        std::filesystem::create_directories(output/"trajectories");
        std::ifstream input(argv[1]);if(!input)throw std::runtime_error("Missing suite");
        auto out=file(output/"runs.csv");
        out<<"case,split,group,start,method,success,plan_success,reason,seconds,ik_seconds,approach_seconds,validation_seconds,knots,q_positive,q_negative,q_travel,full_q,positive_stop,negative_stop,total_l1,mandatory_l1,approach_l1,approach_last_joint_l1,assembly_l1,max_joint_step,audit_valid,audit_reason,audit_samples,posture_candidates,planning_attempts,preinsert_ik_attempts,posture_search_seconds,ik_iterations,fk_evals,collision_states,narrowphase_calls,trajectory\n";
        PlanningScene scene;
        PlanOptions options;options.method=method;options.timeout_seconds=std::stod(argv[5]);options.motion_speed_scale=4;
        std::string line;std::getline(input,line);
        while(std::getline(input,line)) {
            if(line.empty())continue;
            std::replace(line.begin(),line.end(),',',' ');std::istringstream row(line);
            Scenario s=scenario_suite().front();std::string group,start;
            row>>s.id>>s.split>>group>>start>>s.core.x>>s.core.y>>s.core.z>>s.core.theta>>s.core.phi>>s.core.alpha;
            if(!row)throw std::runtime_error("Invalid suite row");
            if(split!="all" && s.split!=split)continue;
            s.q_sweep=true;s.q_best_effort=true;s.target_degrees=-90;s.enforce_core_pose_limits=false;
            if(start=="zero")s.start.setZero();
            else if(start=="legacy") {
                s.arm_base_z=-.1;s.assembly_translation.x()=.2;
                s.start<<-.58298870078787879,1.4014363481415861,1.4956286729240151,.51538441396359136,
                    -1.5264021873645037,-.58813609807777278,2.7857137959190528;
            } else if(start!="preset")throw std::runtime_error("Unknown start");
            if(group=="blocked")s.blocking_obstacle=true;
            PlanResult r;
            try {r=plan(scene,s,options);}
            catch(const std::exception& error) {r.reason=std::string("exception: ")+error.what();}
            ValidationResult audit;
            if(r.success) {scene.configure(s);audit=validate_trajectory(scene,r.knots,options);}
            double total=0,mandatory=0,approach=0,approach_last=0,assembly=0,max_step=0;
            for(std::size_t i=1;i<r.knots.size();++i) {
                const auto delta=(r.knots[i].q-r.knots[i-1].q).cwiseAbs().eval();
                total+=delta.sum();max_step=std::max(max_step,delta.maxCoeff());
                if(r.knots[i].phase<Phase::Twist)mandatory+=delta.sum();
                if(r.knots[i].phase==Phase::Approach) {approach+=delta.sum();approach_last+=delta[6];}
                else if(r.knots[i].phase<Phase::Twist)assembly+=delta.sum();
            }
            const std::string path="trajectories/"+s.id+".csv";
            trajectory(output/path,r);
            const bool success=r.success && audit.valid;
            out<<s.id<<','<<s.split<<','<<group<<','<<start<<','<<method_name(method)<<','<<success<<','<<r.success
               <<','<<quote(r.reason)<<','<<r.planning_seconds<<','<<r.ik_seconds<<','<<r.approach_seconds
               <<','<<r.validation_seconds<<','<<r.knots.size()<<','<<r.q_positive_degrees<<','<<r.q_final_degrees
               <<','<<2*r.q_positive_degrees-r.q_final_degrees<<','<<r.full_q_sweep<<','<<quote(r.q_positive_stop)
               <<','<<quote(r.q_negative_stop)<<','<<total<<','<<mandatory<<','<<approach<<','<<approach_last<<','<<assembly<<','<<max_step
               <<','<<audit.valid<<','<<quote(audit.reason)<<','<<audit.samples
               <<','<<r.posture_candidates<<','<<r.planning_attempts<<','<<r.preinsert_ik_attempts<<','<<r.posture_search_seconds
               <<','<<r.counters.ik_iterations<<','<<r.counters.fk_evals<<','<<r.counters.collision_states
               <<','<<r.counters.narrowphase_calls<<','<<path<<'\n';out.flush();
            std::cout<<s.id<<" success="<<success<<" time="<<r.planning_seconds<<" mandatory_l1="<<mandatory
                     <<" Q="<<r.q_positive_degrees<<'/'<<r.q_final_degrees<<" reason="<<r.reason<<std::endl;
            // Keep every request in the denominator, including a cached/uncached
            // disagreement. Such a result is recorded as an audited failure.
        }
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
