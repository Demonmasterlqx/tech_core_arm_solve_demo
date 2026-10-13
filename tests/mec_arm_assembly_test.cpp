#include "tech_core/planning_demo.hpp"
#include <iostream>
#include <stdexcept>

using namespace tech_core;
using namespace tech_core::planning;
void require(bool ok,const std::string& message) {if(!ok)throw std::runtime_error(message);}
int main() {
    try {
        PlanningScene scene(robot_model_path("mec_arm",true));
        auto scenario=scene.default_scenario();
        for(auto method:{Method::Baseline,Method::RigidCache}) {
            PlanOptions options;options.method=method;options.timeout_seconds=15;
            const auto result=plan(scene,scenario,options);
            require(result.success,"MecArm full assembly failed: "+result.reason);
            require(result.validation.rule_success && result.knots.front().q.size()==6 &&
                    (result.knots.front().q-scenario.start).norm()<1e-12,"MecArm trajectory start/contract");
            scene.configure(scenario); // Disable both caches for independent replay.
            options.validation_joint_step*=.5;options.validation_min_substeps*=2;
            const auto dense=validate_trajectory(scene,result.knots,options);
            require(dense.valid,"MecArm dense replay failed: "+dense.reason);
            std::cout<<method_name(method)<<" knots="<<result.knots.size()<<" samples="<<dense.samples
                     <<" error_m="<<dense.max_position_error<<'\n';
            auto malformed=result.knots;malformed[1].q.conservativeResize(7);
            require(!validate_trajectory(scene,malformed,options).valid,"Mixed robot dimensions accepted");
        }
        auto blocked=scenario;blocked.blocking_obstacle=true;
        PlanOptions options;options.method=Method::RigidCache;options.timeout_seconds=2;
        require(!plan(scene,blocked,options).success,"Blocking obstacle accepted");
        auto unreachable=scenario;unreachable.assembly_translation.x()+=3;
        require(!plan(scene,unreachable,options).success,"Unreachable assembly accepted");
        auto demo_options=demo_plan_options();demo_options.timeout_seconds=10;
        const auto preview=make_demo_preview(scenario,demo_options);
        require(preview.result.success && !preview.candidate_path.empty(),"Selected robot async-preview workspace");
        require(preview.ghost_knots.front().q.size()==6,"Selected robot preview dimension");
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
