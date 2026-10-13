#include "tech_core/arm_kinematics.hpp"
#include "tech_core/arm_simulation.hpp"
#include "tech_core/planning_demo.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <stdexcept>

using namespace tech_core;
using namespace tech_core::planning;
namespace {
void require(bool ok, const std::string& message) {
    if (!ok) throw std::runtime_error(message);
}
}

int main(int argc, char** argv) {
    try {
        require(argc==2,"Expected exported robot id");
        const std::string id=argv[1];
        const int n=id=="zhuimeng_dual"?7:6;
        ArmKinematics arm(robot_model_path(id));
        ArmSimulation control(robot_model_path(id));
        require(arm.home().size()==n && int(control.joint_names().size())==n,"Selected arm DOF");
        require(arm.model()->njnt==n,"Gripper/unused arm became a planning DOF");
        require(mj_name2id(arm.model(),mjOBJ_BODY,"carried_unit")>=0,"Control demo lost carried unit");
        if(n==7) require(mj_name2id(arm.model(),mjOBJ_BODY,"right_link_1")==-1,"Unused right arm retained");
        Eigen::Vector3d p;Eigen::Matrix3d r;Jacobian j;
        arm.forward(arm.home(),p,r,&j);
        TaskTarget target;target.position=p;target.direction=r.col(2);
        auto seed=arm.home();seed[0]+=.02;
        for(auto method:{IKMethod::Baseline,IKMethod::FullStep,IKMethod::Reduced5D,IKMethod::AdaptiveLM,IKMethod::BoundedQP}) {
            IKOptions options;options.method=method;
            require(arm.inverse(target,seed,options).success,"FK/IK round trip");
        }
        control.set_joint_velocities(std::vector<double>(n,.03));
        for(int i=0;i<30;++i)control.step();
        require(control.finite(),"Nonfinite control state");

        PlanningScene scene(robot_model_path(id,true));
        const auto scenario=scene.default_scenario();
        require(scene.dof()==n && scene.model()->njnt==n+3,"Assembly DOF");
        require(scene.robot().demo_start_at_preset,"CAD demo must start at calibrated preset");
        scene.configure(scenario);
        const int core=mj_name2id(scene.model(),mjOBJ_BODY,"core0_base");
        const Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> core_rotation(scene.data()->xmat+9*core);
        require((core_rotation.col(2)-Eigen::Vector3d::UnitZ()).norm()<1e-10,"Core must start upright");
        scene.forward(scenario.start,p,r,&j);
        const auto ready=make_task(scene,PlanOptions{}).front().target;
        require((p-ready.position).norm()<.0005 && (r.col(2)-ready.direction).norm()<.002,"Initial relative pose is not aligned at pre-insertion");
        const auto collision=scene.collision(scenario.start,{});
        require(collision.valid,"Initial collision: "+collision.pair);
        // BoundedQP covers the tight flange seating case where interpolation
        // between accurate IK endpoints crossed the 0.1 mm contact envelope.
        for(auto method:{Method::Baseline,Method::RigidCache,Method::BoundedQP}) {
            PlanOptions options;options.method=method;options.timeout_seconds=20;
            const auto result=plan(scene,scenario,options);
            require(result.success && result.validation.rule_success,"Assembly failed: "+result.reason);
            require((result.knots.front().q-scenario.start).norm()<1e-12,"Planner changed actual start");
            scene.configure(scenario);
            options.validation_joint_step*=.5;options.validation_min_substeps*=2;
            const auto replay=validate_trajectory(scene,result.knots,options);
            require(replay.valid && replay.rule_success,"Uncached dense replay: "+replay.reason);
            std::cout<<id<<" "<<method_name(method)<<" samples="<<replay.samples<<" error="<<replay.max_position_error<<'\n';
        }
        auto blocked=scenario;blocked.blocking_obstacle=true;
        PlanOptions options;options.method=Method::RigidCache;options.timeout_seconds=1;
        require(!plan(scene,blocked,options).success,"Blocking obstacle accepted");
        auto unreachable=scenario;unreachable.assembly_translation.x()+=3;
        require(!plan(scene,unreachable,options).success,"Unreachable target accepted");
        PlanningDemo demo(scenario);
        require((demo.joints()-scenario.start).norm()<1e-12,"Initial demo differs from calibrated pose");
        demo.home();require(demo.joints().norm()<1e-12,"CAD active-joint zero lost");
        demo.reset();require((demo.joints()-scenario.start).norm()<1e-12,"Reset lost model preset");
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
