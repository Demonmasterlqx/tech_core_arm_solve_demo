#include "tech_core/arm_kinematics.hpp"
#include "tech_core/arm_simulation.hpp"
#include "tech_core/planning_demo.hpp"
#include <iostream>
#include <stdexcept>

using namespace tech_core;
using namespace tech_core::planning;
namespace {
void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
template<class F> void rejects(F action) {
    bool rejected=false;
    try { action(); } catch(const std::invalid_argument&) { rejected=true; }
    require(rejected,"Invalid dimension must be rejected before Eigen indexing");
}
void synthetic(int n) {
    JointLimits limits(n); limits.lower.setConstant(-1); limits.upper.setConstant(1);
    ForwardKinematics fk=[](const JointVector& q, Eigen::Vector3d& p, Eigen::Matrix3d& r, Jacobian* j) {
        p=q.head<3>();r.setIdentity();
        if(j) {j->setZero(6,q.size());j->topLeftCorner<3,3>().setIdentity();}
    };
    TaskTarget target; target.position={.15,-.12,.08};
    for(auto method:{IKMethod::Baseline,IKMethod::FullStep,IKMethod::Reduced5D,IKMethod::AdaptiveLM,IKMethod::BoundedQP}) {
        IKOptions options;options.method=method;
        auto result=solve_ik(fk,target,JointVector::Zero(n),limits,options);
        require(result.success && result.q.size()==n,"All IK backends must support arbitrary joint counts");
        options.max_joint_step=JointVector::Ones(n+1);
        require(!solve_ik(fk,target,JointVector::Zero(n),limits,options).success,"Wrong step dimensions accepted");
    }
    require(!solve_ik(fk,target,JointVector::Zero(n+1),limits).success,"Wrong limit dimensions accepted");
    ForwardKinematics malformed=[](const auto&,auto& p,auto& r,auto* j){p.setZero();r.setIdentity();if(j)j->setZero(6,1);};
    require(!solve_ik(malformed,target,JointVector::Zero(n),limits).success,"Malformed FK Jacobian accepted");
}
void robot(const std::string& id,int n) {
    ArmSimulation control(robot_model_path(id));
    ArmKinematics arm(robot_model_path(id));
    require(arm.home().size()==n && int(control.joint_names().size())==n,"Robot joint count");
    auto q=arm.home();
    Eigen::Vector3d p;Eigen::Matrix3d r;Jacobian j;
    arm.forward(q,p,r,&j);
    require(j.rows()==6 && j.cols()==n,"Runtime Jacobian dimensions");
    for(int i=0;i<n;++i) {
        auto plus=q,minus=q;plus[i]+=1e-6;minus[i]-=1e-6;
        Eigen::Vector3d pp,pm;Eigen::Matrix3d rp,rm;
        arm.forward(plus,pp,rp);arm.forward(minus,pm,rm);
        require(((pp-pm)/2e-6-j.col(i).head<3>()).norm()<1e-7,"Jacobian finite difference");
    }
    TaskTarget target;target.position=p;target.direction=r.col(2);
    auto seed=q;seed[0]+=.025;
    for(auto method:{IKMethod::Baseline,IKMethod::FullStep,IKMethod::Reduced5D,IKMethod::AdaptiveLM,IKMethod::BoundedQP}) {
        IKOptions options;options.method=method;
        const auto result=arm.inverse(target,seed,options);
        require(result.success && result.q.size()==n,"Real robot FK/IK round trip");
    }
    rejects([&]{arm.forward(JointVector::Zero(n+1),p,r);});
    rejects([&]{control.set_joint_positions(std::vector<double>(n+1,0));});
    control.set_joint_velocities(std::vector<double>(n,.02));
    for(int i=0;i<50;++i)control.step();
    require(control.finite(),"Robot velocity control");
    control.set_cartesian_velocity({.01,0,0});
    for(int i=0;i<50;++i)control.step();
    require(control.finite(),"Robot Cartesian control");
    PlanningScene scene(robot_model_path(id,true));
    auto scenario=scene.default_scenario();
    scene.configure(scenario);scene.forward(scenario.start,p,r,&j);
    require(scene.dof()==n && j.cols()==n,"Task scene dimensions");
    const auto plain=scene.collision(scenario.start,{});
    scene.enable_rigid_pair_cache(true);scene.enable_exact_state_cache(true);
    const auto cached=scene.collision(scenario.start,{});
    require(plain.valid==cached.valid,"Robot collision cache equivalence");
    scene.collision(scenario.start,{});
    require(scene.counters.exact_state_cache_hits==1,"Dynamic collision key");
    require(!scene.collision(JointVector::Zero(n+1),{}).valid,"Wrong collision dimensions accepted");
    scenario.start=JointVector::Zero(n+1);
    rejects([&]{scene.configure(scenario);});
    if(id=="mec_arm") {
        PlanningDemo demo(scene.default_scenario());
        require(demo.joints().size()==n,"Demo must load selected model");
        demo.home();demo.reset();
        require(demo.joints().size()==n,"Home/reset preserves selected dimensions");
    }
}
}
int main() {
    try {
        synthetic(3);synthetic(8);
        robot("rm26",7);robot("mec_arm",6);
        rejects([]{robot_model_path("missing_robot");});
        std::vector<Knot> knots(2);knots[0].q=JointVector::Zero(6);knots[1].q=JointVector::Constant(6,.1);
        parameterize(knots,1,std::vector<double>(6,.25));
        require(1.5*.1/knots.back().time<=.25+1e-12,"Per-robot mock rate limit");
        std::cout<<"Multi-arm contracts passed (3/6/7/8 DOF, IK, control, collision, demo, invalid input)\n";
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
