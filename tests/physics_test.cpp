#include "tech_core/simulation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using namespace tech_core;
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string& what) {
    require(std::isfinite(actual) && std::abs(actual-expected)<=tolerance,
            what+": expected "+std::to_string(expected)+", got "+std::to_string(actual));
}
int id(const mjModel* model, mjtObj type, const std::string& name) {
    const int result=mj_name2id(model,type,name.c_str());
    require(result>=0,"Missing model object "+name);
    return result;
}
void joint(Simulation& sim, const std::string& name, double value) {
    const int j=id(sim.model(),mjOBJ_JOINT,name);
    sim.data()->qpos[sim.model()->jnt_qposadr[j]]=value;
}
void local_fixture(Simulation& sim) {
    mj_resetData(sim.model(),sim.data());
    for(int core=0;core<2;++core) {
        const int b=id(sim.model(),mjOBJ_BODY,"core"+std::to_string(core)+"_base");
        const int mocap=sim.model()->body_mocapid[b];
        mju_zero(sim.data()->mocap_pos+3*mocap,3);
        sim.data()->mocap_pos[3*mocap+1]=core*1.0;
        sim.data()->mocap_pos[3*mocap+2]=1.0; // Keep test fixture above the ground.
        sim.data()->mocap_quat[4*mocap]=1;
        mju_zero(sim.data()->mocap_quat+4*mocap+1,3);
    }
    mj_forward(sim.model(),sim.data());
}
void no_warnings(const Simulation& sim) {
    for(int i=0;i<mjNWARNING;++i)
        require(sim.data()->warning[i].number==0,"MuJoCo warning "+std::to_string(i));
}

void orientation_contract() {
    // Table 5-10's normal is e=(sin(phi)cos(theta),sin(phi)sin(theta),cos(phi)).
    // Check pole cases and both extremes, including nonzero axial roll.
    for(double theta:{-90.,0.,90.}) for(double phi:{0.,45.,90.})
        for(double alpha:{-45.,0.,45.}) {
            Pose p; p.theta=theta; p.phi=phi; p.alpha=alpha;
            validate_pose(p,3);
            auto q=pose_quaternion(p);
            double norm=0;for(double v:q) norm+=v*v;
            near(norm,1,1e-12,"Quaternion norm");
            mjtNum matrix[9];mju_quat2Mat(matrix,q.data());
            near(matrix[0],std::sin(radians(phi))*std::cos(radians(theta)),1e-12,"Normal x");
            near(matrix[3],std::sin(radians(phi))*std::sin(radians(theta)),1e-12,"Normal y");
            near(matrix[6],std::cos(radians(phi)),1e-12,"Normal z");
        }
    SimulationConfig config;
    config.pose={-.1,.25,.65,25,75,-15};
    Simulation sim(config);
    sim.data()->time=1.0;
    sim.step();
    const int b=id(sim.model(),mjOBJ_BODY,"core0_base");
    const auto& p=config.pose;
    near(sim.data()->xpos[3*b],p.x,1e-12,"Station x after settling");
    near(sim.data()->xpos[3*b+1],p.y,1e-12,"Station y after settling");
    near(sim.data()->xpos[3*b+2],p.z,1e-12,"Station z after settling");
    auto q=pose_quaternion(p);mjtNum expected[9];mju_quat2Mat(expected,q.data());
    for(int j=0;j<9;++j) near(sim.data()->xmat[9*b+j],expected[j],1e-12,"Actual station orientation");
    require(sim.finite(),"Orientation fixture produced nonfinite state");
    no_warnings(sim);
    // A legal table coordinate can still put the physical station across X=0.
    // The whole geometry restriction must therefore remain an independent check.
    auto crossing=config;
    crossing.pose={-.04,.25,.65,35,40,-25};
    validate_pose(crossing.pose,crossing.level);
    bool rejected=false;
    try {Simulation invalid(crossing);} catch(const std::invalid_argument&) {rejected=true;}
    require(rejected,"Range-legal geometry crossing base-front plane must be rejected");
}

void geometry_and_contacts() {
    Simulation sim;
    local_fixture(sim);
    const auto* m=sim.model();auto* d=sim.data();
    require(m->nq==8 && m->nv==8 && m->nu==8 && m->nmocap==2,"Two four-axis fixtures required");
    for(int core=0;core<2;++core) {
        const auto prefix="core"+std::to_string(core)+"_";
        int peg=id(m,mjOBJ_GEOM,prefix+"peg");
        near(m->geom_size[3*peg],.018,1e-12,"36mm pillar radius");
        near(m->geom_size[3*peg+1],.05,1e-12,"100mm exposed pillar half-length");
        int unit=id(m,mjOBJ_BODY,prefix+"energy_unit");
        near(m->body_mass[unit],.4,1e-12,"Energy unit mass");
        int cap=id(m,mjOBJ_GEOM,prefix+"unit_cap");
        near(m->geom_size[3*cap],.0475,1e-12,"95mm upper flange radius");
        int tip=id(m,mjOBJ_SITE,prefix+"tip");
        near(d->site_xpos[3*tip],0,1e-12,"Initial E x");
        near(d->site_xpos[3*tip+1],core,1e-12,"Initial E y");
        near(d->site_xpos[3*tip+2],1,1e-12,"Initial E z");
        for(const auto* suffix:{"insert","lift"}) {
            const int j=id(m,mjOBJ_JOINT,prefix+suffix);
            near(m->jnt_range[2*j],0,1e-12,"Linear lower stop");
            near(m->jnt_range[2*j+1],.1,1e-12,"100mm travel");
            near(m->dof_frictionloss[m->jnt_dofadr[j]],10,1e-12,"10N guide resistance assumption");
        }
    }
    // Raycast the actual collision mesh from inside the bore: diameter40mm.
    const mjtNum point[3]={.05,0,1}, radial[3]={0,1,0};
    int hit=-1;
    near(mj_ray(m,d,point,radial,nullptr,1,-1,&hit),.020,2e-7,"Actual bore inner radius");
    require(hit>=0,"Bore ray must hit an energy-unit wall");
    const mjtNum axial_point[3]={.001,0,1}, axial[3]={1,0,0};
    near(mj_ray(m,d,axial_point,axial,nullptr,1,-1,&hit),.099,2e-7,"100mm blind bore depth");
    joint(sim,"core0_insert",.08);mj_forward(m,d);
    require(d->ncon==0,"Centered80mm insertion should clear the hollow bore");
    const int peg=id(m,mjOBJ_GEOM,"core0_peg");
    sim.model()->geom_pos[3*peg+1]=.006;
    mj_forward(m,d);
    require(d->ncon>0,"6mm peg misalignment must create physical wall contact");
    sim.model()->geom_pos[3*peg+1]=0;
    joint(sim,"core0_insert",.102);mj_forward(m,d);
    require(d->ncon>0,"Insertion beyond100mm must contact the blind bore stop");
    no_warnings(sim);
}

void nested_axis_kinematics() {
    Simulation sim;
    local_fixture(sim);
    joint(sim,"core0_lift",.1);
    joint(sim,"core0_flip",pi/2);
    joint(sim,"core0_twist",pi/6);
    joint(sim,"core0_insert",.1);
    mj_forward(sim.model(),sim.data());
    const int tip=id(sim.model(),mjOBJ_SITE,"core0_tip");
    const auto* p=sim.data()->site_xpos+3*tip;
    // R_Q(+X,30deg)*R_P(-Y,90deg)*(.169,0,-.108), plus pivot.
    // Q must sweep the flipped tip laterally instead of spinning its own axis.
    near(p[0],-.169+.108,1e-12,"P/Q tip x");
    near(p[1],-.169*.5,1e-12,"Q30 lateral tip sweep");
    near(p[2],1+.1+.108+.169*std::cos(pi/6),1e-12,"P/Q tip z");
    const int opening=id(sim.model(),mjOBJ_SITE,"core0_opening");
    const auto* o=sim.data()->site_xpos+3*opening;
    near(o[0],p[0],1e-12,"Insertion follows flipped axis x");
    near(o[1],p[1]+.1*.5,1e-12,"Insertion follows flipped axis y");
    near(o[2],p[2]-.1*std::cos(pi/6),1e-12,"Insertion follows flipped axis z");
}

void reject_nonfinite_commands() {
    Simulation sim;
    for(double bad:{std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::infinity(),
                    -std::numeric_limits<double>::infinity()}) {
        for(int component=0;component<4;++component) {
            Command c;
            if(component==0)c.insertion=bad;
            if(component==1)c.lift=bad;
            if(component==2)c.flip=bad;
            if(component==3)c.twist=bad;
            bool rejected=false;
            try {sim.set_command(0,c);} catch(const std::invalid_argument&) {rejected=true;}
            require(rejected,"Nonfinite command must be rejected before entering MuJoCo");
        }
    }
    sim.set_command(0,{-1,2,20,-20});
    auto c=sim.command(0);
    near(c.insertion,0,0,"Insertion clamp");near(c.lift,.1,0,"Lift clamp");
    near(c.flip,pi/2,0,"P clamp");near(c.twist,-pi/2,0,"Q clamp");
}

void bounded_torques_and_release() {
    Simulation sim;
    auto check=[&] {
        require(sim.finite(),"Dynamic state must remain finite");
        for(int i=0;i<2;++i) {
            require(std::abs(sim.return_torque(i))<=.3+1e-12,"Q return torque exceeds0.3Nm");
            require(std::abs(sim.holding_torque(i))<=.5+1e-12,"Q hold torque exceeds0.5Nm");
        }
    };
    // Stop before the demo's confirmation. This independently tests the stated
    // post-dwell gripper-release behavior; level completion is covered by CTest.
    while(sim.time()<11.0) {sim.drive_demo();sim.step();check();}
    require(sim.rules().safety_locked(0),"Correct Q pose held>2s must enable safety hold");
    sim.set_gripper_enabled(false);
    const double until=sim.time()+.5;
    while(sim.time()<until) {sim.step();check();}
    const auto o=sim.observation(0);
    near(o.insertion,.1,.003,"Released energy remains inserted");
    near(o.lift,.1,.003,"Released carriage remains raised");
    near(o.flip,pi/2,radians(2),"Released P remains flipped");
    near(o.twist,radians(sim.config().target_degrees[0]),radians(5),"Released Q remains accepted");
    no_warnings(sim);
}
} // namespace

int main() {
    try {
        orientation_contract();
        geometry_and_contacts();
        nested_axis_kinematics();
        reject_nonfinite_commands();
        bounded_torques_and_release();
        std::cout<<"Physics contracts passed: orientation, geometry, contact, axes, input validation, torque/release\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<"Physics contract failure: "<<error.what()<<'\n';
        return 1;
    }
}
