#include "tech_core/planning.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace tech_core::planning;
namespace {
using RowMatrix=Eigen::Matrix<double,3,3,Eigen::RowMajor>;
void require(bool value,const std::string& reason){if(!value)throw std::runtime_error(reason);}
int id(const mjModel*m,mjtObj type,const char* name){int i=mj_name2id(m,type,name);require(i>=0,std::string("Missing ")+name);return i;}

void frame_and_ground(PlanningScene& scene){
    Scenario s;s.assembly_translation={.30,.20,-.25};scene.configure(s);
    const int ee=id(scene.model(),mjOBJ_SITE,"arm_tool"),tcp=id(scene.model(),mjOBJ_SITE,"planning_tcp");
    const int unit=id(scene.model(),mjOBJ_BODY,"carried_unit"),ground=id(scene.model(),mjOBJ_GEOM,"ground_visual");
    for(int k=0;k<12;++k){
        JointVector q=s.start;q[0]=-.4+.08*k;q[3]=-.3+.04*k;q[4]=-.2+.07*k;
        scene.set_state(q,{});
        const Eigen::Vector3d ep=Eigen::Map<const Eigen::Vector3d>(scene.data()->site_xpos+3*ee);
        const Eigen::Matrix3d er=Eigen::Map<const RowMatrix>(scene.data()->site_xmat+9*ee);
        const Eigen::Vector3d tp=Eigen::Map<const Eigen::Vector3d>(scene.data()->site_xpos+3*tcp);
        const Eigen::Matrix3d tr=Eigen::Map<const RowMatrix>(scene.data()->site_xmat+9*tcp);
        const Eigen::Matrix3d ur=Eigen::Map<const RowMatrix>(scene.data()->xmat+9*unit);
        require((tp-ep-er*Eigen::Vector3d(0,-.015,-.010)).norm()<1e-10,"Opening must include the fixed side/axial grasp offset");
        require((tr.col(2)-ur.col(0)).norm()<1e-10,"Planning TCP Z must follow actual unit X");
        require(std::abs(scene.data()->geom_xpos[3*ground+2]+.25)<1e-12,"Ground must follow assembly floor height");
    }
    s.assembly_translation.z()=-.12;scene.configure(s);
    require(std::abs(scene.data()->geom_xpos[3*ground+2]+.12)<1e-12,"Reconfiguration must update ground, not leave prior platform height");
}

void hollow_bore(PlanningScene& scene){
    Scenario s;scene.configure(s);auto*m=scene.model();auto*d=scene.data();
    const int unit=id(m,mjOBJ_BODY,"carried_unit"),base=id(m,mjOBJ_BODY,"core0_base");
    const int mocap=m->body_mocapid[base];
    const Eigen::Vector3d p=Eigen::Map<const Eigen::Vector3d>(d->xpos+3*unit);
    const Eigen::Matrix3d r=Eigen::Map<const RowMatrix>(d->xmat+9*unit);
    const Eigen::Quaterniond quat(r);
    const int peg=id(m,mjOBJ_GEOM,"core0_peg"),wall=id(m,mjOBJ_GEOM,"carried_unit_tube_0");
    const int iron=id(m,mjOBJ_GEOM,"carried_unit_iron");
    auto align=[&](double radial){
        Eigen::Map<Eigen::Vector3d>(d->mocap_pos+3*mocap)=p+r*Eigen::Vector3d(.1,radial,0);
        d->mocap_quat[4*mocap]=quat.w();d->mocap_quat[4*mocap+1]=quat.x();
        d->mocap_quat[4*mocap+2]=quat.y();d->mocap_quat[4*mocap+3]=quat.z();
        mj_forward(m,d);
    };
    align(0);
    const double nominal=mj_geomDistance(m,d,peg,wall,.003,nullptr);
    const double stop=mj_geomDistance(m,d,peg,iron,.001,nullptr);
    require(nominal>.0015,"Nominal 36 mm peg must fit in the 40 mm hollow bore");
    require(std::abs(stop)<1e-5,"Fully inserted peg must touch the 100 mm iron stop");
    align(.003);
    const double wrong=mj_geomDistance(m,d,peg,wall,.003,nullptr);
    require(wrong<-.0005,"3 mm radial misalignment must penetrate a real bore wall, not be blanket-exempted");
    std::cout<<"bore clearance="<<nominal<<" stop="<<stop<<" offset penetration="<<wrong<<'\n';
}

void physical_base_translation(PlanningScene& scene){
    auto s=scenario_suite().front();scene.configure(s);
    const int ground=id(scene.model(),mjOBJ_GEOM,"ground_visual"),mount=id(scene.model(),mjOBJ_GEOM,"mount_visual");
    const double floor_z=scene.data()->geom_xpos[3*ground+2],mount_z=scene.data()->geom_xpos[3*mount+2];
    Eigen::Vector3d p0,p1;Eigen::Matrix3d r0,r1;Jacobian j0,j1;
    scene.forward(s.start,p0,r0,&j0);const auto target=scene.mechanism_target({});
    s.arm_base_z=-.1;scene.configure(s);scene.configure(s);
    scene.forward(s.start,p1,r1,&j1);
    require((p1-p0-Eigen::Vector3d(0,0,-.1)).norm()<1e-12,"Physical base lowering must move TCP exactly 100 mm without accumulating");
    require((r1-r0).norm()<1e-12 && (j1-j0).norm()<1e-10,"Pure base translation must preserve orientation and point Jacobian");
    require((scene.mechanism_target({}).position-target.position).norm()<1e-12,"Lowering robot must not translate the core");
    require(std::abs(scene.data()->geom_xpos[3*ground+2]-floor_z)<1e-12,"Ground must stay fixed when robot lowers");
    require(std::abs(scene.data()->geom_xpos[3*mount+2]-mount_z+.1)<1e-12,"Fixed robot mount must move with the robot");
    s.arm_base_z=0;scene.configure(s);scene.forward(s.start,p1,r1);
    require((p1-p0).norm()<1e-12,"Reconfigure must restore original base position");
    s.arm_base_z=std::numeric_limits<double>::quiet_NaN();bool rejected=false;
    try{scene.configure(s);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"Reject nonfinite base placement");
    scene.configure(scenario_suite().front());
}

void split_wrist_hulls(PlanningScene& scene){
    Scenario s;s.start<<0,1,1.4,0,.7,0,0;scene.configure(s);
    auto*m=scene.model();auto*d=scene.data();
    const double original=mj_geomDistance(m,d,id(m,mjOBJ_GEOM,"j4_link_visual"),id(m,mjOBJ_GEOM,"j6_link_visual"),.001,nullptr);
    require(original<-.01,"Fixture must retain the original whole-mesh convex-hull false positive");
    for(int a=0;a<6;++a)for(int b=0;b<5;++b){
        const std::string an="collision_arm_j4_link_part_"+std::to_string(a),bn="collision_arm_j6_link_part_"+std::to_string(b);
        const double distance=mj_geomDistance(m,d,id(m,mjOBJ_GEOM,an.c_str()),id(m,mjOBJ_GEOM,bn.c_str()),.001,nullptr);
        require(distance>=.001-1e-8,"Source component split must preserve the differential-wrist cavity");
    }
}

void contact_boundaries(PlanningScene& scene){
    const Scenario scenario=scenario_suite().front();scene.configure(scenario);
    for(double clearance:{-.001,std::numeric_limits<double>::quiet_NaN()}) {
        const auto result=scene.collision(scenario.start,{},clearance);
        require(!result.valid && result.pair=="invalid_clearance","Invalid clearance must fail before geometric checks");
    }
    const ForwardKinematics fk=[&](const JointVector&q,Eigen::Vector3d&p,Eigen::Matrix3d&r,Jacobian*j){scene.forward(q,p,r,j);};
    JointVector q=scenario.start;MechanismState state;
    IKOptions options;options.position_tolerance=1e-7;options.axis_tolerance=1e-7;
    for(int step=0;step<=54;++step){
        state.insertion=std::min(.1,.002*step);state.lift=std::max(0.,.002*(step-50));
        const auto ik=solve_ik(fk,scene.mechanism_target(state),q,scene.limits(),options);
        require(ik.success,"Contact fixture IK must converge: "+ik.reason);q=ik.q;
    }
    const auto nominal=scene.collision(q,state);
    require(nominal.valid,"Named seated surfaces must pass separating-plane certificate: "+nominal.pair);
    auto*m=scene.model();auto*d=scene.data();
    const int core=id(m,mjOBJ_BODY,"core0_base"),mocap=m->body_mocapid[core];
    const Eigen::Vector3d original=Eigen::Map<const Eigen::Vector3d>(d->mocap_pos+3*mocap);
    const Eigen::Vector3d axis=scene.mechanism_target(state).direction;
    Eigen::Map<Eigen::Vector3d>(d->mocap_pos+3*mocap)=original+.001*axis;
    const auto overlap=scene.collision(q,state);
    require(!overlap.valid && overlap.pair.find("assembly_axial_overlap")!=std::string::npos,
        "1 mm true axial intrusion must fail the same support certificate: "+overlap.pair);
    std::cout<<"assembly contact nominal="<<nominal.distance<<" intrusion="<<overlap.pair<<'\n';
    scene.configure(scenario);
}

void rotated_grasp_cache_equivalence(PlanningScene& scene) {
    // Fixed development input where native world-frame mesh CCD reported a
    // 0.444 mm penetration for a welded grasp pair only after disabling cache.
    auto s=scenario_suite().front();s.enforce_core_pose_limits=false;s.start.setZero();
    s.core={.03880536366456411,.1575398960575357,.5963354371411802,
            46.355362393817984,145.26359223887667,-85.18092988237612};
    JointVector a,b;
    a<<-.5349466535352002,1.3870617258143012,.7278698618865178,1.0237407407769206,
        -.3412093226212784,-.996550170064477,1.1448191658317814;
    b<<-.5319618535641634,1.3843030170561499,.73066714219504,1.0253480917056987,
        -.34075027216028103,-.9952616803338302,1.1506819146128535;
    scene.configure(s);
    for(int step=0;step<=32;++step) {
        const double u=step/32.;const JointVector q=a+u*(b-a);
        MechanismState mechanism;mechanism.insertion=.008+.002*u;
        scene.enable_rigid_pair_cache(true);
        const auto cached=scene.collision(q,mechanism);
        scene.enable_rigid_pair_cache(false);
        const auto direct=scene.collision(q,mechanism);
        require(cached.valid && direct.valid,"Rigid grasp distance must be independent of world pose: "+direct.pair);
        require(cached.pair==direct.pair && std::abs(cached.distance-direct.distance)<1e-9,
                "Cached and uncached checks must share one rigid-pair geometry contract");
    }
    std::cout<<"rotated fixed grasp: 33 cached/uncached checks agree\n";
}
}
int main(){try{PlanningScene scene;frame_and_ground(scene);physical_base_translation(scene);hollow_bore(scene);split_wrist_hulls(scene);contact_boundaries(scene);rotated_grasp_cache_equivalence(scene);std::cout<<"planning scene geometry tests passed\n";return 0;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
