#include "tech_core/simulation.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace tech_core {
namespace {
double smooth(double x) { x = std::clamp(x, 0.0, 1.0); return x*x*(3-2*x); }
bool all_finite(std::initializer_list<double> values) {
    return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
}
}

std::array<double, 4> pose_quaternion(const Pose& p) {
    // Rz(theta) Ry(phi-90deg) Rx(alpha): the first column is e=(sin(phi)
    // cos(theta), sin(phi) sin(theta), cos(phi)). Intrinsic roll follows e.
    const double a=radians(p.theta)/2, b=radians(p.phi-90)/2, c=radians(p.alpha)/2;
    return {std::cos(a)*std::cos(b)*std::cos(c)+std::sin(a)*std::sin(b)*std::sin(c),
            std::cos(a)*std::cos(b)*std::sin(c)-std::sin(a)*std::sin(b)*std::cos(c),
            std::cos(a)*std::sin(b)*std::cos(c)+std::sin(a)*std::cos(b)*std::sin(c),
            std::sin(a)*std::cos(b)*std::cos(c)-std::cos(a)*std::sin(b)*std::sin(c)};
}

void validate_pose(const Pose& p, int level) {
    if (!all_finite({p.x,p.y,p.z,p.theta,p.phi,p.alpha}) ||
        p.x < -.1 || p.x > 0 || p.y < .1 || p.y > .3 || p.z < .5 || p.z > .7 ||
        p.theta < -90 || p.theta > 90 || p.phi < 0 || p.phi > 90 || p.alpha < -45 || p.alpha > 45)
        throw std::invalid_argument("Pose outside rule table 5-10 (metres and degrees)");
    if (level==1 && (p.theta!=0 || p.phi!=90 || p.alpha!=0))
        throw std::invalid_argument("Level 1 requires theta=0, phi=90, alpha=0");
}

int Simulation::lookup(mjtObj type, const std::string& name) const {
    int id=mj_name2id(model_.get(),type,name.c_str());
    if(id<0) throw std::runtime_error("Missing model object: "+name);
    return id;
}

Simulation::Simulation(SimulationConfig config):config_(std::move(config)) {
    validate_pose(config_.pose,config_.level);
    if(config_.level==4) {
        auto second=config_.pose;
        second.theta=config_.secondary_angles[0];second.phi=config_.secondary_angles[1];second.alpha=config_.secondary_angles[2];
        validate_pose(second,4);
    }
    if (!std::isfinite(config_.pair_spacing) || config_.pair_spacing<=0 || config_.pair_spacing>.4)
        throw std::invalid_argument("Pair spacing must be in (0, 0.4] metres");
    char error[2048]{};
    model_.reset(mj_loadXML(config_.model_path.c_str(),nullptr,error,sizeof(error)));
    if (!model_) throw std::runtime_error(std::string("MuJoCo model: ")+error);
    data_.reset(mj_makeData(model_.get()));
    if (!data_) throw std::runtime_error("Unable to allocate mjData");
    const char* suffix[]={"insert","lift","flip","twist"};
    for (int i=0;i<2;++i) {
        const auto prefix="core"+std::to_string(i)+"_";
        mocap_[i]=model_->body_mocapid[lookup(mjOBJ_BODY,prefix+"base")];
        if(mocap_[i]<0) throw std::runtime_error("Core base must be a mocap body");
        for (int j=0;j<4;++j) {
            joints_[i][j]=lookup(mjOBJ_JOINT,prefix+suffix[j]);
            actuators_[i][j]=lookup(mjOBJ_ACTUATOR,prefix+suffix[j]+"_motor");
        }
    }
    reset();
    validate_geometry();
    reset();
}

void Simulation::validate_geometry() {
    // Table ranges alone are insufficient: check the modeled station geometry
    // at the chosen pose, excluding the manipulated energy unit. AABB support
    // bounds are conservative for cylinders/meshes; this is a proxy workspace.
    data_->time=1;update_station();mj_forward(model_.get(),data_.get());
    for(int g=0;g<model_->ngeom;++g) {
        const char* raw=mj_id2name(model_.get(),mjOBJ_GEOM,g);if(!raw) continue;
        const std::string name=raw;
        if(name.rfind("core0_",0)!=0 && (config_.level!=4 || name.rfind("core1_",0)!=0)) continue;
        if(name.find("_unit_")!=std::string::npos) continue;
        const auto* a=model_->geom_aabb+6*g;
        const auto* r=data_->geom_xmat+9*g;
        double xmax=data_->geom_xpos[3*g];
        for(int k=0;k<3;++k) xmax+=r[k]*a[k]+std::abs(r[k])*a[3+k];
        if(xmax>1e-5) throw std::invalid_argument("Pose crosses base-front X=0 plane in proxy geometry: "+name);
    }
}

void Simulation::reset() {
    mj_resetData(model_.get(),data_.get());
    rules_=std::make_unique<AssemblyRules>(config_.level,
        std::array<double,2>{radians(config_.target_degrees[0]),radians(config_.target_degrees[1])});
    commands_={}; lock_active_={}; return_torque_={}; holding_torque_={};
    card_present_=robot_alive_=gripper_enabled_=true;
    update_station();
    mj_forward(model_.get(),data_.get());
    rules_->update(time(),{observation(0),observation(1)});
    update_lights();
}

void Simulation::update_station() {
    const double s=smooth(time());
    for (int i=0;i<2;++i) {
        auto pose=config_.pose;
        if(i==1 && config_.level==4) {
            pose.theta=config_.secondary_angles[0];pose.phi=config_.secondary_angles[1];pose.alpha=config_.secondary_angles[2];
        }
        auto target=pose_quaternion(pose);
        // Shortest-arc quaternion interpolation from identity, including poles.
        if(target[0]<0) for(auto& v:target) v=-v;
        const double angle=std::acos(std::clamp(target[0],-1.0,1.0));
        std::array<double,4> q{1,0,0,0};
        if(angle>1e-9) {
            const double factor=std::sin(s*angle)/std::sin(angle);
            q={std::cos(s*angle),target[1]*factor,target[2]*factor,target[3]*factor};
        }
        auto* p=data_->mocap_pos+3*mocap_[i];
        p[0]=-.2+s*(config_.pose.x+.2);
        p[1]=.2+s*(config_.pose.y-.2)-i*config_.pair_spacing;
        p[2]=.8+s*(config_.pose.z-.8);
        // The unused second station is parked away in single-core modes.
        if(i==1 && config_.level<4) p[1]=-10.0;
        std::copy(q.begin(),q.end(),data_->mocap_quat+4*mocap_[i]);
    }
}

Observation Simulation::observation(std::size_t i) const {
    Observation o;
    const auto& j=joints_.at(i);
    o.insertion=data_->qpos[model_->jnt_qposadr[j[0]]];
    o.lift=data_->qpos[model_->jnt_qposadr[j[1]]];
    o.flip=data_->qpos[model_->jnt_qposadr[j[2]]];
    o.twist=data_->qpos[model_->jnt_qposadr[j[3]]];
    o.station_ready=time()>=1.0;
    o.card_present=card_present_; o.robot_alive=robot_alive_;
    return o;
}

Command Simulation::command(std::size_t i) const { return commands_.at(i); }
void Simulation::set_command(std::size_t i,Command c) {
    if(!all_finite({c.insertion,c.lift,c.flip,c.twist})) throw std::invalid_argument("Nonfinite command");
    c.insertion=std::clamp(c.insertion,0.0,.1); c.lift=std::clamp(c.lift,0.0,.1);
    c.flip=std::clamp(c.flip,0.0,pi/2); c.twist=std::clamp(c.twist,-pi/2,pi/2);
    commands_.at(i)=c;
}

void Simulation::drive_demo() {
    // Smooth reference trajectories; all actual positions come from mj_step.
    for(std::size_t i=0;i<rules_->core_count();++i) {
        const double t=time()-1.2-.18*i;
        Command c;
        c.insertion=.1*smooth(t/1.2);
        if(config_.level>=2) c.lift=.1*smooth((t-1.7)/1.2);
        if(config_.level>=3) {
            c.flip=(pi/2)*smooth((t-3.5)/1.5);
            c.twist=radians(config_.target_degrees[i])*smooth((t-5.6)/1.6);
        }
        set_command(i,c);
    }
    double confirmation=config_.level==1?3.2:config_.level==2?5.0:11.5;
    if(time()>=confirmation && !rules_->successful() && !rules_->failed())
        for(std::size_t i=0;i<rules_->core_count();++i)
            if(rules_->stage(i)!=Stage::Confirmed) confirm(i);
}

bool Simulation::confirm(std::size_t i) {
    rules_->update(time(),{observation(0),observation(1)});
    const bool accepted=rules_->confirm(i,time()); update_lights(); return accepted;
}

void Simulation::step() {
    update_station();
    mj_forward(model_.get(),data_.get());
    mju_zero(data_->qfrc_applied,model_->nv);
    for (std::size_t i=0;i<2;++i) {
        const auto c=commands_[i];
        const double targets[]={c.insertion,c.lift,c.flip,c.twist};
        const double kp[]={6000,6000,38,18}, kd[]={75,100,3.0,1.2};
        const auto o=observation(i);
        // Only Q has the specified return-to-zero torque. This applies at all
        // Q positions; the cap is the rule's maximum, not a measured spring law.
        return_torque_[i]=-std::clamp(.6*o.twist,-.3,.3);
        const bool locked=i<rules_->core_count() && rules_->safety_locked(i);
        if(locked && !lock_active_[i]) lock_angle_[i]=o.twist;
        lock_active_[i]=locked;
        const int qdof=model_->jnt_dofadr[joints_[i][3]];
        holding_torque_[i]=locked?std::clamp(12*(lock_angle_[i]-o.twist)-.3*data_->qvel[qdof],-.5,.5):0;
        data_->qfrc_applied[qdof]=return_torque_[i]+holding_torque_[i];
        for(int j=0;j<4;++j) {
            const int joint=joints_[i][j], dof=model_->jnt_dofadr[joint], act=actuators_[i][j];
            const double q=data_->qpos[model_->jnt_qposadr[joint]], v=data_->qvel[dof];
            // The manual permits release after the safety hold. The unknown
            // station latch/counterbalance is represented by ideal gravity
            // support when released after locking. The extra Q servo remains
            // separately capped at 0.5Nm; this is not an identified mechanism.
            if(locked && !gripper_enabled_) data_->qfrc_applied[dof]+=data_->qfrc_bias[dof];
            // Compensating qfrc_bias supports the gripped unit against gravity;
            // frictionloss and bounded passive Q torques remain in the physics.
            double force=kp[j]*(targets[j]-q)-kd[j]*v+data_->qfrc_bias[dof];
            data_->ctrl[act]=gripper_enabled_?std::clamp(force,
                model_->actuator_ctrlrange[2*act],model_->actuator_ctrlrange[2*act+1]):0;
        }
    }
    mj_step(model_.get(),data_.get());
    // mj_step leaves derived site poses at the pre-integration configuration.
    update_station(); mj_forward(model_.get(),data_.get());
    rules_->update(time(),{observation(0),observation(1)});
    update_lights();
    if(!finite()) throw std::runtime_error("Non-finite state from MuJoCo");
}

bool Simulation::finite() const {
    for(int i=0;i<model_->nq;++i) if(!std::isfinite(data_->qpos[i])) return false;
    for(int i=0;i<model_->nv;++i) if(!std::isfinite(data_->qvel[i])) return false;
    return std::isfinite(time());
}

void Simulation::update_lights() {
    for(std::size_t i=0;i<rules_->core_count();++i) {
        const auto prefix="core"+std::to_string(i)+"_";
        const auto stage=rules_->stage(i);
        const bool fast=std::fmod(time(),1.0/3)<1.0/6, slow=std::fmod(time(),1.0)<.5;
        auto paint=[&](const std::string& name,float r,float g,float b,bool on=true) {
            const int id=mj_name2id(model_.get(),mjOBJ_GEOM,(prefix+name).c_str());
            if(id<0) throw std::runtime_error("Missing source-model light: "+prefix+name);
            float* rgba=model_->geom_rgba+4*id;
            rgba[0]=on?r:.025f;rgba[1]=on?g:.025f;rgba[2]=on?b:.025f;rgba[3]=1;
            const int mat=model_->geom_matid[id];
            if(mat>=0) {
                std::copy(rgba,rgba+4,model_->mat_rgba+4*mat);
                model_->mat_emission[mat]=on?.7f:0;
            }
        };
        const float red=i==0?.95f:.035f, green=i==0?.035f:.18f, blue=i==0?.045f:1.f;
        bool side_on=true, marker_on=true;
        if(stage==Stage::Ready) side_on=marker_on=slow;
        if(stage==Stage::Inserted) {side_on=slow;marker_on=fast;}
        if(stage==Stage::Lifted || stage==Stage::Confirmed) side_on=marker_on=fast;
        // Table5-11 group2 is the two side windows. Group3 is the actual MARK
        // strokes AND the three peg-side strips, never a colored backing square.
        for(int k=1;k<=2;++k) paint("back_main_light_"+std::to_string(k),red,green,blue,side_on);
        for(int k=1;k<=6;++k) paint("pillar_L_light_"+std::to_string(k),red,green,blue,marker_on);
        for(int k=1;k<=3;++k) paint("pillar_side_light_"+std::to_string(k),red,green,blue,marker_on);
        const double current=degrees(observation(i).twist),target=config_.target_degrees[i];
        // Preserve all36 original fans and the center gap. Zero is indicated
        // by the two adjacent5deg bars, not an invented37th center strip.
        auto nearest=[](double value,int k) {
            const double angle=k<=18?95.0-5*k:90.0-5*k;
            const double magnitude=std::max(5.0,std::round(std::abs(value)/5)*5);
            return std::abs(value)<2.5 ? std::abs(angle)==5 :
                std::abs(angle-std::copysign(magnitude,value))<1e-6;
        };
        for(int k=1;k<=36;++k) {
            bool on=slow;float r=.96f,g=.97f,b=1;
            if(stage==Stage::Moving) on=k==static_cast<int>(time()*20)%36+1;
            if(stage==Stage::Flipped || stage==Stage::Aligned) {
                on=nearest(current,k)||nearest(target,k);
                if(nearest(target,k)) {r=red;g=green;b=blue;}
                if(nearest(target,k)&&stage==Stage::Aligned) {r=.04f;g=1;b=.12f;}
            }
            if(stage==Stage::Confirmed) on=fast;
            if(stage==Stage::Failed) {r=1;g=.35f;b=0;on=fast;}
            paint("back_light_bar_"+std::to_string(k),r,g,b,on);
        }
    }
}
} // namespace tech_core
