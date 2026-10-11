#include "tech_core/planning.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace tech_core::planning {
namespace {
using RowMatrix = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
bool starts(const std::string& s, const std::string& p) { return s.rfind(p, 0) == 0; }
int named(const mjModel* m, mjtObj kind, const char* name) {
    const int id = mj_name2id(m, kind, name);
    if (id < 0) throw std::runtime_error(std::string("Missing planning object: ") + name);
    return id;
}

// Certify the WHOLE fixed jaw proxy, not a closest-point witness. MuJoCo's
// zero-distance mesh queries may leave fromto unset; interpreting that world
// zero as a contact point made an invariant grasp depend on the arm posture.
bool fixed_grasp_region(const mjModel* m, const mjData* d, int jaw, int unit) {
    const int jb=m->geom_bodyid[jaw], ub=m->geom_bodyid[unit];
    if(m->body_weldid[jb]!=m->body_weldid[ub] || m->geom_type[jaw]!=mjGEOM_MESH) return false;
    const int mesh=m->geom_dataid[jaw], begin=m->mesh_vertadr[mesh], count=m->mesh_vertnum[mesh];
    const Eigen::Matrix3d r=Eigen::Map<const RowMatrix>(d->xmat+9*ub).transpose();
    const Eigen::Matrix3d gr=Eigen::Map<const RowMatrix>(d->geom_xmat+9*jaw);
    const Eigen::Vector3d gp=Eigen::Map<const Eigen::Vector3d>(d->geom_xpos+3*jaw);
    const Eigen::Vector3d up=Eigen::Map<const Eigen::Vector3d>(d->xpos+3*ub);
    std::vector<Eigen::Vector2d> points;
    for(int i=0;i<count;++i) {
        const auto* v=m->mesh_vert+3*(begin+i);
        const Eigen::Vector3d p=r*(gp+gr*Eigen::Vector3d(v[0],v[1],v[2])-up);
        if(p.x()<.0348 || p.x()>.1102) return false;
        points.emplace_back(p.y(),p.z());
    }
    std::sort(points.begin(),points.end(),[](const auto& a,const auto& b){
        return a.x()<b.x() || (a.x()==b.x() && a.y()<b.y());
    });
    points.erase(std::unique(points.begin(),points.end(),[](const auto&a,const auto&b){
        return (a-b).squaredNorm()<1e-24;
    }),points.end());
    if(points.size()<3) return false;
    auto cross=[](const Eigen::Vector2d&a,const Eigen::Vector2d&b){return a.x()*b.y()-a.y()*b.x();};
    std::vector<Eigen::Vector2d> hull(2*points.size());
    int k=0;
    for(const auto&p:points) {
        while(k>=2 && cross(hull[k-1]-hull[k-2],p-hull[k-1])<=0) --k;
        hull[k++]=p;
    }
    const int lower=k;
    for(int i=static_cast<int>(points.size())-2;i>=0;--i) {
        while(k>lower && cross(hull[k-1]-hull[k-2],points[i]-hull[k-1])<=0) --k;
        hull[k++]=points[i];
    }
    hull.resize(k-1);
    bool inside=true;
    double radius=std::numeric_limits<double>::infinity();
    for(std::size_t i=0;i<hull.size();++i) {
        const auto& a=hull[i];const auto& b=hull[(i+1)%hull.size()];
        const Eigen::Vector2d edge=b-a;
        inside=inside && cross(edge,-a)>=-1e-14;
        const double t=std::clamp(-a.dot(edge)/edge.squaredNorm(),0.0,1.0);
        radius=std::min(radius,(a+t*edge).norm());
    }
    return !inside && radius>=.0320;
}

std::array<double,2> axial_support(const mjModel* m,const mjData* d,int geom,
                                   const Eigen::Vector3d& axis) {
    const Eigen::Vector3d p=Eigen::Map<const Eigen::Vector3d>(d->geom_xpos+3*geom);
    const Eigen::Matrix3d r=Eigen::Map<const RowMatrix>(d->geom_xmat+9*geom);
    const Eigen::Vector3d n=r.transpose()*axis;
    const double centre=axis.dot(p);
    if(m->geom_type[geom]==mjGEOM_CYLINDER) {
        const double extent=m->geom_size[3*geom]*std::hypot(n.x(),n.y())+
            m->geom_size[3*geom+1]*std::abs(n.z());
        return {centre-extent,centre+extent};
    }
    if(m->geom_type[geom]!=mjGEOM_MESH)
        throw std::runtime_error("Unsupported assembly support geometry");
    const int mesh=m->geom_dataid[geom],begin=m->mesh_vertadr[mesh],count=m->mesh_vertnum[mesh];
    double lo=std::numeric_limits<double>::infinity(),hi=-lo;
    for(int i=0;i<count;++i) {
        const auto* v=m->mesh_vert+3*(begin+i);
        const double value=centre+n.dot(Eigen::Vector3d(v[0],v[1],v[2]));
        lo=std::min(lo,value);hi=std::max(hi,value);
    }
    return {lo,hi};
}
}

PlanningScene::PlanningScene(std::string path) {
    char error[2048]{};
    model_.reset(mj_loadXML(path.c_str(), nullptr, error, sizeof(error)));
    if (!model_) throw std::runtime_error(error);
    data_.reset(mj_makeData(model_.get()));
    if (!data_) throw std::runtime_error("Cannot allocate planning data");
    const char* joints[] = {"j0", "j1", "j2", "j3", "j4",
                           "end_differential_controller/j5", "end_differential_controller/j6"};
    for (int i = 0; i < 7; ++i) {
        arm_joints_[i] = named(model(), mjOBJ_JOINT, joints[i]);
        qadr_[i] = model_->jnt_qposadr[arm_joints_[i]];
        dof_[i] = model_->jnt_dofadr[arm_joints_[i]];
        limits_.lower[i] = model_->jnt_range[2*arm_joints_[i]];
        limits_.upper[i] = model_->jnt_range[2*arm_joints_[i]+1];
    }
    arm_root_body_ = model_->jnt_bodyid[arm_joints_[0]];
    mount_geom_ = named(model(), mjOBJ_GEOM, "mount_visual");
    arm_root_original_z_ = model_->body_pos[3*arm_root_body_+2];
    mount_original_z_ = model_->geom_pos[3*mount_geom_+2];
    const char* mechanism[] = {"core0_twist", "core0_flip", "core0_lift"};
    for (int i = 0; i < 3; ++i)
        core_qadr_[i] = model_->jnt_qposadr[named(model(), mjOBJ_JOINT, mechanism[i])];
    // Control the carried unit opening, not the source ee origin. This site
    // includes the declared fixed jaw-to-unit transform in both FK and J.
    tool_ = named(model(), mjOBJ_SITE, "planning_tcp");
    core_mocap_ = model_->body_mocapid[named(model(), mjOBJ_BODY, "core0_base")];
    jacp_.resize(3*model_->nv); jacr_.resize(3*model_->nv);
    centres_.resize(model_->ngeom); extents_.resize(model_->ngeom);
    axial_supports_.resize(model_->ngeom);axial_support_valid_.resize(model_->ngeom);
    std::vector<int> arm, unit, core, fixed;
    std::vector<int> group(model_->ngeom, -1);
    for (int g = 0; g < model_->ngeom; ++g) {
        const char* raw = mj_id2name(model(), mjOBJ_GEOM, g);
        if (!raw) continue;
        const std::string n = raw;
        if (starts(n,"collision_arm_") ||
            ((n.size() > 7 && n.substr(n.size()-7) == "_visual") &&
            (starts(n,"j") || starts(n,"end_") || starts(n,"camera_")) &&
            n!="j4_link_visual" && n!="j6_link_visual" &&
            n!="end_left_link_visual" && n!="end_right_link_visual")) {
            arm.push_back(g);
            int b = model_->geom_bodyid[g];
            while (b && model_->body_jntnum[b] == 0) b = model_->body_parentid[b];
            for (int k = 0; k < 7; ++k)
                if (b && model_->body_jntadr[b] == arm_joints_[k]) group[g] = k;
            if (group[g] < 0) throw std::runtime_error("Unclassified arm collision body " + n);
        } else if (starts(n,"carried_unit_flange_") || starts(n,"carried_unit_tube_") ||
                   n == "carried_unit_iron" || n == "carried_unit_cap") unit.push_back(g);
        else if (n == "core0_peg" || starts(n,"core0_pillar_L_light_") ||
                 starts(n,"core0_pillar_base_") || starts(n,"core0_pillar_inner_ring_") ||
                 n == "core0_pillar_slide") core.push_back(g);
        else if (n == "ground_visual" || n == "mount_visual" || n == "collision_core_shell" || n=="collision_block_fixture")
            fixed.push_back(g);
    }
    if (arm.size()<19 || unit.size()!=98 || core.size()!=16 || fixed.size()!=4)
        throw std::runtime_error("Planning collision manifest does not match scene");
    auto add = [&](int a, int b, bool contact=false, bool grasp=false) {
        const bool rigid=model_->body_weldid[model_->geom_bodyid[a]]==model_->body_weldid[model_->geom_bodyid[b]];
        pairs_.push_back({a,b,contact,grasp,rigid});
    };
    for (std::size_t i=0;i<arm.size();++i) {
        for(std::size_t j=i+1;j<arm.size();++j)
            if (std::abs(group[arm[i]]-group[arm[j]])>1) add(arm[i],arm[j]);
        for(int g:fixed) {
            // Base j0 is physically mounted in the fixed mount, and j1 adjoins it.
            const std::string n=mj_id2name(model(),mjOBJ_GEOM,g);
            if(n=="mount_visual" && group[arm[i]]<=1) continue;
            add(arm[i],g);
        }
        for(int g:core) add(arm[i],g);
        for(int g:unit) {
            const std::string arm_name=mj_id2name(model(),mjOBJ_GEOM,arm[i]);
            const std::string unit_name=mj_id2name(model(),mjOBJ_GEOM,g);
            const bool jaw=(starts(arm_name,"collision_arm_end_left_link_") || starts(arm_name,"collision_arm_end_right_link_")) &&
                arm_name.size()>=5 && arm_name.substr(arm_name.size()-5)=="_grip";
            add(arm[i],g,false,jaw && (starts(unit_name,"carried_unit_tube_") || unit_name=="carried_unit_iron"));
        }
    }
    for(int u:unit) {
        for(int f:fixed) add(u,f);
        const std::string un=mj_id2name(model(),mjOBJ_GEOM,u);
        for(int c:core) {
            const std::string cn=mj_id2name(model(),mjOBJ_GEOM,c);
            const bool contact=(cn=="core0_peg" && un=="carried_unit_iron") ||
                (starts(un,"carried_unit_flange_") && cn!="core0_peg" && cn!="core0_pillar_slide");
            add(u,c,contact);
        }
    }
    std::set<int> unique;
    for(const auto& p:pairs_) { unique.insert(p.a); unique.insert(p.b); }
    collision_geoms_.assign(unique.begin(),unique.end());
    configure(Scenario{});
    std::set<int> checked_grasp_geoms;
    for(const auto& pair:pairs_) if(pair.grasp_contact && checked_grasp_geoms.insert(pair.a).second)
        if(!fixed_grasp_region(model(),data(),pair.a,pair.b))
            throw std::runtime_error(std::string("Jaw grasp proxy is not confined to fixed outer sidewall: ")+
                mj_id2name(model(),mjOBJ_GEOM,pair.a));
}

void PlanningScene::configure(const Scenario& scenario) {
    validate_pose(scenario.core,3);
    if(!std::isfinite(scenario.target_degrees) || std::abs(scenario.target_degrees)>90 ||
       !scenario.start.allFinite() || !scenario.assembly_translation.allFinite() ||
       !std::isfinite(scenario.assembly_yaw) || !std::isfinite(scenario.arm_base_z))
        throw std::invalid_argument("Invalid planning scenario");
    if(scenario.q_sweep && scenario.target_degrees!=-90)
        throw std::invalid_argument("Q sweep must end at -90 degrees");
    if(scenario.q_best_effort && !scenario.q_sweep)
        throw std::invalid_argument("Best-effort Q requires a bidirectional sweep");
    scenario_=scenario;
    rigid_pair_cache_enabled_=false;
    exact_state_cache_enabled_=false;exact_state_cache_.clear();
    for(auto& pair:pairs_) pair.cached_cutoff=-1;
    model_->body_pos[3*arm_root_body_+2]=arm_root_original_z_+scenario.arm_base_z;
    model_->geom_pos[3*mount_geom_+2]=mount_original_z_+scenario.arm_base_z;
    const int ground=named(model(),mjOBJ_GEOM,"ground_visual");
    model_->geom_pos[3*ground+2]=scenario.assembly_translation.z();
    const auto cq=pose_quaternion(scenario.core);
    const Eigen::Matrix3d assembly=Eigen::AngleAxisd(scenario.assembly_yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    core_rotation_=assembly*Eigen::Quaterniond(cq[0],cq[1],cq[2],cq[3]).toRotationMatrix();
    core_position_=scenario.assembly_translation+assembly*Eigen::Vector3d(scenario.core.x,scenario.core.y,scenario.core.z);
    Eigen::Map<Eigen::Vector3d>(data_->mocap_pos+3*core_mocap_)=core_position_;
    const Eigen::Quaterniond q(core_rotation_);
    auto* dest=data_->mocap_quat+4*core_mocap_;
    dest[0]=q.w();dest[1]=q.x();dest[2]=q.y();dest[3]=q.z();
    const int obstacle=named(model(),mjOBJ_GEOM,"collision_block_fixture");
    Eigen::Map<Eigen::Vector3d>(model_->geom_pos+3*obstacle)=scenario.blocking_obstacle?
        mechanism_target({}).position:Eigen::Vector3d(0,0,-10);
    set_state(scenario.start,{});
    counters={};
}

void PlanningScene::forward(const JointVector& q, Eigen::Vector3d& p, Eigen::Matrix3d& r, Jacobian* j) {
    if(!q.allFinite()) throw std::invalid_argument("Nonfinite planning configuration");
    for(int i=0;i<7;++i) data_->qpos[qadr_[i]]=q[i];
    mj_kinematics(model(),data());
    p=Eigen::Map<const Eigen::Vector3d>(data_->site_xpos+3*tool_);
    r=Eigen::Map<const RowMatrix>(data_->site_xmat+9*tool_);
    if(j) {
        mj_comPos(model(),data());
        mj_jacSite(model(),data(),jacp_.data(),jacr_.data(),tool_);
        for(int row=0;row<3;++row) for(int k=0;k<7;++k) {
            (*j)(row,k)=jacp_[row*model_->nv+dof_[k]];
            (*j)(row+3,k)=jacr_[row*model_->nv+dof_[k]];
        }
    }
    ++counters.fk_evals;
}

void PlanningScene::set_state(const JointVector& q,const MechanismState& s) {
    data_->qpos[core_qadr_[0]]=s.twist;
    data_->qpos[core_qadr_[1]]=s.flip;
    data_->qpos[core_qadr_[2]]=s.lift;
    Eigen::Vector3d p; Eigen::Matrix3d r; forward(q,p,r);
}

TaskTarget PlanningScene::mechanism_target(const MechanismState& s) const {
    const Eigen::Matrix3d r=Eigen::AngleAxisd(s.twist,Eigen::Vector3d::UnitX()).toRotationMatrix()*
        Eigen::AngleAxisd(-s.flip,Eigen::Vector3d::UnitY()).toRotationMatrix();
    TaskTarget t;
    t.position=core_position_+core_rotation_*(Eigen::Vector3d(-.169,0,.208)+
        r*Eigen::Vector3d(.169-s.insertion,0,-.208+s.lift));
    t.direction=core_rotation_*r.col(0);
    return t;
}

MechanismState PlanningScene::inferred_mechanism(const Eigen::Vector3d& p,
                                                const Eigen::Matrix3d& r,Phase phase) const {
    if(phase==Phase::Approach) return {};
    const Eigen::Vector3d axis=core_rotation_.transpose()*r.col(2);
    MechanismState state;
    if(phase>=Phase::Flip) state.flip=std::atan2(std::hypot(axis.y(),axis.z()),axis.x());
    if(phase>=Phase::Twist) state.twist=std::atan2(-axis.y(),axis.z());
    const Eigen::Matrix3d undo=Eigen::AngleAxisd(state.flip,Eigen::Vector3d::UnitY()).toRotationMatrix()*
        Eigen::AngleAxisd(-state.twist,Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Vector3d local=undo*(core_rotation_.transpose()*(p-core_position_)-Eigen::Vector3d(-.169,0,.208));
    state.insertion=.169-local.x();state.lift=.208+local.z();
    return state;
}

bool PlanningScene::core_within_front_plane() const {
    // Evaluate all station visual AABBs at the initial mechanism state. This is
    // the same conservative table-range + front-plane gate as Simulation.
    const Eigen::Matrix3d world_to_assembly=Eigen::AngleAxisd(-scenario_.assembly_yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    for(int g=0;g<model_->ngeom;++g) {
        const char* raw=mj_id2name(model_.get(),mjOBJ_GEOM,g);
        if(!raw || !starts(raw,"core0_")) continue;
        const Eigen::Matrix3d r=world_to_assembly*Eigen::Map<const RowMatrix>(data_->geom_xmat+9*g);
        const auto* a=model_->geom_aabb+6*g;
        const Eigen::Vector3d p=world_to_assembly*(Eigen::Map<const Eigen::Vector3d>(data_->geom_xpos+3*g)-scenario_.assembly_translation);
        const double xmax=p.x()+r.row(0).dot(Eigen::Map<const Eigen::Vector3d>(a))+
            r.row(0).cwiseAbs().dot(Eigen::Map<const Eigen::Vector3d>(a+3));
        if(xmax>1e-5) return false;
    }
    return true;
}

CollisionResult PlanningScene::collision(const JointVector& q,const MechanismState& s,double clearance) {
    ++counters.collision_states;
    if(!std::isfinite(clearance) || clearance<0) return {false,-1,"invalid_clearance"};
    if(!q.allFinite() || (q.array()<limits_.lower.array()-1e-10).any() ||
       (q.array()>limits_.upper.array()+1e-10).any()) return {false,-1,"joint_limits"};
    if(!std::isfinite(s.insertion+s.lift+s.flip+s.twist)) return {false,-1,"nonfinite_mechanism"};
    if(exact_state_cache_enabled_) {
        std::array<double,12> key{};
        std::copy(q.data(),q.data()+7,key.begin());
        key[7]=s.insertion;key[8]=s.lift;key[9]=s.flip;key[10]=s.twist;key[11]=clearance;
        const auto found=exact_state_cache_.find(key);
        if(found!=exact_state_cache_.end()) {
            ++counters.exact_state_cache_hits;
            set_state(q,s); // Preserve collision()'s observable scene state.
            return found->second;
        }
        const auto result=collision_uncached(q,s,clearance);
        exact_state_cache_.emplace(key,result);return result;
    }
    return collision_uncached(q,s,clearance);
}

CollisionResult PlanningScene::collision_uncached(const JointVector& q,const MechanismState& s,double clearance) {
    set_state(q,s);
    const int tip=named(model(),mjOBJ_SITE,"core0_tip");
    const Eigen::Vector3d assembly_axis=Eigen::Map<const RowMatrix>(data_->site_xmat+9*tip).col(0);
    if(rigid_pair_cache_enabled_)std::fill(axial_support_valid_.begin(),axial_support_valid_.end(),false);
    auto support=[&](int g) {
        if(!rigid_pair_cache_enabled_)return axial_support(model(),data(),g,assembly_axis);
        // One state, one axis: multiple mating pairs can reuse the same exact
        // support value. Invalidated before every state, including refinement.
        if(!axial_support_valid_[g]) {
            axial_supports_[g]=axial_support(model(),data(),g,assembly_axis);
            axial_support_valid_[g]=true;
        }
        return axial_supports_[g];
    };
    for(int g:collision_geoms_) {
        const auto* a=model_->geom_aabb+6*g;
        const Eigen::Map<const RowMatrix> r(data_->geom_xmat+9*g);
        centres_[g]=Eigen::Map<const Eigen::Vector3d>(data_->geom_xpos+3*g)+r*Eigen::Map<const Eigen::Vector3d>(a);
        extents_[g]=r.cwiseAbs()*Eigen::Map<const Eigen::Vector3d>(a+3);
    }
    CollisionResult result;
    for(auto& pair:pairs_) {
        const bool seated=pair.assembly_contact && s.insertion>=.0998;
        // Named mating surfaces may approach continuously to zero separation;
        // only the final seated state permits 0.1 mm numerical penetration.
        const bool mating=pair.assembly_contact && s.insertion>=0 && s.insertion<=.100001;
        const double required=(seated || pair.grasp_contact)?-.0001:(mating?0:clearance);
        const double cutoff=std::max(clearance,.001)+1e-6;
        if(rigid_pair_cache_enabled_ && pair.rigid && pair.cached_cutoff==cutoff) {
            ++counters.rigid_pair_cache_hits;
            result.distance=std::min(result.distance,pair.cached_distance);
            if(pair.cached_distance<required-1e-9) {
                result.valid=false;
                result.pair=std::string(mj_id2name(model(),mjOBJ_GEOM,pair.a))+"/"+mj_id2name(model(),mjOBJ_GEOM,pair.b);
                return result;
            }
            continue;
        }
        const bool plane=model_->geom_type[pair.a]==mjGEOM_PLANE || model_->geom_type[pair.b]==mjGEOM_PLANE;
        if(rigid_pair_cache_enabled_ && plane) {
            const int pg=model_->geom_type[pair.a]==mjGEOM_PLANE?pair.a:pair.b;
            const int other=pg==pair.a?pair.b:pair.a;
            const Eigen::Vector3d normal=Eigen::Map<const RowMatrix>(data_->geom_xmat+9*pg).col(2);
            const double lower=normal.dot(centres_[other]-Eigen::Map<const Eigen::Vector3d>(data_->geom_xpos+3*pg))-
                normal.cwiseAbs().dot(extents_[other]);
            // The world AABB lies wholly above the plane: a conservative
            // separating-plane proof, including rotated plane normals.
            if(lower>cutoff) {++counters.broadphase_skips;continue;}
        }
        const Eigen::Vector3d separation=(centres_[pair.a]-centres_[pair.b]).cwiseAbs()-extents_[pair.a]-extents_[pair.b];
        if(!plane && separation.maxCoeff()>cutoff) {
            ++counters.broadphase_skips;
            if(rigid_pair_cache_enabled_ && pair.rigid) {pair.cached_cutoff=cutoff;pair.cached_distance=cutoff;}
            continue;
        }
        if(pair.assembly_contact && (mating || seated)) {
            // Each listed pair has the unit on the outward side of the
            // station. Exact convex support extrema certify a separating
            // plane, or bound required separation by the allowed 0.1 mm.
            // Native CCD 3.3.7 can return -47 mm for these nearly coplanar
            // faces; even an exactly aligned fixture reproduces that error.
            // Never use this certificate for bore walls or unnamed pairs.
            const auto unit=support(pair.a);
            const auto station=support(pair.b);
            const double gap=unit[0]-station[1];
            result.distance=std::min(result.distance,gap);
            if(gap<required-1e-9) {
                result.valid=false;
                result.pair=std::string(mj_id2name(model(),mjOBJ_GEOM,pair.a))+"/"+
                    mj_id2name(model(),mjOBJ_GEOM,pair.b)+"/assembly_axial_overlap_d="+std::to_string(gap);
                return result;
            }
            continue;
        }
        ++counters.narrowphase_calls;
        const double distance=mj_geomDistance(model(),data(),pair.a,pair.b,cutoff,nullptr);
        if(!std::isfinite(distance)) return {false,-1,"nonfinite_distance"};
        if(rigid_pair_cache_enabled_ && pair.rigid) {pair.cached_cutoff=cutoff;pair.cached_distance=distance;}
        if(distance<result.distance) result.distance=distance;
        // The complete grasp mesh is certified in unit coordinates at load;
        // both belong to one welded rigid body. Still check separation and
        // penetration on every state, using the same validator for all methods.
        if(distance<required-1e-9) {
            result.valid=false;
            result.pair=std::string(mj_id2name(model(),mjOBJ_GEOM,pair.a))+"/"+mj_id2name(model(),mjOBJ_GEOM,pair.b);
            return result;
        }
    }
    return result;
}

} // namespace tech_core::planning
