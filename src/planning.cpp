#include "tech_core/planning.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tech_core::planning {
namespace {
using Clock=std::chrono::steady_clock;
double seconds(Clock::time_point start) { return std::chrono::duration<double>(Clock::now()-start).count(); }
bool valid_motion_scale(double value) {
    return std::isfinite(value) && value>0 && std::isfinite(2*value*value);
}
void accumulate(Counters& total,const Counters& added) {
    total.ik_iterations+=added.ik_iterations;total.fk_evals+=added.fk_evals;
    total.collision_states+=added.collision_states;total.narrowphase_calls+=added.narrowphase_calls;
    total.broadphase_skips+=added.broadphase_skips;total.rigid_pair_cache_hits+=added.rigid_pair_cache_hits;
    total.exact_state_cache_hits+=added.exact_state_cache_hits;
}
IKResult solve_continuous_ik(const ForwardKinematics& fk,const TaskTarget& target,
                            const JointVector& seed,const JointLimits& limits,
                            const IKOptions& options,bool recover) {
    auto result=solve_ik(fk,target,seed,limits,options);
    if(!result.success && recover && !(options.should_stop && options.should_stop())) {
        auto bounded=options;bounded.method=IKMethod::BoundedQP;bounded.max_iterations=120;
        auto fallback=solve_ik(fk,target,result.q,limits,bounded);
        fallback.iterations+=result.iterations;fallback.fk_evals+=result.fk_evals;
        result=std::move(fallback);
    }
    if(result.success && recover)for(int joint=0;joint<7;++joint) {
        const double q=result.q[joint];
        for(int turn=-2;turn<=2;++turn) {
            const double alternative=q+turn*2*pi;
            if(alternative>=limits.lower[joint] && alternative<=limits.upper[joint] &&
               std::abs(alternative-seed[joint])<std::abs(result.q[joint]-seed[joint]))result.q[joint]=alternative;
        }
    }
    return result;
}
double interval_seconds(const Knot& a,const Knot& b,double scale) {
    const double dq=(b.q-a.q).cwiseAbs().maxCoeff();
    return b.phase==Phase::Dwell?.5:std::max({.1,1.5*dq,std::sqrt(3*dq)})/scale;
}
MechanismState mix(const MechanismState& a,const MechanismState& b,double t) {
    return {a.insertion+t*(b.insertion-a.insertion),a.lift+t*(b.lift-a.lift),
            a.flip+t*(b.flip-a.flip),a.twist+t*(b.twist-a.twist)};
}
TaskTarget mix_target(const TaskTarget& a,const TaskTarget& b,double t) {
    TaskTarget out;
    out.position=(1-t)*a.position+t*b.position;
    const Eigen::Quaterniond delta=Eigen::Quaterniond::FromTwoVectors(a.direction,b.direction);
    out.direction=Eigen::Quaterniond::Identity().slerp(t,delta)*a.direction;
    out.local_axis=a.local_axis;
    return out;
}
TaskPoint between(PlanningScene& scene,const TaskPoint& a,const TaskPoint& b,double t) {
    TaskPoint out; out.mechanism=mix(a.mechanism,b.mechanism,t); out.phase=b.phase;
    out.target=b.phase==Phase::Approach?mix_target(a.target,b.target,t):scene.mechanism_target(out.mechanism);
    return out;
}
double angle(const Eigen::Vector3d& a,const Eigen::Vector3d& b) {
    return std::acos(std::clamp(a.normalized().dot(b.normalized()),-1.0,1.0));
}
double phase_progress(const TaskPoint& p,double target) {
    switch(p.phase) {
        case Phase::Approach:return 0;
        case Phase::Insert:return p.mechanism.insertion/.1;
        case Phase::Lift:return p.mechanism.lift/.1;
        case Phase::Flip:return p.mechanism.flip/(pi/2);
        case Phase::Twist:return target==0?1:p.mechanism.twist/radians(target);
        case Phase::Dwell:return 1;
    }
    return 0;
}
JointVector atlas_at(const AtlasEntry& e,const TaskPoint& p) {
    const double progress=phase_progress(p,e.scenario.target_degrees);
    const Knot* previous=nullptr;
    for(const auto& k:e.knots) {
        if(k.phase!=p.phase) continue;
        const double kp=phase_progress(k,e.scenario.target_degrees);
        if(kp>=progress) {
            if(!previous) return k.q;
            const double a=phase_progress(*previous,e.scenario.target_degrees);
            const double t=kp>a?(progress-a)/(kp-a):1;
            return ((1-t)*previous->q+t*k.q).eval();
        }
        previous=&k;
    }
    return previous?previous->q:e.knots.front().q;
}
// Shared dense interval check. Q extension uses precisely the same samples,
// interpolation, collision envelope and tracking/rate limits as final acceptance.
bool validate_segment(PlanningScene& scene,const Knot& a,const Knot& b,std::size_t i,
                      const PlanOptions& options,ValidationResult& out,
                      const std::function<bool()>& stop,AssemblyRules* rules=nullptr) {
    const double dq=(b.q-a.q).cwiseAbs().maxCoeff();
    const double dt=i?b.time-a.time:1;
    out.max_joint_speed=std::max(out.max_joint_speed,1.5*dq/dt);
    out.max_joint_acceleration=std::max(out.max_joint_acceleration,6*dq/(dt*dt));
    if(out.max_joint_speed>options.motion_speed_scale+1e-8 ||
       out.max_joint_acceleration>2*options.motion_speed_scale*options.motion_speed_scale+1e-8) {
        out.reason="rate_limit";return false;
    }
    const int n=i?std::max(options.validation_min_substeps,
                  static_cast<int>(std::ceil(1.5*dq/options.validation_joint_step))):1;
    for(int step=i?1:0;step<=(i?n:0);++step) {
        if(stop && stop()) {out.reason="deadline_during_validation";return false;}
        const double u=double(step)/n,blend=u*u*(3-2*u);
        const JointVector q=(1-blend)*a.q+blend*b.q;
        const TaskPoint expected=i?between(scene,a,b,blend):static_cast<const TaskPoint&>(b);
        Eigen::Vector3d p;Eigen::Matrix3d r;
        scene.forward(q,p,r);
        const double pe=(p-expected.target.position).norm();
        const double ae=angle(r.col(2),expected.target.direction);
        out.max_position_error=std::max(out.max_position_error,pe);
        out.max_axis_error=std::max(out.max_axis_error,ae);
        ++out.samples;
        if(pe>options.validation_position_tolerance || ae>options.validation_axis_tolerance) {
            out.reason="tracking_"+std::string(phase_name(b.phase))+"_"+std::to_string(i)+
                "_p="+std::to_string(pe)+"_a="+std::to_string(ae);return false;
        }
        const auto collision=scene.collision(q,expected.mechanism);
        if(!collision.valid) {
            out.reason="collision_"+std::string(phase_name(b.phase))+"_"+std::to_string(i)+
                "_"+collision.pair+"_d="+std::to_string(collision.distance);return false;
        }
        if(rules) {
            Observation obs;
            const auto measured=scene.inferred_mechanism(p,r,b.phase);
            obs.insertion=measured.insertion;obs.lift=measured.lift;
            obs.flip=measured.flip;obs.twist=measured.twist;
            rules->update(a.time+u*(b.time-a.time),{obs,{}});
            if(rules->failed()) {out.reason="rules_"+rules->failure_reason();return false;}
        }
    }
    return true;
}

}

const char* phase_name(Phase p) {
    switch(p) {
        case Phase::Approach:return "approach";case Phase::Insert:return "insert";
        case Phase::Lift:return "lift";case Phase::Flip:return "flip";
        case Phase::Twist:return "twist";case Phase::Dwell:return "dwell";
    } return "unknown";
}
const char* method_name(Method m) {
    switch(m) {
        case Method::Baseline:return "baseline";case Method::FullStep:return "full_step";
        case Method::Reduced5D:return "reduced_5d";case Method::AdaptiveLM:return "adaptive_lm";
        case Method::Predictor:return "predictor";case Method::SparseAdaptive:return "sparse_adaptive";
        case Method::Atlas:return "atlas";
        case Method::BoundedQP:return "bounded_qp";
        case Method::RigidCache:return "rigid_cache";
    } return "unknown";
}
Method parse_method(const std::string& n) {
    for(Method m:{Method::Baseline,Method::FullStep,Method::Reduced5D,Method::AdaptiveLM,
                  Method::Predictor,Method::SparseAdaptive,Method::Atlas,Method::BoundedQP,Method::RigidCache})
        if(n==method_name(m))return m;
    throw std::invalid_argument("Unknown planning method: "+n);
}

std::vector<TaskPoint> make_task(PlanningScene& scene,const PlanOptions& options) {
    if(!std::isfinite(options.position_spacing) || !std::isfinite(options.angular_spacing) ||
       options.position_spacing<1e-5 || options.angular_spacing<1e-5)
        throw std::invalid_argument("Task spacing must be positive");
    std::vector<TaskPoint> points;
    // The assembly task starts 40 mm outside E, along the opening's outward
    // axis. plan() solves at this pose and searches a collision-checked joint
    // connection from the physical start; no Cartesian path is prescribed.
    TaskPoint approached; approached.target=scene.mechanism_target({});
    approached.target.position+=.04*approached.target.direction;
    points.push_back(approached);
    TaskPoint aligned;aligned.target=scene.mechanism_target({});
    const int alignment_steps=std::max(1,static_cast<int>(std::ceil(.04/options.position_spacing)));
    for(int i=1;i<=alignment_steps;++i) points.push_back(between(scene,approached,aligned,double(i)/alignment_steps));
    auto append=[&](MechanismState end,Phase phase,double motion,double spacing) {
        const TaskPoint start=points.back();
        TaskPoint finish; finish.mechanism=end; finish.phase=phase; finish.target=scene.mechanism_target(end);
        const int n=std::max(1,static_cast<int>(std::ceil(motion/spacing)));
        for(int i=1;i<=n;++i) points.push_back(between(scene,start,finish,double(i)/n));
    };
    append({.1,0,0,0},Phase::Insert,.1,options.position_spacing);
    append({.1,.1,0,0},Phase::Lift,.1,options.position_spacing);
    append({.1,.1,pi/2,0},Phase::Flip,pi/2,options.angular_spacing);
    if(scene.scenario().q_sweep)
        append({.1,.1,pi/2,pi/2},Phase::Twist,pi/2,options.angular_spacing);
    append({.1,.1,pi/2,radians(scene.scenario().target_degrees)},Phase::Twist,
           std::abs(radians(scene.scenario().target_degrees)-points.back().mechanism.twist),options.angular_spacing);
    TaskPoint dwell=points.back();dwell.phase=Phase::Dwell;points.push_back(dwell);
    return points;
}

void parameterize(std::vector<Knot>& knots,double motion_speed_scale) {
    if(!valid_motion_scale(motion_speed_scale)) throw std::invalid_argument("Invalid motion speed scale");
    if(knots.empty())return;
    knots.front().time=0;
    for(std::size_t i=1;i<knots.size();++i) {
        // Cubic position interpolation with zero velocity at each point,
        // matching upstream's zero-velocity message contract. Enforce the
        // explicit default mock v<=1 rad/s and a<=2 rad/s^2. Scaling time
        // changes these by s and s^2; these remain simulation-only limits.
        const double dt=interval_seconds(knots[i-1],knots[i],motion_speed_scale);
        knots[i].time=knots[i-1].time+dt;
    }
}

ValidationResult validate_trajectory(PlanningScene& scene,const std::vector<Knot>& knots,
                                    const PlanOptions& options,const std::function<bool()>& stop) {
    ValidationResult out;
    if(!valid_motion_scale(options.motion_speed_scale) ||
       !std::isfinite(options.validation_joint_step) || options.validation_joint_step<=0 ||
       !std::isfinite(options.validation_position_tolerance) || options.validation_position_tolerance<=0 ||
       !std::isfinite(options.validation_axis_tolerance) || options.validation_axis_tolerance<=0 ||
       options.validation_min_substeps<1) {out.reason="invalid_validation_options";return out;}
    if(knots.size()<6) {out.reason="incomplete_trajectory";return out;}
    std::array<bool,6> phases{};
    for(const auto& k:knots) {
        const auto& s=k.mechanism;
        if(!k.q.allFinite() || !k.target.position.allFinite() || !k.target.direction.allFinite() ||
           !k.target.local_axis.allFinite() || !std::isfinite(k.time) ||
           !std::isfinite(s.insertion) || !std::isfinite(s.lift) || !std::isfinite(s.flip) || !std::isfinite(s.twist)) {
            out.reason="nonfinite_knot";return out;
        }
        const int phase=static_cast<int>(k.phase);
        if(phase<0 || phase>=6) {out.reason="invalid_phase";return out;}
        phases[phase]=true;
        if(std::abs(k.target.direction.norm()-1)>1e-6 ||
           (k.target.local_axis-Eigen::Vector3d::UnitZ()).norm()>1e-10) {
            out.reason="invalid_target_axis";return out;
        }
    }
    if(!std::all_of(phases.begin(),phases.end(),[](bool x){return x;})) {out.reason="missing_phase";return out;}
    if((knots.front().q-scene.scenario().start).norm()>1e-10) {out.reason="wrong_start";return out;}
    if(std::abs(knots.front().time)>1e-12) {out.reason="invalid_timestamps";return out;}
    for(std::size_t i=1;i<knots.size();++i)
        if(knots[i].time<=knots[i-1].time) {out.reason="invalid_timestamps";return out;}
    int previous_phase=0;
    for(const auto& k:knots) {
        const int p=static_cast<int>(k.phase);const auto& s=k.mechanism;
        if(p<previous_phase || p>previous_phase+1) {out.reason="phase_order";return out;}
        previous_phase=p;
        constexpr double eps=1e-9;
        bool valid=s.insertion>=-eps && s.insertion<=.1+eps && s.lift>=-eps && s.lift<=.1+eps &&
            s.flip>=-eps && s.flip<=pi/2+eps && std::abs(s.twist)<=pi/2+eps;
        if(k.phase==Phase::Approach) valid=valid && std::abs(s.insertion)+std::abs(s.lift)+std::abs(s.flip)+std::abs(s.twist)<eps;
        if(k.phase==Phase::Insert) valid=valid && std::abs(s.lift)+std::abs(s.flip)+std::abs(s.twist)<eps;
        if(k.phase>=Phase::Lift) valid=valid && std::abs(s.insertion-.1)<eps;
        if(k.phase==Phase::Lift) valid=valid && std::abs(s.flip)+std::abs(s.twist)<eps;
        if(k.phase>=Phase::Flip) valid=valid && std::abs(s.lift-.1)<eps;
        if(k.phase==Phase::Flip) valid=valid && std::abs(s.twist)<eps;
        if(k.phase>=Phase::Twist) valid=valid && std::abs(s.flip-pi/2)<eps;
        if(k.phase==Phase::Dwell) {
            const double goal=scene.scenario().q_best_effort?knots.back().mechanism.twist:
                              radians(scene.scenario().target_degrees);
            valid=valid && std::abs(s.twist-goal)<eps;
        }
        if(!valid) {out.reason="invalid_mechanism_contract";return out;}
        if(k.phase!=Phase::Approach) {
            const auto expected=scene.mechanism_target(s);
            if((k.target.position-expected.position).norm()>1e-8 ||
               (k.target.direction-expected.direction).norm()>1e-8) {
                out.reason="invalid_exported_target";return out;
            }
        }
    }
    if(scene.scenario().q_sweep) {
        bool positive_endpoint=false,returning=false;
        double previous=0;
        for(const auto& k:knots) if(k.phase==Phase::Twist) {
            const double q=k.mechanism.twist;
            if(scene.scenario().q_best_effort) {
                if(q<previous-1e-9) returning=true;
                if(returning && q>previous+1e-9) {out.reason="q_sweep_order";return out;}
            } else if((!positive_endpoint && q<previous-1e-9) ||
                      (positive_endpoint && q>previous+1e-9)) {
                out.reason="q_sweep_order";return out;
            }
            if(std::abs(q-pi/2)<1e-9) positive_endpoint=true;
            previous=q;
        }
        if(!scene.scenario().q_best_effort && (!positive_endpoint || std::abs(previous+pi/2)>1e-9)) {
            out.reason="q_sweep_missing_endpoint";return out;
        }
        if(std::abs(previous-knots.back().mechanism.twist)>1e-9) {
            out.reason="q_dwell_jump";return out;
        }
    }
    const double goal=scene.scenario().q_best_effort?knots.back().mechanism.twist:
                      radians(scene.scenario().target_degrees);
    AssemblyRules rules(3,{goal,0});
    for(std::size_t i=0;i<knots.size();++i) {
        const auto& b=knots[i];const auto& a=knots[i?i-1:0];
        if(!std::isfinite(b.time) || (i && b.time<=a.time)) {out.reason="invalid_timestamps";return out;}
        phases[static_cast<int>(b.phase)]=true;
        if(!validate_segment(scene,a,b,i,options,out,stop,&rules)) return out;
    }
    if(!std::all_of(phases.begin(),phases.end(),[](bool x){return x;})) {out.reason="missing_phase";return out;}
    out.rule_success=rules.confirm(0,knots.back().time);
    if(stop && stop()) {out.reason="deadline_during_validation";return out;}
    out.valid=out.rule_success;
    out.reason=out.valid?"validated":"rules_not_complete";
    return out;
}

static PlanResult plan_candidate(PlanningScene& scene,const Scenario& scenario,const PlanOptions& options,
                const std::vector<AtlasEntry>* atlas, const std::optional<JointVector>& preinsert_goal = {}) {
    const auto start=Clock::now();
    PlanResult result;result.method=options.method;
    auto stop=[&](){return seconds(start)>=options.timeout_seconds;};
    auto finish=[&]() {
        if(result.success && scenario.q_sweep && !result.knots.empty()) {
            result.q_positive_degrees=0;
            for(const auto& k:result.knots) result.q_positive_degrees=std::max(result.q_positive_degrees,k.mechanism.twist*180/pi);
            result.q_final_degrees=result.knots.back().mechanism.twist*180/pi;
            result.full_q_sweep=std::abs(result.q_positive_degrees-90)<1e-8 && std::abs(result.q_final_degrees+90)<1e-8;
        }
        result.planning_seconds=seconds(start);result.counters=scene.counters;return result;
    };
    if(!std::isfinite(options.timeout_seconds) || options.timeout_seconds<=0) {
        result.reason="invalid_timeout";return finish();
    }
    if(!valid_motion_scale(options.motion_speed_scale)) {
        result.reason="invalid_motion_speed_scale";return finish();
    }
    if(options.approach_joint_waypoint && !options.approach_joint_waypoint->allFinite()) {
        result.reason="invalid_approach_joint_waypoint";return finish();
    }
    scene.configure(scenario);
    const bool cached_validation=options.method==Method::RigidCache ||
        (options.optimize_joint_motion && options.method==Method::Baseline);
    scene.enable_rigid_pair_cache(cached_validation);
    scene.enable_exact_state_cache(scenario.q_best_effort && cached_validation);
    if(scenario.enforce_core_pose_limits && !scene.core_within_front_plane()) {
        result.reason="invalid_station_front_plane";return finish();
    }
    const auto initial=scene.collision(scenario.start,{});
    if(!initial.valid) {result.reason="start_collision_"+initial.pair+"_d="+std::to_string(initial.distance);return finish();}
    PlanOptions sampling=options;
    if(options.method==Method::SparseAdaptive) {sampling.position_spacing=.012;sampling.angular_spacing=radians(5);}
    const auto targets=make_task(scene,sampling);
    const AtlasEntry* entry=nullptr;
    if(options.method==Method::Atlas && atlas) {
        double nearest=std::numeric_limits<double>::infinity();
        for(const auto& e:*atlas) {
            // Atlas phase indexing assumes a monotone Q leg. Sweeps use live IK.
            if(e.knots.empty() || e.scenario.split!="development" ||
               e.scenario.q_sweep || scenario.q_sweep)continue;
            const double distance=std::hypot(e.scenario.core.x-scenario.core.x,e.scenario.core.z-scenario.core.z)+
                std::abs(e.scenario.core.y-scenario.core.y)+.002*std::abs(e.scenario.target_degrees-scenario.target_degrees)+
                .002*(std::abs(e.scenario.core.theta-scenario.core.theta)+std::abs(e.scenario.core.phi-scenario.core.phi)+
                      std::abs(e.scenario.core.alpha-scenario.core.alpha));
            if(distance<nearest){nearest=distance;entry=&e;}
        }
    }
    if(options.method==Method::Atlas && !entry) {
        result.reason="atlas_has_no_valid_development_entries";return finish();
    }
    if(entry) result.atlas_source=entry->scenario.id;
    IKOptions ik;
    ik.should_stop=stop;
    if(options.method==Method::SparseAdaptive) {
        // Coarse endpoint error must leave room for the stricter 0.1 mm mating
        // contact envelope, even though global path tracking permits 0.5 mm.
        ik.position_tolerance=1e-6;ik.axis_tolerance=1e-6;
    }
    switch(options.method) {
        case Method::Baseline:ik.method=IKMethod::Baseline;break;
        case Method::FullStep:ik.method=IKMethod::FullStep;break;
        case Method::AdaptiveLM:ik.method=IKMethod::AdaptiveLM;break;
        case Method::BoundedQP:ik.method=IKMethod::BoundedQP;break;
        default:ik.method=IKMethod::Reduced5D;break;
    }
    const bool recover=options.optimize_joint_motion &&
        (options.method==Method::Baseline || options.method==Method::RigidCache);
    if(recover) {
        ik.alpha=.7;ik.max_joint_step=JointVector::Constant(.2);ik.max_iterations=160;
        // The seated-contact envelope is 0.1 mm: an IK endpoint tolerance of
        // 0.1 mm leaves no room for interpolated tracking or numerical error.
        ik.position_tolerance=1e-6;ik.axis_tolerance=1e-6;
    }
    const ForwardKinematics fk=[&](const JointVector& q,Eigen::Vector3d& p,Eigen::Matrix3d& r,Jacobian* j){scene.forward(q,p,r,j);};
    JointVector previous=scenario.start;
    const auto ikstart=Clock::now();
    auto ik_seconds=[&] {return std::max(0.,seconds(ikstart)-result.approach_seconds);};
    auto connect=[&](const JointVector& waypoint) {
        const auto connection_start=Clock::now();
        // Leave time for the assembly IK and full-path validation. Search and
        // optimization share this request's deadline, including failed trials.
        const double budget=.6*std::max(0.,options.timeout_seconds-seconds(start));
        auto connection_stop=[&] {return stop() || seconds(connection_start)>=budget;};
        const auto connected=optimize_approach(scene,previous,waypoint,options,connection_stop);
        result.approach_iterations+=connected.iterations;
        result.approach_shortcuts+=connected.shortcuts;
        auto done=[&](bool success) {result.approach_seconds+=seconds(connection_start);return success;};
        // Only interpolate edges certified by the optimizer. FK targets keep
        // this free-space motion separate from the subsequent assembly task.
        if(!connected.path.empty() && result.knots.empty()) {
            Knot first;first.q=connected.path.front();
            Eigen::Matrix3d r;scene.forward(first.q,first.target.position,r);
            first.target.direction=r.col(2);result.knots.push_back(first);
        }
        for(std::size_t edge=1;edge<connected.path.size();++edge) {
            const auto& from=connected.path[edge-1];const auto& to=connected.path[edge];
            const int n=std::max(1,static_cast<int>(std::ceil((to-from).cwiseAbs().maxCoeff()/.02)));
            for(int i=1;i<=n;++i) {
                if(stop()) {result.reason="deadline_during_approach_connection";return done(false);}
                Knot k;k.q=i==n?to:(from+(double(i)/n)*(to-from)).eval();
                Eigen::Matrix3d rotation;scene.forward(k.q,k.target.position,rotation);
                k.target.direction=rotation.col(2);result.knots.push_back(k);
            }
        }
        if(!connected.success) {result.reason=connected.reason;return done(false);}
        previous=waypoint;
        return done(true);
    };
    if(options.approach_joint_waypoint) {
        const auto& waypoint=*options.approach_joint_waypoint;
        if((waypoint.array()<scene.limits().lower.array()).any() ||
           (waypoint.array()>scene.limits().upper.array()).any()) {
            result.reason="approach_waypoint_joint_limits";return finish();
        }
        if(!connect(waypoint)) {result.ik_seconds=ik_seconds();return finish();}
    }
    // A redundant IK solution can itself collide. Keep the actual start fixed,
    // and try a bounded set of seeds for the same pre-insertion target. This is
    // shared by all backends; every solve and collision check is timed online.
    const JointVector nominal=Scenario{}.start;
    std::vector<JointVector> seeds{previous,nominal,.5*(scene.limits().lower+scene.limits().upper)};
    std::mt19937 seed_random(20261011);std::normal_distribution<double> seed_noise(0,.6);
    for(int attempt=0;attempt<5;++attempt) {
        JointVector seed=attempt%2?previous:nominal;
        for(int j=0;j<7;++j) seed[j]+=seed_noise(seed_random);
        seeds.push_back(seed.cwiseMax(scene.limits().lower).cwiseMin(scene.limits().upper));
    }
    IKResult approached;
    bool clear_goal=false;
    std::string goal_failure;
    if(preinsert_goal) {
        approached.q=*preinsert_goal;approached.success=true;
        clear_goal=scene.collision(approached.q,{}).valid;
        if(!clear_goal)goal_failure="preinsert_goal_collision";
    }
    for(const auto& seed:seeds) {
        if(preinsert_goal)break;
        if(stop()) break;
        ++result.preinsert_ik_attempts;
        approached=solve_continuous_ik(fk,targets.front().target,seed,scene.limits(),ik,recover);
        scene.counters.ik_iterations+=approached.iterations;
        if(!approached.success) {goal_failure="ik_preinsert_"+approached.reason;continue;}
        const auto collision=scene.collision(approached.q,{});
        if(collision.valid) {clear_goal=true;break;}
        goal_failure="preinsert_goal_collision_"+collision.pair;
    }
    if(!clear_goal) {
        result.reason=stop()?"deadline_during_preinsert_ik":goal_failure;
        result.ik_seconds=ik_seconds();return finish();
    }
    result.preinsert_joints=approached.q;
    if(!connect(approached.q)) {result.ik_seconds=ik_seconds();return finish();}
    result.knots.back().target=targets.front().target;
    const std::size_t connection_knots=result.knots.size();
    for(std::size_t i=1;i<targets.size();++i) {
        if(scenario.q_best_effort && targets[i].phase>=Phase::Twist) break;
        if(stop()) {result.reason="deadline_during_ik";result.ik_seconds=ik_seconds();return finish();}
        JointVector seed=previous;
        if((options.method==Method::Predictor || options.method==Method::RigidCache) && i) {
            Eigen::Vector3d p;Eigen::Matrix3d r;Jacobian j;scene.forward(previous,p,r,&j);
            const auto axis=r.col(2).eval();
            seed+=predict_task_step(j,axis,targets[i].target.position-p,axis.cross(targets[i].target.direction));
            seed=seed.cwiseMax(scene.limits().lower).cwiseMin(scene.limits().upper);
        } else if(entry && i && targets[i].phase!=Phase::Approach && targets[i-1].phase==targets[i].phase) {
            // Transfer a neighbouring DEVELOPMENT trajectory increment, then
            // correct against the new live target and validate every segment.
            TaskPoint a=targets[i-1],b=targets[i];
            if(b.phase==Phase::Twist && scenario.target_degrees!=0) {
                a.mechanism.twist*=entry->scenario.target_degrees/scenario.target_degrees;
                b.mechanism.twist*=entry->scenario.target_degrees/scenario.target_degrees;
            }
            seed+=atlas_at(*entry,b)-atlas_at(*entry,a);
            seed=seed.cwiseMax(scene.limits().lower).cwiseMin(scene.limits().upper);
        }
        const auto solved=solve_continuous_ik(fk,targets[i].target,seed,scene.limits(),ik,recover);
        scene.counters.ik_iterations+=solved.iterations;
        if(!solved.success) {
            result.reason="ik_"+std::string(phase_name(targets[i].phase))+"_"+std::to_string(i)+"_"+solved.reason;
            result.ik_seconds=ik_seconds();return finish();
        }
        Knot k;static_cast<TaskPoint&>(k)=targets[i];k.q=solved.q;result.knots.push_back(k);previous=k.q;
    }
    if(options.method==Method::SparseAdaptive || recover) {
        // Refinement retains existing endpoints, so it cannot repair a
        // collision at an already solved coarse knot. Reject that candidate.
        for(const auto& k:result.knots) {
            if(stop()) {result.reason="deadline_during_refinement";result.ik_seconds=ik_seconds();return finish();}
            const auto check=scene.collision(k.q,k.mechanism);
            if(!check.valid) {
                result.reason="collision_coarse_"+std::string(phase_name(k.phase))+"_"+check.pair;
                result.ik_seconds=ik_seconds();return finish();
            }
        }
        // Refine only intervals whose joint-space interpolation misses the
        // manifold or collision envelope; final acceptance is shared.
        for(int pass=0;pass<8;++pass) {
            bool changed=false;std::vector<Knot> refined;refined.push_back(result.knots.front());
            for(std::size_t i=1;i<result.knots.size();++i) {
                if(stop()) {result.reason="deadline_during_refinement";result.ik_seconds=ik_seconds();return finish();}
                const auto& a=result.knots[i-1];const auto& b=result.knots[i];
                // Preserve the collision-checked connection; refine the assembly
                // task. The final validator still checks every connection span.
                if(i<connection_knots) {refined.push_back(b);continue;}
                bool split=false;
                for(double t:{.25,.5,.75}) {
                    const auto expected=between(scene,a,b,t);Eigen::Vector3d p;Eigen::Matrix3d r;
                    scene.forward((1-t)*a.q+t*b.q,p,r);
                    // Endpoint accuracy alone cannot bound the error of a
                    // joint chord. Leave room inside the 0.1 mm mating envelope.
                    const double position_bound=recover?std::min(.000025,.5*options.validation_position_tolerance):
                        .5*options.validation_position_tolerance;
                    const double axis_bound=recover?std::min(.0001,.5*options.validation_axis_tolerance):
                        .5*options.validation_axis_tolerance;
                    if((p-expected.target.position).norm()>position_bound ||
                       angle(r.col(2),expected.target.direction)>axis_bound) split=true;
                    if(!scene.collision((1-t)*a.q+t*b.q,expected.mechanism).valid) split=true;
                }
                if(split) {
                    const auto target=between(scene,a,b,.5);
                    const auto solved=solve_continuous_ik(fk,target.target,.5*(a.q+b.q),scene.limits(),ik,recover);
                    scene.counters.ik_iterations+=solved.iterations;
                    if(!solved.success){result.reason="adaptive_refinement_"+solved.reason;result.ik_seconds=ik_seconds();return finish();}
                    Knot mid;static_cast<TaskPoint&>(mid)=target;mid.q=solved.q;refined.push_back(mid);changed=true;
                }
                refined.push_back(b);
            }
            result.knots=std::move(refined);if(!changed)break;
        }
    }
    result.ik_seconds=ik_seconds();
    if(scenario.q_best_effort) {
        // Reject colliding pre-Q endpoints before paying for all dense samples.
        // This only rejects candidates; acceptance still uses every original
        // substep. Exact endpoint results can then be reused by that validator.
        const auto coarse_start=Clock::now();
        for(const auto& k:result.knots) {
            if(stop()) {
                result.validation_seconds=seconds(coarse_start);
                result.reason="deadline_during_pre_q_screen";return finish();
            }
            const auto collision=scene.collision(k.q,k.mechanism);
            if(!collision.valid) {
                result.validation_seconds=seconds(coarse_start);
                result.reason="collision_pre_q_"+std::string(phase_name(k.phase))+"_"+collision.pair;
                return finish();
            }
        }
        result.validation_seconds=seconds(coarse_start);
        // First certify the entire mandatory prefix and a stationary Q stage.
        // This is the only fallback if the optional extension cannot be fully
        // validated in time; an IK-only or collision-failing prefix is never used.
        const std::size_t prefix_size=result.knots.size();
        Knot zero=result.knots.back();zero.phase=Phase::Twist;
        result.knots.push_back(zero);zero.phase=Phase::Dwell;result.knots.push_back(zero);
        parameterize(result.knots,options.motion_speed_scale);
        const auto first_check=Clock::now();
        result.validation=validate_trajectory(scene,result.knots,options,stop);
        const double prefix_check_seconds=seconds(first_check);
        result.validation_seconds+=prefix_check_seconds;
        if(!result.validation.valid) {result.reason=result.validation.reason;return finish();}
        const auto fallback_knots=result.knots;
        const auto fallback_validation=result.validation;
        result.knots.resize(prefix_size);
        double positive_checks=0,negative_checks=0,repair_seconds=0;
        const double repair_budget=.15*options.timeout_seconds;
        // Reserve measured validation work for the final complete pass,
        // including the reverse of the positive leg, plus return overhead.
        auto growth_stop=[&]() {
            // Exact-state memoization covers repeated prefix/forward samples.
            // Reverse samples are still budgeted at their measured full cost.
            const double reserve=cached_validation?
                .2*prefix_check_seconds+1.3*positive_checks+.2*negative_checks+.06:
                1.3*(prefix_check_seconds+2*positive_checks+negative_checks)+.06;
            return seconds(start)+reserve>=options.timeout_seconds;
        };
        IKOptions qik=ik;qik.should_stop=growth_stop;
        auto extend=[&](double goal,double& checked_seconds) {
            const double origin=result.knots.back().mechanism.twist;
            const int count=std::max(1,static_cast<int>(std::ceil(std::abs(goal-origin)/sampling.angular_spacing)));
            for(int step=1;step<=count;++step) {
                if(growth_stop()) return std::string("online_budget");
                Knot next=result.knots.back();next.phase=Phase::Twist;
                next.mechanism.twist=origin+(goal-origin)*double(step)/count;
                next.target=scene.mechanism_target(next.mechanism);
                JointVector seed=next.q;
                const auto solve_start=Clock::now();
                if(options.method==Method::Predictor || options.method==Method::RigidCache) {
                    Eigen::Vector3d p;Eigen::Matrix3d r;Jacobian j;scene.forward(seed,p,r,&j);
                    const auto axis=r.col(2).eval();
                    seed+=predict_task_step(j,axis,next.target.position-p,axis.cross(next.target.direction));
                    seed=seed.cwiseMax(scene.limits().lower).cwiseMin(scene.limits().upper);
                }
                const auto solved=solve_continuous_ik(fk,next.target,seed,scene.limits(),qik,recover);
                result.ik_seconds+=seconds(solve_start);scene.counters.ik_iterations+=solved.iterations;
                if(!solved.success) return std::string("ik_")+solved.reason;
                next.q=solved.q;
                next.time=result.knots.back().time+interval_seconds(result.knots.back(),next,options.motion_speed_scale);
                ValidationResult check;const auto check_start=Clock::now();
                bool valid=validate_segment(scene,result.knots.back(),next,result.knots.size(),options,check,growth_stop);
                double accepted_check_seconds=seconds(check_start);
                result.validation_seconds+=accepted_check_seconds;
                if(!valid && recover && check.reason.rfind("collision_",0)==0 &&
                   repair_seconds<repair_budget && !growth_stop()) {
                    // A colliding nearest IK branch is not a task-space travel
                    // limit. Explore the two redundant directions locally,
                    // then project back onto this same next Q target. Retain
                    // the prefix and require the original dense segment check.
                    const auto repair_begin=Clock::now();
                    auto repair_stop=[&] {return growth_stop() ||
                        repair_seconds+seconds(repair_begin)>=repair_budget;};
                    auto repair_ik=qik;repair_ik.should_stop=repair_stop;
                    Eigen::Vector3d p;Eigen::Matrix3d r;Jacobian j;
                    scene.forward(solved.q,p,r,&j);const auto axis=r.col(2).eval();
                    std::vector<JointVector> basis;
                    for(int joint:{6,3,0,4,1,2,5}) {
                        JointVector direction=JointVector::Unit(joint)-predict_task_step(
                            j,axis,j.topRows<3>().col(joint),j.bottomRows<3>().col(joint));
                        for(const auto& b:basis)direction-=b.dot(direction)*b;
                        if(direction.norm()<1e-5)continue;
                        basis.push_back(direction.normalized());
                        if(basis.size()==2)break;
                    }
                    std::vector<JointVector> directions;
                    for(const auto& b:basis) {directions.push_back(b);directions.push_back(-b);}
                    if(basis.size()==2)for(double a:{-1.,1.})for(double b:{-1.,1.})
                        directions.push_back((a*basis[0]+b*basis[1]).normalized());
                    for(double radius:{.025,.05,.1,.2}) {
                        for(const auto& direction:directions) {
                            if(repair_stop())break;
                            const auto alternative_begin=Clock::now();
                            const auto alternative=solve_continuous_ik(fk,next.target,
                                solved.q+radius*direction,scene.limits(),repair_ik,true);
                            result.ik_seconds+=seconds(alternative_begin);
                            scene.counters.ik_iterations+=alternative.iterations;
                            if(!alternative.success)continue;
                            Knot candidate=next;candidate.q=alternative.q;
                            candidate.time=result.knots.back().time+
                                interval_seconds(result.knots.back(),candidate,options.motion_speed_scale);
                            ValidationResult candidate_check;const auto candidate_begin=Clock::now();
                            valid=validate_segment(scene,result.knots.back(),candidate,result.knots.size(),
                                options,candidate_check,repair_stop);
                            const double candidate_seconds=seconds(candidate_begin);
                            result.validation_seconds+=candidate_seconds;
                            if(valid) {next=std::move(candidate);accepted_check_seconds=candidate_seconds;break;}
                        }
                        if(valid || repair_stop())break;
                    }
                    repair_seconds+=seconds(repair_begin);
                }
                if(!valid) return check.reason;
                // Failed repair trials are already timed above; the reserved
                // final pass only needs to repeat the accepted segment.
                checked_seconds+=accepted_check_seconds;
                result.knots.push_back(next);
            }
            return std::string("travel_limit");
        };
        result.q_positive_stop=extend(pi/2,positive_checks);
        result.q_positive_degrees=result.knots.back().mechanism.twist*180/pi;
        // Return along the already checked positive joint path to Q=0. The
        // final validator rechecks every reverse interval, including timestamps.
        const auto positive_end=result.knots.size();
        if(positive_end==prefix_size) {
            zero=result.knots.back();zero.phase=Phase::Twist;
            zero.time=result.knots.back().time+interval_seconds(result.knots.back(),zero,options.motion_speed_scale);
            result.knots.push_back(zero);
        } else for(std::size_t j=positive_end-1;j>prefix_size-1;--j) {
            Knot reverse=result.knots[j-1];reverse.phase=Phase::Twist;
            reverse.time=result.knots.back().time+interval_seconds(result.knots.back(),reverse,options.motion_speed_scale);
            result.knots.push_back(reverse);
        }
        result.q_negative_stop=extend(-pi/2,negative_checks);
        Knot dwell=result.knots.back();dwell.phase=Phase::Dwell;dwell.time+=.5;result.knots.push_back(dwell);
        const auto final_check=Clock::now();
        result.validation=validate_trajectory(scene,result.knots,options,stop);
        result.validation_seconds+=seconds(final_check);
        if(!result.validation.valid) {
            const auto reason="extension_rejected_"+result.validation.reason;
            result.knots=fallback_knots;result.validation=fallback_validation;
            result.q_positive_degrees=0;result.q_positive_stop=reason;result.q_negative_stop=reason;
        }
        result.q_final_degrees=result.knots.back().mechanism.twist*180/pi;
        result.full_q_sweep=std::abs(result.q_positive_degrees-90)<1e-8 && std::abs(result.q_final_degrees+90)<1e-8;
        result.success=!stop();
        result.reason=!result.success?"deadline_after_validation":result.full_q_sweep?"validated_full_q":
            (result.q_positive_degrees==0 && result.q_final_degrees==0?"validated_no_q_motion":"validated_partial_q");
        return finish();
    }
    parameterize(result.knots,options.motion_speed_scale);
    const auto validation_start=Clock::now();
    result.validation=validate_trajectory(scene,result.knots,options,stop);
    result.validation_seconds=seconds(validation_start);
    result.success=result.validation.valid;
    result.reason=result.validation.reason;
    if(result.success && stop()) {result.success=false;result.reason="deadline_after_validation";}
    return finish();
}

PlanResult plan(PlanningScene& scene,const Scenario& scenario,const PlanOptions& options,
                const std::vector<AtlasEntry>* atlas) {
    if(!options.optimize_joint_motion ||
       (options.method!=Method::Baseline && options.method!=Method::RigidCache) ||
       !std::isfinite(options.timeout_seconds) || options.timeout_seconds<=0 ||
       !valid_motion_scale(options.motion_speed_scale) ||
       options.approach_joint_waypoint)
        return plan_candidate(scene,scenario,options,atlas);
    const auto begin=Clock::now();
    auto stop=[&] {return seconds(begin)>=options.timeout_seconds;};
    scene.configure(scenario);scene.enable_rigid_pair_cache(true);
    if((scenario.enforce_core_pose_limits && !scene.core_within_front_plane()) ||
       !scene.collision(scenario.start,{}).valid)
        return plan_candidate(scene,scenario,options,atlas);
    const ForwardKinematics fk=[&](const JointVector& q,Eigen::Vector3d& p,Eigen::Matrix3d& r,Jacobian* j){scene.forward(q,p,r,j);};
    const auto target=make_task(scene,options).front().target;
    const auto search_begin=Clock::now();
    double search_fraction=.15,search_ik_seconds=0;
    auto search_stop=[&] {return stop() || seconds(search_begin)>=search_fraction*options.timeout_seconds;};
    IKOptions ik;ik.method=options.method==Method::Baseline?IKMethod::Baseline:IKMethod::Reduced5D;
    ik.should_stop=search_stop;ik.alpha=.7;ik.max_joint_step=JointVector::Constant(.2);ik.max_iterations=160;
    ik.position_tolerance=1e-6;ik.axis_tolerance=1e-6;
    auto search_solve=[&](const TaskTarget& task,const JointVector& seed,const JointLimits& limits,
                          const IKOptions& settings) {
        const auto started=Clock::now();
        auto solved=solve_continuous_ik(fk,task,seed,limits,settings,true);
        search_ik_seconds+=seconds(started);scene.counters.ik_iterations+=solved.iterations;
        return solved;
    };
    const auto lookahead=make_task(scene,options);
    auto feasible_branch=[&](JointVector q) {
        for(std::size_t k=1;k<lookahead.size() && lookahead[k].phase<Phase::Twist;++k) {
            if(search_stop())return false;
            const auto& point=lookahead[k];
            JointVector seed=q;
            if(options.method==Method::RigidCache) {
                Eigen::Vector3d p;Eigen::Matrix3d r;Jacobian j;scene.forward(q,p,r,&j);
                const auto axis=r.col(2).eval();
                seed+=predict_task_step(j,axis,point.target.position-p,axis.cross(point.target.direction));
                seed=seed.cwiseMax(scene.limits().lower).cwiseMin(scene.limits().upper);
            }
            const auto next=search_solve(point.target,seed,scene.limits(),ik);
            if(!next.success)return false;
            if((k%5==0 || k+1==lookahead.size() || lookahead[k+1].phase!=point.phase) &&
               !scene.collision(next.q,point.mechanism).valid)return false;
            q=next.q;
        }
        return true;
    };
    auto nearby=[&](JointVector q,const JointVector& reference) {
        for(int j=0;j<7;++j) {
            if(scene.limits().upper[j]-scene.limits().lower[j]<2*pi)continue;
            double best=q[j];
            for(int turn=-2;turn<=2;++turn) {
                const double value=q[j]+turn*2*pi;
                if(value>=scene.limits().lower[j] && value<=scene.limits().upper[j] &&
                   std::abs(value-reference[j])<std::abs(best-reference[j]))best=value;
            }
            q[j]=best;
        }
        return q;
    };
    struct Candidate {JointVector q;double cost;};
    std::vector<Candidate> candidates;
    std::vector<JointVector> seeds{scenario.start,Scenario{}.start,.5*(scene.limits().lower+scene.limits().upper)};
    std::mt19937 random(20261011);std::normal_distribution<double> normal(0,.55);
    for(int n=0;n<9;++n) {
        JointVector seed=n%2?scenario.start:Scenario{}.start;
        for(int j=0;j<7;++j)seed[j]+=normal(random);
        seeds.push_back(seed.cwiseMax(scene.limits().lower).cwiseMin(scene.limits().upper));
    }
    std::size_t ik_attempts=0;
    // Keep the unmodified live IK candidate in the ranking, so a posture that
    // is already close to the goal is not displaced by a worse wrist-locked
    // branch merely because that branch was enumerated first.
    IKOptions original_ik=ik;original_ik.max_joint_step.reset();original_ik.max_iterations=300;
    for(const auto& seed:seeds) {
        if(search_stop())break;
        const auto solved=search_solve(target,seed,scene.limits(),original_ik);++ik_attempts;
        if(!solved.success)continue;
        const JointVector q=nearby(solved.q,scenario.start);
        if(!scene.collision(q,{}).valid || !feasible_branch(q))continue;
        const auto delta=(q-scenario.start).cwiseAbs().eval();
        candidates.push_back({q,delta.sum()+3*delta[6]});
        break;
    }
    search_fraction=.25;
    // Use the same damped posture search in both online backends; the branch
    // continuation and final trajectory still use the selected backend's IK.
    // A full reduced-task step can jump to a distant wrist-locked IK branch.
    auto posture_ik=ik;posture_ik.method=IKMethod::Baseline;
    for(int mode=0;mode<8 && !search_stop();++mode) {
        for(const auto& raw:seeds) {
            if(search_stop())break;
            JointLimits limits=scene.limits();JointVector seed=raw;
            if(mode<7) {
                const double offset=mode==0?0:((mode+1)/2)*.5*(mode%2?1:-1);
                const double wrist=std::clamp(scenario.start[6]+offset,limits.lower[6],limits.upper[6]);
                seed[6]=wrist;limits.lower[6]=limits.upper[6]=wrist;
            }
            const auto solved=search_solve(target,seed,limits,posture_ik);++ik_attempts;
            if(!solved.success)continue;
            const JointVector q=nearby(solved.q,scenario.start);
            if(!scene.collision(q,{}).valid)continue;
            if(std::any_of(candidates.begin(),candidates.end(),[&](const auto& c){return (c.q-q).norm()<.1;}))continue;
            // This only rejects branches that already fail at checked points;
            // acceptance still requires the original complete dense validator.
            if(!feasible_branch(q))continue;
            const auto delta=(q-scenario.start).cwiseAbs().eval();
            candidates.push_back({q,delta.sum()+3*delta[6]});
        }
        if(candidates.size()>=2)break;
    }
    std::sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b){return a.cost<b.cost;});
    PlanResult best;best.method=options.method;best.reason="preinsert_posture_search_exhausted";
    const double search_seconds=seconds(search_begin);
    Counters total_counters=scene.counters;
    double total_ik=search_ik_seconds,total_approach=0,total_validation=0;
    std::size_t attempts=0,total_approach_iterations=0,total_shortcuts=0;
    for(std::size_t i=0;i<=candidates.size() && !stop();++i) {
        auto trial=options;trial.timeout_seconds=options.timeout_seconds-seconds(begin);
        auto result=plan_candidate(scene,scenario,trial,atlas,
            i<candidates.size()?std::optional<JointVector>(candidates[i].q):std::nullopt);
        ++attempts;total_ik+=result.ik_seconds;total_approach+=result.approach_seconds;total_validation+=result.validation_seconds;
        accumulate(total_counters,result.counters);ik_attempts+=result.preinsert_ik_attempts;
        total_approach_iterations+=result.approach_iterations;total_shortcuts+=result.approach_shortcuts;
        if(attempts==1 || result.success || result.knots.size()>best.knots.size())best=std::move(result);
        if(best.success)break;
    }
    best.planning_seconds=seconds(begin);best.ik_seconds=total_ik;best.approach_seconds=total_approach;
    best.validation_seconds=total_validation;best.preinsert_ik_attempts=ik_attempts;
    best.posture_search_seconds=search_seconds;best.counters=total_counters;
    best.approach_iterations=total_approach_iterations;best.approach_shortcuts=total_shortcuts;
    best.posture_candidates=candidates.size();best.planning_attempts=attempts;
    if(!attempts && stop())best.reason="deadline_during_posture_search";
    if(best.success && stop()) {best.success=false;best.reason="deadline_after_validation";}
    return best;
}

std::vector<Scenario> scenario_suite() {
    std::vector<Scenario> cases;
    auto add=[&](std::string id,std::string split,Pose pose,double q) {
        Scenario s;s.id=std::move(id);s.split=std::move(split);s.core=pose;s.target_degrees=q;
        cases.push_back(s);
    };
    add("dev_nominal","development",{-.09,.2,.5,0,90,0},35);
    add("dev_low","development",{-.1,.2,.5,0,90,0},-35);
    add("dev_yaw","development",{-.1,.23,.52,15,85,0},60);
    add("dev_roll","development",{-.1,.17,.52,-15,85,10},-60);
    add("dev_tilt","development",{-.1,.2,.5,0,65,0},45);
    add("dev_qzero","development",{-.08,.2,.5,0,90,0},0);
    add("confirm_qpos","confirmation",{-.095,.21,.525,5,88,5},90);
    add("confirm_qneg","confirmation",{-.095,.19,.525,-5,88,-5},-90);
    add("confirm_high","confirmation",{-.1,.2,.6,0,90,0},30);
    add("confirm_left","confirmation",{-.1,.28,.52,10,85,-10},-45);
    add("confirm_right","confirmation",{-.1,.12,.52,-10,85,10},45);
    add("confirm_tilt","confirmation",{-.1,.2,.51,0,70,5},-30);
    add("confirm_roll","confirmation",{-.1,.2,.515,0,85,20},50);
    add("confirm_mixed","confirmation",{-.1,.24,.54,-10,80,-10},-50);
    // Untuned boundary requests stay in the evaluation even if this fixed
    // mounting or a method cannot solve them. They are not atlas entries.
    add("confirm_top","confirmation",{-.1,.2,.7,0,90,0},35);
    add("confirm_yaw_pos","confirmation",{-.1,.2,.55,45,90,0},35);
    add("confirm_yaw_neg","confirmation",{-.1,.2,.55,-45,90,0},-35);
    add("confirm_roll_limit","confirmation",{-.1,.2,.55,0,90,45},0);
    add("confirm_ymin","confirmation",{-.1,.1,.55,0,90,0},45);
    add("confirm_ymax","confirmation",{-.1,.3,.55,0,90,0},-45);
    add("reject_unreachable","negative",{-.1,.2,.7,0,90,0},35);
    cases.back().assembly_translation.x()=.65;
    add("reject_blocked","negative",{-.09,.2,.55,0,90,0},35);cases.back().blocking_obstacle=true;
    add("reject_front_plane","negative",{0,.2,.5,90,45,45},35);
    return cases;
}
} // namespace tech_core::planning
