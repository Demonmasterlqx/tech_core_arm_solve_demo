#pragma once

#include "tech_core/planning_ik.hpp"
#include "tech_core/simulation.hpp"
#include <chrono>
#include <memory>
#include <map>
#include <random>
#include <vector>

namespace tech_core::planning {

enum class Phase { Approach, Insert, Lift, Flip, Twist, Dwell };
const char* phase_name(Phase phase);
struct MechanismState {
    double insertion = 0, lift = 0, flip = 0, twist = 0;
};
struct Scenario {
    std::string id;
    std::string split = "development";
    Pose core;
    double target_degrees = 35;
    // Require Q to visit +90 degrees before ending at -90 degrees.
    // This extends the level-3 demonstration; target_degrees must be -90.
    bool q_sweep = false;
    // Keep all pre-Q phases mandatory, but accept the validated Q range found
    // within the online budget. Still try positive first, then reverse.
    bool q_best_effort = false;
    // One fixed, development-selected carrying posture, 60 mm before nominal E.
    JointVector start = (JointVector() << -.62770373654521094, 2.3297993305507192,
        1.0289871622237392, 1.2715734368717495, -1.747192543846495,
        -.81681197466957878, 1.9772845777886507).finished();
    // Rule assembly O frame -> reference world (the original robot base frame).
    // By default the arm base is 250 mm above the assembly ground.
    Eigen::Vector3d assembly_translation{.30, .2, -.25};
    double assembly_yaw = pi;
    // Physical translation of the robot and its fixed mount, in reference world.
    // Core and ground stay fixed. The original URDF chain is unchanged.
    double arm_base_z = 0;
    bool blocking_obstacle = false;
    // The interactive demo allows arbitrary finite core XYZ/RPY. Other
    // callers retain the rule-table bounds and station front-plane rule
    // unless explicitly disabled. Collision and joint checks are unaffected.
    bool enforce_core_pose_limits = true;
};
struct TaskPoint {
    TaskTarget target;
    MechanismState mechanism;
    Phase phase = Phase::Approach;
};
struct Knot : TaskPoint {
    JointVector q = JointVector::Zero();
    double time = 0;
};
struct CollisionResult {
    bool valid = true;
    double distance = 1;
    std::string pair;
};
struct Counters {
    std::size_t ik_iterations = 0, fk_evals = 0, collision_states = 0;
    std::size_t narrowphase_calls = 0, broadphase_skips = 0;
    std::size_t rigid_pair_cache_hits = 0;
    std::size_t exact_state_cache_hits = 0;
};

// A per-planner, CPU-only MuJoCo workspace. Not shared between worker threads.
class PlanningScene {
public:
    explicit PlanningScene(std::string model_path = TECH_CORE_PLANNING_MODEL_PATH);
    void configure(const Scenario& scenario);
    // Reset by configure. Cache only geometry pairs welded to one rigid body;
    // all entries are populated inside the timed request and keyed by cutoff.
    void enable_rigid_pair_cache(bool enabled) { rigid_pair_cache_enabled_ = enabled; }
    // Exact (not quantized) q, mechanism and clearance key. Only for repeated
    // checks in one configured scene; configure() clears/disables all entries.
    void enable_exact_state_cache(bool enabled) { exact_state_cache_enabled_=enabled;exact_state_cache_.clear(); }
    void forward(const JointVector& q, Eigen::Vector3d& position,
                 Eigen::Matrix3d& rotation, Jacobian* jacobian = nullptr);
    void set_state(const JointVector& q, const MechanismState& mechanism);
    CollisionResult collision(const JointVector& q, const MechanismState& mechanism,
                              double clearance = .001);
    TaskTarget mechanism_target(const MechanismState& mechanism) const;
    MechanismState inferred_mechanism(const Eigen::Vector3d& position,
                                     const Eigen::Matrix3d& rotation, Phase phase) const;
    bool core_within_front_plane() const;
    const JointLimits& limits() const { return limits_; }
    const Scenario& scenario() const { return scenario_; }
    Counters counters;
    mjModel* model() { return model_.get(); }
    mjData* data() { return data_.get(); }
private:
    struct Pair {
        int a, b; bool assembly_contact, grasp_contact, rigid;
        double cached_distance = 0, cached_cutoff = -1;
    };
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr, mj_deleteData};
    std::array<int, 7> arm_joints_{}, qadr_{}, dof_{};
    std::array<int, 3> core_qadr_{};
    int tool_ = -1, core_mocap_ = -1;
    int arm_root_body_ = -1, mount_geom_ = -1;
    double arm_root_original_z_ = 0, mount_original_z_ = 0;
    JointLimits limits_;
    Scenario scenario_;
    Eigen::Vector3d core_position_;
    Eigen::Matrix3d core_rotation_;
    std::vector<Pair> pairs_;
    bool rigid_pair_cache_enabled_ = false;
    bool exact_state_cache_enabled_ = false;
    std::map<std::array<double,12>,CollisionResult> exact_state_cache_;
    CollisionResult collision_uncached(const JointVector& q,const MechanismState& mechanism,double clearance);
    std::vector<int> collision_geoms_;
    std::vector<Eigen::Vector3d> centres_, extents_;
    std::vector<std::array<double,2>> axial_supports_;
    std::vector<bool> axial_support_valid_;
    std::vector<mjtNum> jacp_, jacr_;
};

enum class Method { Baseline, FullStep, Reduced5D, AdaptiveLM, Predictor, SparseAdaptive, Atlas, BoundedQP, RigidCache };
const char* method_name(Method method);
Method parse_method(const std::string& name);
struct PlanOptions {
    Method method = Method::Baseline;
    double timeout_seconds = 30;
    // Divide motion intervals by this factor; keep the 0.5 s rule dwell.
    // Mock speed/acceleration bounds scale by s and s^2, respectively.
    double motion_speed_scale = 1;
    double position_spacing = .002;
    double angular_spacing = radians(1);
    // Optional online-selected waypoint before the pre-insertion IK solve.
    // The physical start is retained and connected to it with collision-aware
    // joint-space search and shortcut optimization. The normal goal is the IK
    // solution at E + 40 mm along its outward axis. All connections are densely
    // checked and validated with the entire task.
    std::optional<JointVector> approach_joint_waypoint;
    double validation_position_tolerance = .0005;
    double validation_axis_tolerance = .002;
    double validation_joint_step = .008;
    int validation_min_substeps = 2;
};
struct ApproachResult {
    bool success = false;
    std::string reason;
    // Ordered joint-space waypoints. On failure, only a checked prefix from
    // the physical start is returned, never a disconnected goal-tree branch.
    std::vector<JointVector> path;
    std::size_t iterations = 0, shortcuts = 0;
    double initial_length = 0, optimized_length = 0;
};
// Bidirectional collision-checked search followed by shortcut optimization.
// Mechanisms remain at zero. Edges use the same dense samples as final cubic
// validation after resampling to <= 0.02 rad per joint.
ApproachResult optimize_approach(PlanningScene& scene, const JointVector& start,
                               const JointVector& goal, const PlanOptions& options,
                               const std::function<bool()>& should_stop = {});
struct ValidationResult {
    bool valid = false;
    std::string reason;
    std::size_t samples = 0;
    double max_position_error = 0, max_axis_error = 0;
    double max_joint_speed = 0, max_joint_acceleration = 0;
    bool rule_success = false;
};
struct PlanResult {
    bool success = false;
    std::string reason;
    Method method = Method::Baseline;
    std::string atlas_source;
    std::vector<Knot> knots;
    double planning_seconds = 0, ik_seconds = 0, validation_seconds = 0;
    double approach_seconds = 0;
    std::size_t approach_iterations = 0, approach_shortcuts = 0;
    std::size_t preinsert_ik_attempts = 0;
    Counters counters;
    ValidationResult validation;
    double q_positive_degrees = 0, q_final_degrees = 0;
    bool full_q_sweep = false;
    std::string q_positive_stop, q_negative_stop;
};
struct AtlasEntry { Scenario scenario; std::vector<Knot> knots; };

// Starts at the pre-insertion pose, independent of the actual arm state.
// plan() prepends the joint-space connection from scenario.start to its IK.
std::vector<TaskPoint> make_task(PlanningScene& scene, const PlanOptions& options);
void parameterize(std::vector<Knot>& knots, double motion_speed_scale = 1);
ValidationResult validate_trajectory(PlanningScene& scene, const std::vector<Knot>& knots,
                                    const PlanOptions& options,
                                    const std::function<bool()>& should_stop = {});
PlanResult plan(PlanningScene& scene, const Scenario& scenario, const PlanOptions& options,
                const std::vector<AtlasEntry>* atlas = nullptr);
std::vector<Scenario> scenario_suite();

} // namespace tech_core::planning
