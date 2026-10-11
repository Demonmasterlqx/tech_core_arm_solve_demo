#include "tech_core/planning_demo.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <algorithm>
#include <stdexcept>

namespace tech_core::planning {
namespace {
void valid_step(double dt) {
    if (!std::isfinite(dt) || dt < 0 || dt > .1)
        throw std::invalid_argument("Demo step must be in [0, 0.1] seconds");
}
Eigen::Vector3d bounded(const Eigen::Vector3d& v, double limit) {
    const double norm = v.norm();
    return norm > limit ? (v * (limit / norm)).eval() : v;
}
bool executable(const DemoPreview& preview) {
    const auto& r = preview.result;
    return r.success && r.validation.valid && r.validation.rule_success && r.knots.size() >= 6;
}
}

const char* demo_state_name(DemoState state) {
    switch (state) {
        case DemoState::Editing: return "EDITING";
        case DemoState::Planning: return "PLANNING";
        case DemoState::PreviewReady: return "PREVIEW_READY";
        case DemoState::PreviewFailed: return "PREVIEW_FAILED";
        case DemoState::Executing: return "EXECUTING";
        case DemoState::Paused: return "PAUSED";
        case DemoState::Complete: return "COMPLETE";
    }
    return "UNKNOWN";
}

PlanOptions demo_plan_options() {
    PlanOptions options;
    options.method = Method::RigidCache;
    options.timeout_seconds = 5;
    options.motion_speed_scale = 4;
    return options;
}

DemoPreview make_demo_preview(const Scenario& request, const PlanOptions& options) {
    DemoPreview preview;
    preview.result.method = options.method;
    const auto begin = std::chrono::steady_clock::now();
    try {
        PlanningScene workspace;
        workspace.configure(request);
        // Build the intent before calling the planner, so early collision or
        // front-plane rejection still has an honest, labelled visualization.
        for (const auto& point : make_task(workspace, options))
            preview.requested_path.push_back(point.target.position);
        preview.result = plan(workspace, request, options);
        for (const auto& knot : preview.result.knots) {
            Eigen::Vector3d p;
            Eigen::Matrix3d r;
            workspace.forward(knot.q, p, r);
            preview.candidate_path.push_back(p);
        }
    } catch (const std::exception& error) {
        preview.result.success = false;
        preview.result.reason = std::string("planner_exception: ") + error.what();
        preview.result.planning_seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - begin).count();
    }
    preview.ghost_knots = preview.result.knots;
    if (preview.ghost_knots.empty()) {
        Knot start;
        start.q = request.start;
        preview.ghost_knots.push_back(start);
    } else if (!preview.result.success) {
        parameterize(preview.ghost_knots, options.motion_speed_scale);
    }
    return preview;
}

Knot sample_demo_trajectory(const std::vector<Knot>& knots, double seconds) {
    if (knots.empty() || !std::isfinite(seconds))
        throw std::invalid_argument("Cannot sample an empty trajectory or nonfinite time");
    if (seconds <= knots.front().time) return knots.front();
    if (seconds >= knots.back().time) return knots.back();
    const auto it = std::lower_bound(knots.begin(), knots.end(), seconds,
        [](const Knot& knot, double time) { return knot.time < time; });
    const auto& b = *it;
    const auto& a = *(it - 1);
    const double u = (seconds - a.time) / (b.time - a.time);
    const double s = u * u * (3 - 2 * u);
    Knot sample = b;
    sample.time = seconds;
    sample.q = (1 - s) * a.q + s * b.q;
    sample.mechanism = {a.mechanism.insertion + s * (b.mechanism.insertion - a.mechanism.insertion),
                        a.mechanism.lift + s * (b.mechanism.lift - a.mechanism.lift),
                        a.mechanism.flip + s * (b.mechanism.flip - a.mechanism.flip),
                        a.mechanism.twist + s * (b.mechanism.twist - a.mechanism.twist)};
    return sample;
}

PlanningDemo::PlanningDemo(Scenario initial_scenario, PlanOptions options, DemoPlanner planner)
    : initial_(std::move(initial_scenario)), scenario_(initial_), options_(std::move(options)),
      planner_(std::move(planner)), q_(initial_.start) {
    if (!planner_ || (options_.method != Method::Baseline && options_.method != Method::RigidCache) ||
        !std::isfinite(options_.timeout_seconds) || options_.timeout_seconds <= 0 ||
        options_.timeout_seconds > 30 || !std::isfinite(options_.motion_speed_scale) ||
        options_.motion_speed_scale <= 0 || options_.motion_speed_scale > 4)
        throw std::invalid_argument("Invalid demo planner options");
    initial_.enforce_core_pose_limits = false;
    scenario_.enforce_core_pose_limits = false;
    scene_.configure(scenario_);
    tool_site_ = mj_name2id(scene_.model(), mjOBJ_SITE, "planning_tcp");
    if (tool_site_ < 0) throw std::runtime_error("Missing planning_tcp");
    display_state();
}

PlanningDemo::~PlanningDemo() {
    // Never detach a worker or let it outlive model/library teardown. Planning
    // checks a bounded deadline. Exceptions are consumed in poll().
    if (worker_.valid()) worker_.wait();
}

void PlanningDemo::display_state() {
    scene_.set_state(q_, mechanism_);
    scene_.data()->time = execution_time_;
    mju_zero(scene_.data()->qvel, scene_.model()->nv);
    // PlanningScene only needs FK; rendering also needs lights/composite geoms.
    mj_forward(scene_.model(), scene_.data());
}

Eigen::Vector3d PlanningDemo::tool_position() const {
    // These pointers are only read on the UI thread.
    const auto* d = const_cast<PlanningScene&>(scene_).data();
    return Eigen::Map<const Eigen::Vector3d>(d->site_xpos + 3 * tool_site_);
}

Eigen::Vector3d PlanningDemo::core_position() const {
    const auto& p = scenario_.core;
    return scenario_.assembly_translation +
        Eigen::AngleAxisd(scenario_.assembly_yaw, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d(p.x, p.y, p.z);
}

bool PlanningDemo::editing_allowed() {
    if (state_ == DemoState::Executing || state_ == DemoState::Paused || state_ == DemoState::Complete) {
        message_ = "Backspace: zero joints; R: load assembly preset before editing";
        return false;
    }
    return true;
}

void PlanningDemo::invalidate(const std::string& message) {
    ++revision_;
    preview_.reset();
    state_ = DemoState::Editing;
    execution_time_ = 0;
    message_ = message;
}

void PlanningDemo::home() {
    invalidate("URDF joint zero; core pose preserved. R: assembly preset; Enter: plan");
    q_.setZero();
    mechanism_ = {};
    stop_jog();
    display_state();
}

void PlanningDemo::reset() {
    scenario_ = initial_;
    scene_.configure(scenario_);
    invalidate("Assembly preset loaded. Backspace: URDF zero; Enter: plan and preview");
    q_ = initial_.start;
    mechanism_ = {};
    stop_jog();
    display_state();
}

void PlanningDemo::set_backend(Method method) {
    if (method != Method::Baseline && method != Method::RigidCache)
        throw std::invalid_argument("Demo backends are baseline and rigid_cache");
    if (!editing_allowed() || method == options_.method) return;
    options_.method = method;
    stop_jog();
    invalidate("Backend changed. Enter: plan again");
}

void PlanningDemo::enter() {
    stop_jog();
    // Poll happens in tick(), never here: an Enter received during planning
    // cannot turn into an execution request merely because the future is ready.
    if (state_ == DemoState::Planning) {
        message_ = "Planning... wait for the preview, then press Enter again";
    } else if (state_ == DemoState::PreviewReady) {
        if (!preview_ || !executable(*preview_) ||
            (preview_->result.knots.front().q - q_).norm() > 1e-10) {
            state_ = DemoState::PreviewFailed;
            message_ = "Execution blocked: invalid or stale preview";
            return;
        }
        execution_time_ = 0;
        state_ = DemoState::Executing;
        message_ = "Executing previewed trajectory. Space: pause; Backspace: zero joints";
    } else if (state_ == DemoState::PreviewFailed) {
        message_ = "Failed candidate cannot execute. Edit the scene or press Backspace";
    } else if (state_ == DemoState::Editing) {
        if (worker_.valid()) {
            message_ = "Previous request finishing; press Enter after it is discarded";
            return;
        }
        Scenario request = scenario_;
        request.start = q_;
        worker_revision_ = revision_;
        worker_ = std::async(std::launch::async, planner_, request, options_);
        state_ = DemoState::Planning;
        message_ = "Planning... arm and core are held";
    } else if (state_ == DemoState::Paused) {
        message_ = "Space: resume the paused trajectory";
    } else if (state_ == DemoState::Complete) {
        message_ = "Complete. Backspace: zero joints; R: assembly preset";
    }
}

void PlanningDemo::poll() {
    if (!worker_.valid() || worker_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
    DemoPreview result;
    try { result = worker_.get(); }
    catch (const std::exception& error) {
        result.result.method = options_.method;
        result.result.reason = std::string("worker_exception: ") + error.what();
    }
    if (worker_revision_ != revision_) {
        message_ = "Old plan discarded after scene change. Enter: plan again";
        return;
    }
    if (result.ghost_knots.empty()) {
        Knot start;
        start.q = q_;
        result.ghost_knots.push_back(start);
    }
    preview_ = std::move(result);
    const bool valid = executable(*preview_);
    state_ = valid ? DemoState::PreviewReady : DemoState::PreviewFailed;
    message_ = valid ? "Preview only - arm held. Enter: execute" : "Planning failed - preview only; execution blocked";
}

void PlanningDemo::advance(double dt) {
    const auto& knots = preview_->result.knots;
    execution_time_ = std::min(execution_time_ + dt, knots.back().time);
    const auto sample = sample_demo_trajectory(knots, execution_time_);
    q_ = sample.q;
    mechanism_ = sample.mechanism;
    if (execution_time_ >= knots.back().time) {
        state_ = DemoState::Complete;
        message_ = "Assembly trajectory complete. Backspace: zero joints; R: assembly preset";
    }
    display_state();
}

void PlanningDemo::tick(double seconds) {
    valid_step(seconds);
    poll();
    if (state_ == DemoState::Executing) advance(seconds);
}

void PlanningDemo::toggle_pause() {
    if (state_ == DemoState::Executing) {
        state_ = DemoState::Paused;
        message_ = "Paused. Space: resume; Backspace: zero joints";
    } else if (state_ == DemoState::Paused) {
        state_ = DemoState::Executing;
        message_ = "Executing previewed trajectory";
    }
}

void PlanningDemo::focus_lost() {
    stop_jog();
    if (state_ == DemoState::Executing) toggle_pause();
}

void PlanningDemo::stop_jog() { jog_orientation_.reset(); }

void PlanningDemo::jog_arm(const Eigen::Vector3d& velocity, double seconds) {
    valid_step(seconds);
    if (!velocity.allFinite()) throw std::invalid_argument("Nonfinite arm jog");
    if (seconds == 0 || velocity.squaredNorm() == 0 || !editing_allowed()) return;
    Eigen::Vector3d p;
    Eigen::Matrix3d r;
    Jacobian j;
    scene_.forward(q_, p, r, &j);
    if (!jog_orientation_) jog_orientation_ = r;
    Eigen::Vector3d rotation_error = Eigen::Vector3d::Zero();
    for (int i = 0; i < 3; ++i) rotation_error += .5 * r.col(i).cross(jog_orientation_->col(i));
    constexpr double weight = .15, damping = .005;
    j.bottomRows<3>() *= weight;
    Eigen::Matrix<double, 6, 1> v;
    v << bounded(velocity, .1), weight * 8 * rotation_error;
    const Eigen::Matrix<double, 6, 6> normal = j * j.transpose() +
        damping * damping * Eigen::Matrix<double, 6, 6>::Identity();
    JointVector dq = j.transpose() * normal.ldlt().solve(v);
    const double largest = dq.cwiseAbs().maxCoeff();
    if (largest > 1) dq /= largest;
    q_ = (q_ + seconds * dq).cwiseMax(scene_.limits().lower).cwiseMin(scene_.limits().upper);
    invalidate("Arm moved. Enter: plan from current joints");
    display_state();
}

void PlanningDemo::jog_core(const Eigen::Vector3d& velocity,
                            const Eigen::Vector3d& rpy_rate, double seconds) {
    valid_step(seconds);
    if (!velocity.allFinite() || !rpy_rate.allFinite()) throw std::invalid_argument("Nonfinite core jog");
    if (seconds == 0 || (velocity.squaredNorm() == 0 && rpy_rate.squaredNorm() == 0) || !editing_allowed()) return;
    const Eigen::Vector3d delta = seconds * (Eigen::AngleAxisd(-scenario_.assembly_yaw,
        Eigen::Vector3d::UnitZ()) * velocity);
    auto updated = scenario_;
    auto& p = updated.core;
    p.x += delta.x();
    p.y += delta.y();
    p.z += delta.z();
    p.alpha += seconds * rpy_rate.x();
    p.phi += seconds * rpy_rate.y();
    p.theta += seconds * rpy_rate.z();
    scene_.configure(updated);
    scenario_ = std::move(updated);
    invalidate("Core moved. Enter: plan again");
    display_state();
}

} // namespace tech_core::planning
