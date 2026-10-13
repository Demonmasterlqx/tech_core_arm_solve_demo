#include "tech_core/planning.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {
using namespace tech_core::planning;
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}
std::string csv(const std::string& text) {
    std::string result = "\"";
    for (char c : text) { if (c == '"') result += '"'; result += c; }
    return result + '"';
}
std::string filename(std::string text) {
    for (char& c : text)
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '-' && c != '_') c = '_';
    return text;
}
std::ofstream output_file(const std::filesystem::path& path) {
    std::ofstream stream(path);
    if (!stream) throw std::runtime_error("Cannot write " + path.string());
    stream.exceptions(std::ios::badbit | std::ios::failbit);
    stream << std::setprecision(17);
    return stream;
}
void write_scenarios(const std::filesystem::path& path, const std::vector<Scenario>& suite, const std::string& robot_id) {
    auto out = output_file(path);
    out << "case,split,core_x,core_y,core_z,core_theta_degrees,core_phi_degrees,core_alpha_degrees,target_degrees";
    for(int j=0;j<suite.front().start.size();++j) out<<",start_q"<<j;
    out<<",assembly_translation_x,assembly_translation_y,assembly_translation_z,assembly_yaw_radians,blocking_obstacle,arm_base_z,q_sweep,enforce_core_pose_limits,robot,model_path\n";
    for (const auto& scenario : suite) {
        const auto& p = scenario.core;
        out << csv(scenario.id) << ',' << csv(scenario.split) << ',' << p.x << ',' << p.y << ',' << p.z
            << ',' << p.theta << ',' << p.phi << ',' << p.alpha << ',' << scenario.target_degrees;
        for (int k = 0; k < scenario.start.size(); ++k) out << ',' << scenario.start[k];
        for (int k = 0; k < 3; ++k) out << ',' << scenario.assembly_translation[k];
        out << ',' << scenario.assembly_yaw << ',' << scenario.blocking_obstacle << ',' << scenario.arm_base_z << ',' << scenario.q_sweep << ',' << scenario.enforce_core_pose_limits << ',' << csv(robot_id) << ',' << csv(scenario.model_path) << '\n';
    }
}
PlanResult safe_plan(PlanningScene& scene, const Scenario& scenario, const PlanOptions& options,
                     const std::vector<AtlasEntry>* atlas = nullptr) {
    const auto start = Clock::now();
    try { return plan(scene, scenario, options, atlas); }
    catch (const std::exception& error) {
        PlanResult result;
        result.method = options.method;
        result.planning_seconds = elapsed(start);
        result.reason = std::string("exception: ") + error.what();
        return result;
    }
}
void write_trajectory(const std::filesystem::path& path, const PlanResult& result, int dof) {
    auto out = output_file(path);
    out << "knot,time,phase,insertion,lift,flip,twist";
    for(int j=0;j<dof;++j) out<<",q"<<j;
    out<<",target_x,target_y,target_z,direction_x,direction_y,direction_z\n";
    for (std::size_t i = 0; i < result.knots.size(); ++i) {
        const auto& knot = result.knots[i];
        out << i << ',' << knot.time << ',' << phase_name(knot.phase) << ','
            << knot.mechanism.insertion << ',' << knot.mechanism.lift << ','
            << knot.mechanism.flip << ',' << knot.mechanism.twist;
        for (int k = 0; k < knot.q.size(); ++k) out << ',' << knot.q[k];
        for (int k = 0; k < 3; ++k) out << ',' << knot.target.position[k];
        for (int k = 0; k < 3; ++k) out << ',' << knot.target.direction[k];
        out << '\n';
    }
}
void usage() {
    std::cout << "arm_plan [--robot ID|DIRECTORY] [--model PATH] [--list-robots] [--method NAME|all] [--case ID|all] [--repeat N]\n"
                 "          [--split development|confirmation|negative|all] [--timeout SECONDS]\n"
                 "          [--output DIRECTORY] [--dense-check] [--motion-speed-scale S] [--list]\n";
}
} // namespace

int main(int argc, char** argv) {
    try {
        std::string robot="rm26", model;
        bool robot_selected=false;
        std::string method_arg = "all", case_arg = "all", split = "all";
        std::filesystem::path output = "artifacts/level3_planning/manual";
        int repeats = 1;
        double timeout = 30;
        double motion_speed_scale = 1;
        bool dense_check = false, list = false;
        for (int i = 1; i < argc; ++i) {
            const std::string flag = argv[i];
            auto value = [&]() -> std::string {
                if (++i >= argc) throw std::invalid_argument("Missing value for " + flag);
                return argv[i];
            };
            if (flag == "--robot") {robot=value();robot_selected=true;}
            else if (flag == "--model") model=value();
            else if (flag == "--list-robots") {for(const auto& id:tech_core::available_robots()) std::cout<<id<<'\n';return 0;}
            else if (flag == "--method") method_arg = value();
            else if (flag == "--case") case_arg = value();
            else if (flag == "--split") split = value();
            else if (flag == "--repeat") {
                const auto text = value();
                std::size_t used = 0;
                repeats = std::stoi(text, &used);
                if (used != text.size()) throw std::invalid_argument("Invalid repeat");
            } else if (flag == "--timeout") {
                const auto text = value();
                std::size_t used = 0;
                timeout = std::stod(text, &used);
                if (used != text.size()) throw std::invalid_argument("Invalid timeout");
            } else if (flag == "--motion-speed-scale") {
                const auto text = value();
                std::size_t used = 0;
                motion_speed_scale = std::stod(text, &used);
                if (used != text.size()) throw std::invalid_argument("Invalid motion speed scale");
            }
            else if (flag == "--output") output = value();
            else if (flag == "--dense-check") dense_check = true;
            else if (flag == "--list") list = true;
            else if (flag == "--help" || flag == "-h") { usage(); return 0; }
            else throw std::invalid_argument("Unknown option " + flag);
        }
        if (repeats < 1 || !std::isfinite(timeout) || timeout <= 0)
            throw std::invalid_argument("repeat and timeout must be positive");
        if (!std::isfinite(motion_speed_scale) || motion_speed_scale <= 0 ||
            !std::isfinite(2*motion_speed_scale*motion_speed_scale))
            throw std::invalid_argument("motion speed scale must be positive and finite");
        if (split != "all" && split != "development" && split != "confirmation" && split != "negative")
            throw std::invalid_argument("Invalid split");
        const std::vector<Method> all_methods{Method::Baseline, Method::FullStep, Method::Reduced5D,
            Method::AdaptiveLM, Method::Predictor, Method::SparseAdaptive, Method::Atlas, Method::BoundedQP, Method::RigidCache};
        if(robot_selected && !model.empty()) throw std::invalid_argument("Use --robot or --model, not both");
        if(model.empty()) model=tech_core::robot_model_path(robot,true);
        const auto cold_start = Clock::now();
        PlanningScene scene(model);
        const double cold_seconds = elapsed(cold_start);
        const auto suite = scenario_suite(scene);
        if (list) {
            std::cout << "kind,name,split\n";
            for (const auto method : all_methods) std::cout << "method," << method_name(method) << ",\n";
            for (const auto& scenario : suite) std::cout << "case," << csv(scenario.id) << ',' << csv(scenario.split) << '\n';
            return 0;
        }
        const auto methods = method_arg == "all" ? all_methods : std::vector<Method>{parse_method(method_arg)};
        std::vector<Scenario> cases;
        for (const auto& scenario : suite)
            if ((case_arg == "all" || scenario.id == case_arg) &&
                (split == "all" || scenario.split == split)) cases.push_back(scenario);
        if (cases.empty()) throw std::invalid_argument("No cases match case/split selection");
        if (std::filesystem::exists(output / "runs.csv"))
            throw std::invalid_argument("Output already contains runs.csv; choose a new directory");
        std::filesystem::create_directories(output / "trajectories");
        write_scenarios(output / "scenarios.csv", suite, scene.robot().id);
        auto joints=output_file(output/"robot.csv");
        joints<<"robot,model_path,index,joint,type,lower,upper,velocity_limit\n";
        for(int i=0;i<scene.dof();++i)
            joints<<csv(scene.robot().id)<<','<<csv(model)<<','<<i<<','<<csv(scene.robot().joint_names[i])<<','
                <<(scene.model()->jnt_type[scene.robot().joints[i]]==mjJNT_SLIDE?"prismatic":"revolute")<<','
                <<scene.limits().lower[i]<<','<<scene.limits().upper[i]<<','<<scene.robot().velocity_limits[i]<<'\n';
        joints.close();
        std::cout << std::setprecision(8) << "cold_model_load_seconds=" << cold_seconds << '\n';
        auto metadata = output_file(output / "timing.csv");
        metadata << "cold_model_load_seconds,atlas_build_seconds,atlas_entries\n";
        std::vector<AtlasEntry> atlas;
        double atlas_seconds = 0;
        if (std::find(methods.begin(), methods.end(), Method::Atlas) != methods.end()) {
            const auto build_start = Clock::now();
            auto build = output_file(output / "atlas_build.csv");
            build << "case,split,success,planning_seconds,reason\n";
            PlanOptions options;
            options.method = Method::FullStep;
            options.timeout_seconds = timeout;
            for (const auto& scenario : suite) {
                // Never fit the atlas on confirmation targets, even when only
                // confirmation runs are requested.
                if (scenario.split != "development") continue;
                const auto result = safe_plan(scene, scenario, options);
                build << csv(scenario.id) << ',' << csv(scenario.split) << ',' << result.success
                      << ',' << result.planning_seconds << ',' << csv(result.reason) << '\n';
                build.flush();
                if (result.success) atlas.push_back({scenario, result.knots});
            }
            atlas_seconds = elapsed(build_start);
            std::cout << "atlas_build_seconds=" << atlas_seconds << " atlas_entries=" << atlas.size() << '\n';
        }
        metadata << cold_seconds << ',' << atlas_seconds << ',' << atlas.size() << '\n';
        metadata.flush();
        auto runs = output_file(output / "runs.csv");
        runs << "method,case,split,repeat,success,plan_success,reason,planning_seconds,ik_seconds,validation_seconds,dense_recheck_seconds,dense_recheck_performed,dense_recheck_valid,dense_recheck_reason,knots,ik_iterations,fk_evals,collision_states,narrowphase_calls,broadphase_skips,validation_samples,max_position_error,max_axis_error,max_joint_speed,max_joint_acceleration,rule_success,dense_recheck_samples,trajectory,atlas_source,rigid_pair_cache_hits,motion_speed_scale\n";
        // Rotate method order by case/repeat to avoid assigning every cold run
        // to baseline. Every invocation remains single-threaded and CPU-only.
        for (int repeat = 0; repeat < repeats; ++repeat) {
            for (std::size_t case_index = 0; case_index < cases.size(); ++case_index) {
                const auto& scenario = cases[case_index];
                for (std::size_t order = 0; order < methods.size(); ++order) {
                    const auto method = methods[(order + case_index + repeat) % methods.size()];
                    PlanOptions options;
                    options.method = method;
                    options.timeout_seconds = timeout;
                    options.motion_speed_scale = motion_speed_scale;
                    const auto result = safe_plan(scene, scenario, options, method == Method::Atlas ? &atlas : nullptr);
                    ValidationResult recheck;
                    bool rechecked = false;
                    double recheck_seconds = 0;
                    if (dense_check && result.success) {
                        auto dense = options;
                        dense.validation_joint_step *= .5;
                        dense.validation_min_substeps *= 2;
                        const auto recheck_start = Clock::now();
                        rechecked = true;
                        try {
                            scene.configure(scenario);
                            recheck = validate_trajectory(scene, result.knots, dense,
                                [&] { return elapsed(recheck_start) >= timeout; });
                        } catch (const std::exception& error) { recheck.reason = error.what(); }
                        recheck_seconds = elapsed(recheck_start);
                    }
                    const bool success = result.success && (!rechecked || recheck.valid);
                    const std::string reason = result.success && rechecked && !recheck.valid
                        ? "dense recheck: " + recheck.reason : result.reason;
                    const std::string relative = "trajectories/" + filename(method_name(method)) + "_" +
                        filename(scenario.id) + "_r" + std::to_string(repeat) + ".csv";
                    write_trajectory(output / relative, result, scene.dof());
                    const auto& c = result.counters;
                    const auto& v = result.validation;
                    runs << csv(method_name(method)) << ',' << csv(scenario.id) << ',' << csv(scenario.split)
                         << ',' << repeat << ',' << success << ',' << result.success << ',' << csv(reason)
                         << ',' << result.planning_seconds << ',' << result.ik_seconds << ',' << result.validation_seconds
                         << ',' << recheck_seconds << ',' << rechecked << ',' << (rechecked && recheck.valid) << ',' << csv(recheck.reason)
                         << ',' << result.knots.size() << ',' << c.ik_iterations << ',' << c.fk_evals
                         << ',' << c.collision_states << ',' << c.narrowphase_calls << ',' << c.broadphase_skips
                         << ',' << v.samples << ',' << v.max_position_error << ',' << v.max_axis_error
                         << ',' << v.max_joint_speed << ',' << v.max_joint_acceleration << ',' << v.rule_success
                         << ',' << recheck.samples << ',' << csv(relative) << ',' << csv(result.atlas_source)
                         << ',' << c.rigid_pair_cache_hits << ',' << options.motion_speed_scale << '\n';
                    runs.flush();
                    std::cout << method_name(method) << ' ' << scenario.id << " repeat=" << repeat
                              << " success=" << success << " seconds=" << result.planning_seconds
                              << " reason=" << reason << std::endl;
                }
            }
        }
        return 0; // Expected infeasible cases remain data, not process failures.
    } catch (const std::exception& error) {
        std::cerr << "rm26_plan: " << error.what() << '\n';
        return 2;
    }
}
