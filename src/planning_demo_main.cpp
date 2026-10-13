#include "tech_core/camera_framing.hpp"
#include "tech_core/planning_demo.hpp"

#include <GLFW/glfw3.h>
#include <png.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace {
using namespace tech_core;
using namespace tech_core::planning;
using Clock = std::chrono::steady_clock;

struct Options {
    Scenario scenario = scenario_suite().front();
    PlanOptions planning = demo_plan_options();
    double duration = 0, speed = .06, core_speed = .20, rotation_speed = 60;
    std::string screenshot, log;
};

double number(const std::string& text) {
    std::size_t used = 0;
    const double result = std::stod(text, &used);
    if (used != text.size() || !std::isfinite(result)) throw std::invalid_argument("Expected finite number: " + text);
    return result;
}

void usage() {
    std::cout <<
        "Robot arm + technology core | plan, preview, then execute\n"
        "  WASD: arm +X/-X/+Y/-Y; Q/E: arm +Z/-Z (world)\n"
        "  I/K, J/L, U/O: core +/-X, +/-Y, +/-Z (world)\n"
        "  Shift + I/K, J/L, U/O: core +/-roll, +/-pitch, +/-yaw\n"
        "  1: Baseline; 2: RigidCache (default)\n"
        "  Enter: plan and hold a preview; Enter again: execute a valid plan\n"
        "  Backspace: upstream URDF joint zero, preserving core pose\n"
        "  R: load assembly preset; Space: pause/resume ghost or execution; Esc: exit\n"
        "  [/]: scrub the ghost preview backward/forward (holds real robot)\n"
        "  Mouse: left orbit, right pan, wheel zoom; F: reset camera; F12: PNG\n"
        "Options:\n"
        "  --robot ID|DIRECTORY    robot package (default rm26)\n"
        "  --model PATH            explicit planning MJCF\n"
        "  --list-robots           available robot packages\n"
        "  --backend baseline|rigid_cache\n"
        "  --case ID               scenario_suite case (default dev_nominal)\n"
        "  --timeout SEC           planning deadline, (0,30], default 5\n"
        "  --motion-speed-scale S  demo retiming, (0,4], default 4\n"
        "  --speed M/S             arm jog, (0,0.1], default 0.06\n"
        "  --core-speed M/S        core translation, positive, default 0.20\n"
        "  --core-rotation-speed DEG/S  core rotation, positive, default 60\n"
        "  --duration SEC          bounded wall-time run (default until Esc)\n"
        "  --screenshot PATH.png   capture on F12 and at exit\n"
        "  --log PATH              event log, 1 MiB rotation + two backups\n"
        "Starts at the model's declared demo pose (assembly preset or bounded URDF zero).\n"
        "Final Q rotation tries both directions within +/-90 deg; keeps the validated range.\n"
        "Core XYZ/RPY controls have no pose limits. RPY = alpha, phi-90, theta\n"
        "relative to the assembly frame. Execution is the kinematic simulation.\n";
}

Options arguments(int argc, char** argv) {
    Options out;
    std::string robot="rm26", model, case_id;
    bool robot_selected=false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value = [&]() -> std::string {
            if (++i == argc) throw std::invalid_argument("Missing value for " + arg);
            return argv[i];
        };
        if (arg == "--robot") {robot=value();robot_selected=true;}
        else if (arg == "--model") model=value();
        else if (arg == "--backend") out.planning.method = parse_method(value());
        else if (arg == "--timeout") out.planning.timeout_seconds = number(value());
        else if (arg == "--motion-speed-scale") out.planning.motion_speed_scale = number(value());
        else if (arg == "--speed") out.speed = number(value());
        else if (arg == "--core-speed") out.core_speed = number(value());
        else if (arg == "--core-rotation-speed") out.rotation_speed = number(value());
        else if (arg == "--duration") {
            out.duration = number(value());
            if (out.duration <= 0 || out.duration > 3600) throw std::invalid_argument("Duration must be in (0,3600]");
        } else if (arg == "--screenshot") out.screenshot = value();
        else if (arg == "--log") out.log = value();
        else if (arg == "--case") {
            case_id = value();
        } else throw std::invalid_argument("Unknown option: " + arg);
    }
    if(robot_selected && !model.empty()) throw std::invalid_argument("Use --robot or --model, not both");
    if(model.empty()) model=robot_model_path(robot,true);
    PlanningScene selected(model);
    auto suite=scenario_suite(selected);
    if(case_id.empty()) out.scenario=suite.front();
    else {
        const auto it=std::find_if(suite.begin(),suite.end(),[&](const auto& s){return s.id==case_id;});
        if(it==suite.end()) throw std::invalid_argument("Unknown scenario: "+case_id);
        out.scenario=*it;
    }
    if (out.speed <= 0 || out.speed > .1) throw std::invalid_argument("Speed must be in (0,0.1]");
    if (out.core_speed <= 0 || out.rotation_speed <= 0)
        throw std::invalid_argument("Core translation and rotation speeds must be positive");
    return out;
}

void make_parent(const std::filesystem::path& path) {
    if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
}

std::string json_string(const std::string& text) {
    std::ostringstream out;
    out << '"';
    for (const unsigned char c : text) {
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
        else out << c;
    }
    out << '"';
    return out.str();
}

class EventLog {
public:
    explicit EventLog(std::string path) : path_(std::move(path)) {
        if (!path_.empty()) { make_parent(path_); open(); }
    }
    void write(const std::string& line) {
        std::cout << "PLANNING_DEMO " << line << std::endl;
        if (path_.empty()) return;
        if (size_ + line.size() + 1 > 1024 * 1024) {
            stream_.close();
            std::filesystem::remove(path_ + ".2");
            if (std::filesystem::exists(path_ + ".1")) std::filesystem::rename(path_ + ".1", path_ + ".2");
            std::filesystem::rename(path_, path_ + ".1");
            open();
        }
        stream_ << line << '\n' << std::flush;
        size_ += line.size() + 1;
    }
private:
    void open() {
        size_ = std::filesystem::exists(path_) ? std::filesystem::file_size(path_) : 0;
        stream_.open(path_, std::ios::app);
        if (!stream_) throw std::runtime_error("Cannot open event log " + path_);
        stream_.exceptions(std::ios::badbit | std::ios::failbit);
    }
    std::string path_;
    std::ofstream stream_;
    std::size_t size_ = 0;
};

class Viewer {
public:
    explicit Viewer(Options options)
        : options_(std::move(options)), demo_(options_.scenario, options_.planning), ghost_(options_.scenario.model_path), log_(options_.log) {
        mjv_defaultScene(&scene_);
        mjv_defaultScene(&ghost_scene_);
        mjr_defaultContext(&context_);
        try {
            // CAD carrying packages start at their calibrated assembly pose.
            // Legacy packages keep zero startup; Backspace always exposes zero.
            if (!demo_.scene().robot().demo_start_at_preset) demo_.home();
            glfwSetErrorCallback([](int code, const char* text) { std::cerr << "GLFW " << code << ": " << text << '\n'; });
            if (!glfwInit()) throw std::runtime_error("GLFW initialization failed; use a desktop or xvfb-run");
            glfw_ready_ = true;
            glfwWindowHint(GLFW_SAMPLES, 0);
            window_ = glfwCreateWindow(1440, 1000, "Arm Planning Demo | preview then execute", nullptr, nullptr);
            if (!window_) throw std::runtime_error("Cannot create planning demo window");
            glfwMakeContextCurrent(window_);
            glfwSwapInterval(1);
            std::cout << "OpenGL renderer: " << glGetString(GL_RENDERER) << std::endl;
            mjv_defaultCamera(&camera_);
            mjv_defaultOption(&view_option_);
            for (int i = 0; i < mjNGROUP; ++i) {
                view_option_.geomgroup[i] = i <= 2;
                view_option_.sitegroup[i] = 0;
            }
            mjv_makeScene(demo_.scene().model(), &scene_, 7000);
            mjv_makeScene(ghost_.model(), &ghost_scene_, 1500);
            scene_.flags[mjRND_SHADOW] = 0;
            scene_.flags[mjRND_REFLECTION] = 0;
            mjr_makeContext(demo_.scene().model(), &context_, mjFONTSCALE_150);
            const int arm_root = mj_name2id(ghost_.model(), mjOBJ_BODY, ghost_.robot().root_body.c_str());
            const int core_root = mj_name2id(ghost_.model(), mjOBJ_BODY, "core0_q_bearing");
            if (arm_root < 0 || core_root < 0) throw std::runtime_error("Missing ghost arm/core bodies");
            ghost_geoms_.resize(ghost_.model()->ngeom, false);
            for (int g = 0; g < ghost_.model()->ngeom; ++g)
                for (int b = ghost_.model()->geom_bodyid[g]; b > 0; b = ghost_.model()->body_parentid[b])
                    if (b == arm_root || b == core_root) { ghost_geoms_[g] = true; break; }
            reset_camera();
            glfwGetCursorPos(window_, &mouse_x_, &mouse_y_);
            glfwSetWindowUserPointer(window_, this);
            glfwSetKeyCallback(window_, key_callback);
            glfwSetCursorPosCallback(window_, mouse_callback);
            glfwSetWindowFocusCallback(window_, [](GLFWwindow* w, int focus) {
                get(w).guard([&] {
                    if (!focus) { get(w).demo_.focus_lost(); get(w).clear_motion(); get(w).trace("focus_lost"); }
                });
            });
            glfwSetScrollCallback(window_, [](GLFWwindow* w, double, double dy) {
                auto& v = get(w);
                mjv_moveCamera(v.demo_.scene().model(), mjMOUSE_ZOOM, 0, -.05 * dy, &v.scene_, &v.camera_);
            });
            trace("ready");
        } catch (...) { cleanup(); throw; }
    }
    ~Viewer() { cleanup(); }

    void run() {
        const auto begin = Clock::now();
        auto last = begin;
        while (!glfwWindowShouldClose(window_)) {
            const auto now = Clock::now();
            const double elapsed = std::clamp(std::chrono::duration<double>(now - last).count(), 0., .25);
            last = now;
            if (options_.duration > 0 && std::chrono::duration<double>(now - begin).count() >= options_.duration) break;
            glfwPollEvents();
            // Process key edges before collecting futures. An Enter pressed
            // while planning is ignored even if that result completed meanwhile.
            const auto before = demo_.state();
            const bool pending = demo_.worker_pending();
            manual_step(std::min(elapsed, .05));
            // Substep the execution clock independently of render rate. Manual
            // jog stays bounded after a slow frame; a running trajectory keeps
            // its timing without asking the controller for oversized steps.
            double remaining = elapsed;
            demo_.tick(0);
            while (remaining > 1e-9 && demo_.state() == DemoState::Executing) {
                const double step = std::min(remaining, .05);
                demo_.tick(step);
                remaining -= step;
            }
            update_ghost(elapsed);
            if (demo_.state() != before) trace("state");
            if (pending && !demo_.worker_pending() && demo_.state() == DemoState::Editing) trace("discarded");
            render();
            if (capture_) { screenshot(); capture_ = false; }
            glfwSwapBuffers(window_);
            std::this_thread::sleep_until(now + std::chrono::milliseconds(16));
        }
        if (!fatal_.empty()) throw std::runtime_error(fatal_);
        demo_.focus_lost();
        if (!options_.screenshot.empty()) { render(); screenshot(); }
        trace("exit");
    }

private:
    static Viewer& get(GLFWwindow* w) { return *static_cast<Viewer*>(glfwGetWindowUserPointer(w)); }
    template<class F> void guard(F action) {
        try { action(); }
        catch (const std::exception& error) { fatal_ = error.what(); glfwSetWindowShouldClose(window_, 1); }
    }
    void cleanup() {
        mjr_freeContext(&context_);
        mjv_freeScene(&scene_);
        mjv_freeScene(&ghost_scene_);
        if (window_) { glfwDestroyWindow(window_); window_ = nullptr; }
        if (glfw_ready_) { glfwTerminate(); glfw_ready_ = false; }
    }
    bool down(int key) const { return glfwGetKey(window_, key) == GLFW_PRESS; }
    static bool motion_key(int key) {
        return key == GLFW_KEY_W || key == GLFW_KEY_S || key == GLFW_KEY_A || key == GLFW_KEY_D ||
            key == GLFW_KEY_Q || key == GLFW_KEY_E || key == GLFW_KEY_I || key == GLFW_KEY_K ||
            key == GLFW_KEY_J || key == GLFW_KEY_L || key == GLFW_KEY_U || key == GLFW_KEY_O;
    }
    void clear_motion() {
        demo_.stop_jog();
        blocked_until_release_ = true;
        if (moving_) { moving_ = false; trace("motion_stop"); }
    }
    void trace(const char* event, int key = -1) {
        const auto& p = demo_.scenario().core;
        const auto tcp = demo_.tool_position(), core = demo_.core_position();
        const auto& mechanism = demo_.mechanism();
        std::ostringstream out;
        out << std::setprecision(12) << "{\"event\":" << json_string(event) << ",\"key\":" << key
            << ",\"state\":" << json_string(demo_state_name(demo_.state()))
            << ",\"robot\":" << json_string(demo_.scene().robot().id)
            << ",\"backend\":" << json_string(method_name(demo_.options().method))
            << ",\"revision\":" << demo_.revision() << ",\"time\":" << demo_.execution_time()
            << ",\"tcp\":[" << tcp.x() << ',' << tcp.y() << ',' << tcp.z() << "],\"q\":[";
        for (int i = 0; i < demo_.joints().size(); ++i) out << (i ? "," : "") << demo_.joints()[i];
        out << "],\"core_world\":[" << core.x() << ',' << core.y() << ',' << core.z()
            << "],\"core_rpy\":[" << p.alpha << ',' << p.phi - 90 << ',' << p.theta
            << "],\"mechanism\":[" << mechanism.insertion << ',' << mechanism.lift << ',' << mechanism.flip << ',' << mechanism.twist << ']'
            << ",\"message\":" << json_string(demo_.message());
        if (demo_.preview()) {
            const auto& preview = *demo_.preview();
            const auto& r = preview.result;
            out << ",\"success\":" << (r.success ? "true" : "false") << ",\"reason\":" << json_string(r.reason)
                << ",\"knots\":" << r.knots.size() << ",\"requested_points\":" << preview.requested_path.size()
                << ",\"candidate_points\":" << preview.candidate_path.size()
                << ",\"planning_seconds\":" << r.planning_seconds
                << ",\"approach_seconds\":" << r.approach_seconds
                << ",\"approach_iterations\":" << r.approach_iterations
                << ",\"approach_shortcuts\":" << r.approach_shortcuts
                << ",\"preinsert_ik_attempts\":" << r.preinsert_ik_attempts
                << ",\"posture_candidates\":" << r.posture_candidates
                << ",\"planning_attempts\":" << r.planning_attempts
                << ",\"posture_search_seconds\":" << r.posture_search_seconds
                << ",\"q_positive_degrees\":" << r.q_positive_degrees
                << ",\"q_final_degrees\":" << r.q_final_degrees
                << ",\"full_q_sweep\":" << (r.full_q_sweep ? "true" : "false")
                << ",\"q_positive_stop\":" << json_string(r.q_positive_stop)
                << ",\"q_negative_stop\":" << json_string(r.q_negative_stop);
            if(r.preinsert_joints && !r.knots.empty()) {
                const auto delta=(*r.preinsert_joints-r.knots.front().q).eval();
                out << ",\"preinsert_joint_change_degrees\":" << delta.cwiseAbs().sum()*180/pi
                    << ",\"preinsert_last_joint_change_degrees\":" << delta[delta.size()-1]*180/pi;
            }
        }
        if (show_ghost()) {
            out << ",\"ghost_time\":" << ghost_sample_.time << ",\"ghost_geoms\":" << ghost_geom_count_
                << ",\"ghost_paused\":" << (ghost_paused_ ? "true" : "false") << ",\"ghost_q\":[";
            for (int i = 0; i < ghost_sample_.q.size(); ++i) out << (i ? "," : "") << ghost_sample_.q[i];
            out << ']';
        }
        out << '}';
        log_.write(out.str());
    }
    void reset_camera() {
        camera_.type = mjCAMERA_FREE;
        frame_model(demo_.scene().model(),demo_.scene().data(),camera_);
        camera_.azimuth = 315; camera_.elevation = -22;
        const auto camera = tech_core::robot_numbers(demo_.scene().model(), "robot_demo_camera", false);
        if (!camera.empty()) {
            if (camera.size()!=2 || camera[1]<=-90 || camera[1]>=90)
                throw std::invalid_argument("Robot demo camera requires azimuth/elevation degrees");
            camera_.azimuth=camera[0];camera_.elevation=camera[1];
        }
    }
    static void key_callback(GLFWwindow* w, int key, int, int action, int) {
        auto& v = get(w);
        v.guard([&] {
            if (motion_key(key) && action != GLFW_REPEAT) v.trace(action == GLFW_PRESS ? "key_press" : "key_release", key);
            if (action != GLFW_PRESS) return; // Held Enter must never execute a new preview.
            switch (key) {
                case GLFW_KEY_ESCAPE: v.clear_motion(); glfwSetWindowShouldClose(w, 1); break;
                case GLFW_KEY_ENTER: case GLFW_KEY_KP_ENTER:
                    v.clear_motion(); v.demo_.enter(); v.trace("enter"); break;
                case GLFW_KEY_BACKSPACE:
                    v.clear_motion(); v.demo_.home(); v.trace("home"); break;
                case GLFW_KEY_R:
                    v.clear_motion(); v.demo_.reset(); v.trace("reset"); break;
                case GLFW_KEY_1: case GLFW_KEY_2:
                    v.clear_motion(); v.demo_.set_backend(key == GLFW_KEY_1 ? Method::Baseline : Method::RigidCache);
                    v.trace("backend"); break;
                case GLFW_KEY_SPACE:
                    v.clear_motion();
                    if (v.show_ghost()) { v.ghost_paused_ = !v.ghost_paused_; v.trace("preview_pause"); }
                    else { v.demo_.toggle_pause(); v.trace("pause"); }
                    break;
                case GLFW_KEY_LEFT_BRACKET: case GLFW_KEY_RIGHT_BRACKET:
                    if (v.show_ghost()) {
                        v.ghost_paused_ = true;
                        v.ghost_elapsed_ = std::clamp(std::min(v.ghost_elapsed_, 6.) +
                            (key == GLFW_KEY_RIGHT_BRACKET ? .3 : -.3), 0., 6.);
                        v.update_ghost(0);
                        v.trace("preview_scrub");
                    }
                    break;
                case GLFW_KEY_F: v.reset_camera(); break;
                case GLFW_KEY_F12: v.capture_ = true; break;
                default: break;
            }
        });
    }
    static void mouse_callback(GLFWwindow* w, double x, double y) {
        auto& v = get(w);
        const double dx = x - v.mouse_x_, dy = y - v.mouse_y_;
        v.mouse_x_ = x; v.mouse_y_ = y;
        const bool left = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        const bool right = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        const bool middle = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
        if (!left && !right && !middle) return;
        int width, height;
        glfwGetWindowSize(w, &width, &height);
        if (height <= 0) return;
        const bool shift = v.down(GLFW_KEY_LEFT_SHIFT) || v.down(GLFW_KEY_RIGHT_SHIFT);
        const int action = right ? (shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V) :
            left ? (shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V) : mjMOUSE_ZOOM;
        mjv_moveCamera(v.demo_.scene().model(), action, dx / height, dy / height, &v.scene_, &v.camera_);
    }
    void manual_step(double dt) {
        const Eigen::Vector3d arm(double(down(GLFW_KEY_W)) - down(GLFW_KEY_S),
            double(down(GLFW_KEY_A)) - down(GLFW_KEY_D), double(down(GLFW_KEY_Q)) - down(GLFW_KEY_E));
        const Eigen::Vector3d core(double(down(GLFW_KEY_I)) - down(GLFW_KEY_K),
            double(down(GLFW_KEY_J)) - down(GLFW_KEY_L), double(down(GLFW_KEY_U)) - down(GLFW_KEY_O));
        bool any = false;
        for (int key : {GLFW_KEY_W, GLFW_KEY_S, GLFW_KEY_A, GLFW_KEY_D, GLFW_KEY_Q, GLFW_KEY_E,
                        GLFW_KEY_I, GLFW_KEY_K, GLFW_KEY_J, GLFW_KEY_L, GLFW_KEY_U, GLFW_KEY_O}) any |= down(key);
        if (!any) blocked_until_release_ = false;
        if (blocked_until_release_ || !glfwGetWindowAttrib(window_, GLFW_FOCUSED)) return;
        if (!any) {
            demo_.stop_jog();
            if (moving_) { moving_ = false; trace("motion_stop"); }
            return;
        }
        if (!moving_) { moving_ = true; trace("motion_start"); }
        if (arm.squaredNorm() > 0) demo_.jog_arm(options_.speed * arm.normalized(), dt);
        else demo_.stop_jog();
        const bool shift = down(GLFW_KEY_LEFT_SHIFT) || down(GLFW_KEY_RIGHT_SHIFT);
        if (core.squaredNorm() > 0) {
            const Eigen::Vector3d v = core.normalized();
            demo_.jog_core(shift ? Eigen::Vector3d::Zero().eval() : (options_.core_speed * v).eval(),
                           shift ? (options_.rotation_speed * v).eval() : Eigen::Vector3d::Zero().eval(), dt);
        }
    }

    mjvGeom* geom() {
        if (scene_.ngeom >= scene_.maxgeom) return nullptr;
        auto* g = &scene_.geoms[scene_.ngeom++];
        const mjtNum size[3]{.004, .004, .004}, p[3]{};
        const float rgba[4]{1, 1, 1, 1};
        mjv_initGeom(g, mjGEOM_SPHERE, size, p, nullptr, rgba);
        g->category = mjCAT_DECOR;
        return g;
    }
    void line(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
              const std::array<float, 4>& color, double width, bool arrow = false) {
        if ((a - b).squaredNorm() < 1e-14) return;
        if (auto* g = geom()) {
            std::copy(color.begin(), color.end(), g->rgba);
            mjv_connector(g, arrow ? mjGEOM_ARROW : mjGEOM_CAPSULE, width, a.data(), b.data());
        }
    }
    void dot(const Eigen::Vector3d& p, const std::array<float, 4>& color, double radius) {
        if (auto* g = geom()) {
            mjtNum size[3]{radius, radius, radius};
            mjv_initGeom(g, mjGEOM_SPHERE, size, p.data(), nullptr, color.data());
            g->category = mjCAT_DECOR;
        }
    }
    void path(const std::vector<Eigen::Vector3d>& points, const std::array<float, 4>& color,
              double width, bool dashed) {
        const std::size_t stride = std::max<std::size_t>(1, (points.size() + 1499) / 1500);
        for (std::size_t i = stride; i < points.size(); i += stride) {
            if (dashed && (i / stride) % 2 == 0) continue;
            line(points[i - stride], points[i], color, width);
        }
        if (!dashed && points.size() > 1 && (points.size() - 1) % stride)
            line(points[((points.size() - 1) / stride) * stride], points.back(), color, width);
    }
    bool show_ghost() const {
        return demo_.preview() && (demo_.state() == DemoState::PreviewReady || demo_.state() == DemoState::PreviewFailed);
    }
    void update_ghost(double dt) {
        if (!demo_.preview()) { ghost_revision_.reset(); ghost_geom_count_ = 0; return; }
        if (!show_ghost()) return;
        if (!ghost_revision_ || *ghost_revision_ != demo_.revision()) {
            ghost_revision_ = demo_.revision();
            ghost_elapsed_ = 0;
            ghost_paused_ = false;
            // This separate model/data never replaces or writes the opaque
            // robot's state, even during failed-candidate animation.
            ghost_.configure(demo_.scenario());
        } else if (!ghost_paused_) ghost_elapsed_ = std::fmod(ghost_elapsed_ + dt, 7.);
        const auto& knots = demo_.preview()->ghost_knots;
        const double time = knots.back().time * std::min(ghost_elapsed_ / 6., 1.);
        ghost_sample_ = sample_demo_trajectory(knots, time);
        ghost_.set_state(ghost_sample_.q, ghost_sample_.mechanism);
        mj_forward(ghost_.model(), ghost_.data());
    }
    void append_ghost() {
        ghost_geom_count_ = 0;
        if (!show_ghost()) return;
        mjv_updateScene(ghost_.model(), ghost_.data(), &view_option_, nullptr, &camera_, mjCAT_ALL, &ghost_scene_);
        const bool valid = demo_.preview()->result.success;
        const std::array<float, 4> color = valid ? std::array<float, 4>{.15f, 1, .65f, .32f}
                                                : std::array<float, 4>{1, .2f, .24f, .32f};
        for (int i = 0; i < ghost_scene_.ngeom && scene_.ngeom < scene_.maxgeom; ++i) {
            const auto& source = ghost_scene_.geoms[i];
            if (source.objtype != mjOBJ_GEOM || source.objid < 0 || !ghost_geoms_[source.objid]) continue;
            auto& g = scene_.geoms[scene_.ngeom++];
            g = source;
            // Mesh IDs match the identical source model used by the renderer.
            // Override material/alpha on abstract render geoms only.
            g.matid = -1; g.objid = -1; g.objtype = mjOBJ_UNKNOWN;
            g.category = mjCAT_DECOR; g.segid = -1;
            std::copy(color.begin(), color.end(), g.rgba);
            g.emission = .25f; g.specular = .1f; g.reflectance = 0;
            g.transparent = 1;
            ++ghost_geom_count_;
        }
    }
    void render() {
        int width, height;
        glfwGetFramebufferSize(window_, &width, &height);
        if (width <= 0 || height <= 0) return;
        const mjrRect rect{0, 0, width, height};
        mjv_updateScene(demo_.scene().model(), demo_.scene().data(), &view_option_, nullptr, &camera_, mjCAT_ALL, &scene_);
        const auto origin = demo_.core_position();
        const std::array<std::array<float, 4>, 3> axes{{{1, .3f, .3f, 1}, {.3f, 1, .45f, 1}, {.35f, .6f, 1, 1}}};
        for (int i = 0; i < 3; ++i) line(origin, origin + .075 * Eigen::Vector3d::Unit(i), axes[i], .002, true);
        if (demo_.preview()) {
            const auto& preview = *demo_.preview();
            const bool success = preview.result.success;
            const std::array<float, 4> color = success ? std::array<float, 4>{.2f, 1, .5f, 1} : std::array<float, 4>{1, .18f, .22f, 1};
            path(preview.requested_path, {1, .7f, .18f, 1}, .0012, true);
            path(preview.candidate_path, color, .003, false);
            dot(preview.candidate_path.empty() ? demo_.tool_position() : preview.candidate_path.back(), color, .009);
            if (!preview.requested_path.empty()) dot(preview.requested_path.back(), {1, .7f, .18f, 1}, .006);
            const auto& knots = preview.result.knots;
            const std::size_t step = std::max<std::size_t>(1, knots.size() / 12);
            for (std::size_t i = 0; i < knots.size() && i < preview.candidate_path.size(); i += step)
                line(preview.candidate_path[i], preview.candidate_path[i] + .025 * knots[i].target.direction, color, .0012, true);
        }
        dot(demo_.tool_position(), {.4f, .8f, 1, 1}, .005);
        append_ghost();
        mjr_render(rect, &scene_, &context_);
        const auto tcp = demo_.tool_position(), core = demo_.core_position();
        const auto& p = demo_.scenario().core;
        std::ostringstream left, right, bottom;
        left << demo_.scene().robot().id << " / TECHNOLOGY CORE\n" << demo_state_name(demo_.state())
             << "\nBackend [" << (demo_.options().method == Method::Baseline ? 1 : 2) << "]: " << method_name(demo_.options().method)
             << std::fixed << std::setprecision(3)
             << "\nArm TCP XYZ:  " << tcp.x() << "  " << tcp.y() << "  " << tcp.z() << " m"
             << "\nArm joints (rad):";
        for (int i = 0; i < demo_.joints().size(); ++i) left << ' ' << demo_.joints()[i];
        left << "\nCore XYZ:     " << core.x() << "  " << core.y() << "  " << core.z() << " m"
             << std::setprecision(1) << "\nCore RPY (assembly): " << p.alpha << "  " << p.phi - 90 << "  " << p.theta << " deg"
             << "\nFinal Q: both directions, up to +/-90 deg"
             << "\nCurrent Q: " << demo_.mechanism().twist * 180 / pi << " deg"
             << std::setprecision(2) << "\nCore speed: " << options_.core_speed << " m/s, "
             << std::setprecision(0) << options_.rotation_speed << " deg/s";
        right << "ARM / WORLD AXES\nW/S: +X/-X   A/D: +Y/-Y   Q/E: +Z/-Z\n\n"
              << "CORE / WORLD AXES\nI/K: +X/-X   J/L: +Y/-Y   U/O: +Z/-Z\n"
              << "Shift + I/K: roll +/-\nShift + J/L: pitch +/-\nShift + U/O: yaw +/-\n\n"
              << "1: Baseline   2: RigidCache\nEnter: PLAN > PREVIEW > EXECUTE\n"
              << "Backspace: URDF zero   R: assembly preset\nSpace: pause / resume   [ / ]: scrub ghost\nEsc: exit\n"
              << "Mouse: orbit / pan / zoom   F: camera\nF12: screenshot";
        bottom << demo_.message();
        if (show_ghost()) {
            bottom << "\nGHOST PREVIEW / real arm held | " << (ghost_paused_ ? "PAUSED" : "LOOPING")
                   << " | " << std::fixed << std::setprecision(0) << 100 * std::min(ghost_elapsed_ / 6., 1.) << "%";
        }
        if (demo_.preview()) {
            const auto& preview = *demo_.preview();
            const auto& result = preview.result;
            bottom << "\n" << (result.success ? "GREEN: validated candidate" : "RED: failed candidate / candidate endpoint")
                   << "   |   AMBER DASHES: requested task";
            if (result.knots.empty()) bottom << "\nNo candidate: ghost holds start; amber = requested task";
            bottom << "\n" << result.reason << "\nPlan " << std::fixed << std::setprecision(3) << result.planning_seconds
                   << " s | " << result.knots.size() << " knots | motion x" << demo_.options().motion_speed_scale;
            if(result.preinsert_joints && !result.knots.empty()) {
                const auto delta=(*result.preinsert_joints-result.knots.front().q).eval();
                bottom << "\nPre-insert change: sum " << std::setprecision(1) << delta.cwiseAbs().sum()*180/pi
                       << " deg | last joint " << delta[delta.size()-1]*180/pi << " deg";
            }
            if (!result.knots.empty() && result.success) {
                bottom << " | execution " << std::setprecision(2) << demo_.execution_time() << " / " << result.knots.back().time << " s";
                bottom << "\nValidated Q: 0 -> +" << std::setprecision(1) << result.q_positive_degrees
                       << " -> 0 -> " << result.q_final_degrees << " deg"
                       << (result.full_q_sweep ? " (full travel)" : " (partial travel)")
                       << "\nQ+ stop: " << result.q_positive_stop << "\nQ- stop: " << result.q_negative_stop;
            }
        }
        const auto l = left.str(), r = right.str(), b = bottom.str();
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, l.c_str(), nullptr, &context_);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, rect, r.c_str(), nullptr, &context_);
        mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, b.c_str(), nullptr, &context_);
    }
    void screenshot() {
        int width, height;
        glfwGetFramebufferSize(window_, &width, &height);
        if (width <= 0 || height <= 0) return;
        const std::string path = options_.screenshot.empty() ? "artifacts/keyboard_planning_demo/screenshot.png" : options_.screenshot;
        make_parent(path);
        std::vector<unsigned char> rgb(static_cast<std::size_t>(width) * height * 3);
        mjr_readPixels(rgb.data(), nullptr, {0, 0, width, height}, &context_);
        png_image out{};
        out.version = PNG_IMAGE_VERSION; out.width = width; out.height = height; out.format = PNG_FORMAT_RGB;
        if (!png_image_write_to_file(&out, path.c_str(), 0, rgb.data(), -width * 3, nullptr)) {
            const std::string error = out.message; png_image_free(&out); throw std::runtime_error(error);
        }
        png_image_free(&out);
        std::cout << "Screenshot: " << path << std::endl;
        trace("screenshot");
    }

    Options options_;
    PlanningDemo demo_;
    PlanningScene ghost_;
    EventLog log_;
    GLFWwindow* window_ = nullptr;
    mjvScene scene_{};
    mjvScene ghost_scene_{};
    mjrContext context_{};
    mjvCamera camera_{};
    mjvOption view_option_{};
    bool glfw_ready_ = false, blocked_until_release_ = false, moving_ = false, capture_ = false;
    double mouse_x_ = 0, mouse_y_ = 0;
    std::string fatal_;
    std::vector<bool> ghost_geoms_;
    std::optional<std::uint64_t> ghost_revision_;
    Knot ghost_sample_;
    double ghost_elapsed_ = 0;
    int ghost_geom_count_ = 0;
    bool ghost_paused_ = false;
};
}

int main(int argc, char** argv) {
    for(int i=1;i<argc;++i) if(std::string(argv[i])=="--list-robots") {
        for(const auto& id:tech_core::available_robots()) std::cout<<id<<'\n';
        return 0;
    }
    try {
        for (int i = 1; i < argc; ++i) if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
            usage(); return 0;
        }
        Viewer viewer(arguments(argc, argv));
        viewer.run();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
