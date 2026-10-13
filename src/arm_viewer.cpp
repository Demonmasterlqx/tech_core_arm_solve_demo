#include "tech_core/camera_framing.hpp"
#include "tech_core/arm_viewer.hpp"
#include "tech_core/arm_keyboard.hpp"
#include <GLFW/glfw3.h>
#include <png.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace tech_core {
namespace {
class ArmViewer {
public:
    ArmViewer(ArmSimulation& simulation, bool visible, const ArmViewOptions& opts)
        : sim(simulation), options(opts), interactive(visible) {
        if (!std::isfinite(options.speed) || options.speed <= 0 || options.speed > .10)
            throw std::invalid_argument("Viewer speed must be in (0, 0.10] m/s");
        glfwSetErrorCallback([](int code, const char* message) {
            std::cerr << "GLFW " << code << ": " << message << '\n';
        });
        if (!glfwInit()) throw std::runtime_error("GLFW initialization failed; use --headless or an Xvfb display");
        glfwWindowHint(GLFW_VISIBLE, visible ? GLFW_TRUE : GLFW_FALSE);
        glfwWindowHint(GLFW_SAMPLES, 4);
        window = glfwCreateWindow(1440, 1000, ("Arm " + sim.robot().id + " | Fixed-base kinematic control").c_str(), nullptr, nullptr);
        if (!window) {
            glfwTerminate();
            throw std::runtime_error("Could not create the arm OpenGL window");
        }
        glfwMakeContextCurrent(window);
        glfwSwapInterval(visible ? 1 : 0);
        std::cout << "OpenGL renderer: " << glGetString(GL_RENDERER) << std::endl;
        mjv_defaultCamera(&camera);
        mjv_defaultOption(&option);
        mjv_defaultScene(&scene);
        mjr_defaultContext(&context);
        mjv_makeScene(sim.model(), &scene, std::max(3000, sim.model()->ngeom + 1000));
        mjr_makeContext(sim.model(), &context, mjFONTSCALE_150);
        position_target = sim.joint_positions();
        reset_camera();
        glfwGetCursorPos(window, &mouse_x, &mouse_y);
        glfwSetWindowUserPointer(window, this);
        glfwSetKeyCallback(window, key_callback);
        glfwSetCursorPosCallback(window, mouse_callback);
        glfwSetWindowFocusCallback(window, [](GLFWwindow* w, int focused) {
            auto& v = get(w);
            if (!focused) v.clear_motion("focus_lost");
        });
        glfwSetScrollCallback(window, [](GLFWwindow* w, double, double dy) {
            auto& v = get(w);
            mjv_moveCamera(v.sim.model(), mjMOUSE_ZOOM, 0, -.05 * dy, &v.scene, &v.camera);
        });
        trace("ready");
    }

    ~ArmViewer() {
        // Preserve the original exception if a nonfinite state caused teardown.
        if (interactive) {
            try { sim.stop(); } catch (...) {}
        }
        mjr_freeContext(&context);
        mjv_freeScene(&scene);
        glfwDestroyWindow(window);
        glfwTerminate();
    }

    void trace(const char* event, int key = -1) const {
        const auto xyz = sim.tool_position();
        std::cout << "ARM_VIEWER event=" << event;
        if (key >= 0) std::cout << " key=" << key;
        std::cout << " mode=" << mode_name()
                  << " time=" << std::fixed << std::setprecision(6) << sim.time()
                  << " tool=" << xyz[0] << ',' << xyz[1] << ',' << xyz[2]
                  << " joints=";
        const auto joints = sim.joint_positions();
        for (std::size_t i = 0; i < joints.size(); ++i)
            std::cout << (i ? "," : "") << joints[i];
        std::cout << std::endl;
    }

    void render() {
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        if (width <= 0 || height <= 0) return;
        mjrRect rect{0, 0, width, height};
        mjv_updateScene(sim.model(), sim.data(), &option, nullptr, &camera, mjCAT_ALL, &scene);
        mjr_render(rect, &scene, &context);
        if (!options.overlay) return;
        const auto xyz = sim.tool_position();
        const auto joints = sim.joint_positions();
        const auto names = sim.joint_names();
        std::ostringstream left, right;
        left << sim.robot().id << " ARM / FIXED BASE\nMuJoCo " << mj_versionString()
             << " | CPU kinematics\nMock drive: direct position / velocity\n\n"
             << "Control: " << mode_name() << (paused ? " / PAUSED" : "")
             << "\nt = " << std::fixed << std::setprecision(2) << sim.time() << " s"
             << "\nTool XYZ (m): " << std::setprecision(3) << xyz[0] << "  " << xyz[1] << "  " << xyz[2]
             << "\nCartesian speed: " << options.speed << " m/s\n";
        for (std::size_t i = 0; i < joints.size(); ++i) {
            const bool slide=sim.model()->jnt_type[sim.robot().joints[i]]==mjJNT_SLIDE;
            left << '\n' << (selected == i ? "> " : "  ") << i + 1 << ": " << names[i]
                 << "  " << std::setprecision(1) << joints[i] * (slide?1000:180/3.14159265358979323846) << (slide?" mm":" deg");
        }
        right << "CARTESIAN / world axes\nW / Up: forward +X\nS / Down: backward -X\n"
              << "A / Left: left +Y\nD / Right: right -Y\nQ / E: up +Z / down -Z\nRelease keys: hold\n\n"
              << "M: Cartesian / Position / Velocity\n1-9 / [ ]: select joint\n+ / -: joint jog (bounded per joint)\n\n"
              << "Space: pause / resume\nR: reset home\nMouse left: orbit\nMouse right: pan\nWheel: zoom\nEsc: exit";
        const auto l = left.str(), r = right.str();
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, l.c_str(), nullptr, &context);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, rect, r.c_str(), nullptr, &context);
    }

    void png(const std::string& path) {
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        if (width <= 0 || height <= 0) throw std::runtime_error("Cannot capture a zero-size framebuffer");
        std::vector<unsigned char> rgb(static_cast<std::size_t>(width) * height * 3);
        mjr_readPixels(rgb.data(), nullptr, mjrRect{0, 0, width, height}, &context);
        png_image out{};
        out.version = PNG_IMAGE_VERSION;
        out.width = width;
        out.height = height;
        out.format = PNG_FORMAT_RGB;
        if (!png_image_write_to_file(&out, path.c_str(), 0, rgb.data(), -width * 3, nullptr)) {
            const std::string message = out.message;
            png_image_free(&out);
            throw std::runtime_error("PNG write failed: " + message);
        }
        png_image_free(&out);
        std::cout << "Screenshot: " << path << std::endl;
    }

    void manual_step(double dt) {
        const ArmKeyboardInput keys{
            down(GLFW_KEY_W) || down(GLFW_KEY_UP), down(GLFW_KEY_S) || down(GLFW_KEY_DOWN),
            down(GLFW_KEY_A) || down(GLFW_KEY_LEFT), down(GLFW_KEY_D) || down(GLFW_KEY_RIGHT),
            down(GLFW_KEY_Q), down(GLFW_KEY_E)};
        const double jog = double(down(GLFW_KEY_EQUAL) || down(GLFW_KEY_KP_ADD)) -
                           double(down(GLFW_KEY_MINUS) || down(GLFW_KEY_KP_SUBTRACT));
        const bool any = keys.forward || keys.backward || keys.left || keys.right || keys.up || keys.down || jog != 0;
        if (!any) blocked_until_release = false;
        if (paused || blocked_until_release || !glfwGetWindowAttrib(window, GLFW_FOCUSED)) return;
        const auto velocity = arm_keyboard_velocity(keys, options.speed);
        const bool active = mode == ArmControlMode::Cartesian
            ? velocity[0] != 0 || velocity[1] != 0 || velocity[2] != 0 : jog != 0;
        if (!active) {
            if (moving) {
                sim.stop();
                position_target = sim.joint_positions();
                moving = false;
                trace("motion_stop");
            }
            return;
        }
        if (!moving) trace("motion_start");
        moving = true;
        if (mode == ArmControlMode::Cartesian) sim.set_cartesian_velocity(velocity);
        else if (mode == ArmControlMode::Position) {
            const auto& limits = sim.joint_limits().at(selected);
            position_target.at(selected) = std::clamp(position_target.at(selected) + jog * .5 * dt,
                                                      limits[0], limits[1]);
            sim.set_joint_positions(position_target);
        } else {
            std::vector<double> velocities(sim.joint_positions().size(), 0);
            velocities.at(selected) = jog * .5;
            sim.set_joint_velocities(velocities);
        }
    }

    ArmSimulation& sim;
    GLFWwindow* window = nullptr;
    bool paused = false;

private:
    static ArmViewer& get(GLFWwindow* w) {
        return *static_cast<ArmViewer*>(glfwGetWindowUserPointer(w));
    }
    bool down(int key) const { return glfwGetKey(window, key) == GLFW_PRESS; }
    const char* mode_name() const {
        switch (mode) {
            case ArmControlMode::Position: return "POSITION";
            case ArmControlMode::Velocity: return "VELOCITY";
            default: return "CARTESIAN";
        }
    }
    void clear_motion(const char* event) {
        sim.stop();
        position_target = sim.joint_positions();
        moving = false;
        blocked_until_release = true;
        trace(event);
    }
    void reset_camera() {
        camera.type = mjCAMERA_FREE;
        frame_model(sim.model(),sim.data(),camera);
        camera.azimuth = 135;
        camera.elevation = -25;
    }
    static void key_callback(GLFWwindow* w, int key, int, int action, int) {
        auto& v = get(w);
        const bool motion_key = key == GLFW_KEY_W || key == GLFW_KEY_S ||
            key == GLFW_KEY_A || key == GLFW_KEY_D || key == GLFW_KEY_Q || key == GLFW_KEY_E ||
            key == GLFW_KEY_UP || key == GLFW_KEY_DOWN || key == GLFW_KEY_LEFT || key == GLFW_KEY_RIGHT ||
            key == GLFW_KEY_EQUAL || key == GLFW_KEY_MINUS || key == GLFW_KEY_KP_ADD || key == GLFW_KEY_KP_SUBTRACT;
        if (motion_key && (action == GLFW_PRESS || action == GLFW_RELEASE))
            v.trace(action == GLFW_PRESS ? "key_press" : "key_release", key);
        if (action != GLFW_PRESS) return;
        switch (key) {
            case GLFW_KEY_ESCAPE:
                v.clear_motion("exit");
                glfwSetWindowShouldClose(w, 1);
                break;
            case GLFW_KEY_SPACE:
                v.paused = !v.paused;
                v.clear_motion(v.paused ? "pause" : "resume");
                break;
            case GLFW_KEY_R:
                v.sim.reset();
                v.clear_motion("reset");
                v.reset_camera();
                break;
            case GLFW_KEY_LEFT_BRACKET:
                v.selected = (v.selected + v.sim.joint_names().size() - 1) % v.sim.joint_names().size();
                v.clear_motion("select");
                break;
            case GLFW_KEY_RIGHT_BRACKET:
                v.selected = (v.selected + 1) % v.sim.joint_names().size();
                v.clear_motion("select");
                break;
            case GLFW_KEY_M:
                v.mode = v.mode == ArmControlMode::Cartesian ? ArmControlMode::Position
                    : v.mode == ArmControlMode::Position ? ArmControlMode::Velocity : ArmControlMode::Cartesian;
                v.clear_motion("mode");
                break;
            default:
                if (key >= GLFW_KEY_1 && key <= GLFW_KEY_9 &&
                    static_cast<std::size_t>(key - GLFW_KEY_1) < v.sim.joint_names().size()) {
                    v.selected = static_cast<std::size_t>(key - GLFW_KEY_1);
                    v.clear_motion("select");
                }
                break;
        }
    }
    static void mouse_callback(GLFWwindow* w, double x, double y) {
        auto& v = get(w);
        const double dx = x - v.mouse_x, dy = y - v.mouse_y;
        v.mouse_x = x;
        v.mouse_y = y;
        const bool left = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        const bool right = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        const bool middle = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
        if (!left && !right && !middle) return;
        int width, height;
        glfwGetWindowSize(w, &width, &height);
        if (height <= 0) return;
        const bool shift = v.down(GLFW_KEY_LEFT_SHIFT) || v.down(GLFW_KEY_RIGHT_SHIFT);
        const int action = right ? (shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V)
            : left ? (shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V) : mjMOUSE_ZOOM;
        mjv_moveCamera(v.sim.model(), action, dx / height, dy / height, &v.scene, &v.camera);
    }
    ArmViewOptions options;
    bool interactive;
    ArmControlMode mode = ArmControlMode::Cartesian;
    std::vector<double> position_target;
    bool moving = false, blocked_until_release = false;
    std::size_t selected = 0;
    double mouse_x = 0, mouse_y = 0;
    mjvCamera camera{};
    mjvOption option{};
    mjvScene scene{};
    mjrContext context{};
};
}  // namespace

void save_arm_snapshot(ArmSimulation& sim, const std::string& path, const ArmViewOptions& options) {
    ArmViewer viewer(sim, false, options);
    viewer.render();
    viewer.png(path);
}

void run_arm_viewer(ArmSimulation& sim, double duration, const std::string& screenshot,
                    const ArmViewOptions& options) {
    if (!std::isfinite(duration) || duration < 0)
        throw std::invalid_argument("Viewer duration must be finite and nonnegative");
    ArmViewer viewer(sim, true, options);
    using Clock = std::chrono::steady_clock;
    auto previous = Clock::now();
    const double dt = sim.model()->opt.timestep;
    if (!std::isfinite(dt) || dt <= 0) throw std::runtime_error("Invalid simulation timestep");
    double accumulator = 0, elapsed_simulation = 0;
    while (!glfwWindowShouldClose(viewer.window) && (duration == 0 || elapsed_simulation < duration)) {
        const auto frame_start = Clock::now();
        const double elapsed = std::chrono::duration<double>(frame_start - previous).count();
        previous = frame_start;
        glfwPollEvents();
        if (glfwWindowShouldClose(viewer.window)) break;
        // Bound catch-up after a slow render or window drag; never jump the arm.
        accumulator = viewer.paused ? 0 : accumulator + std::min(elapsed, .10);
        viewer.manual_step(0);  // Observe key releases even while no physics step is due.
        while (!viewer.paused && accumulator >= dt && (duration == 0 || elapsed_simulation < duration)) {
            viewer.manual_step(dt);
            sim.step();
            if (!sim.finite()) throw std::runtime_error("Arm simulation became non-finite");
            accumulator -= dt;
            elapsed_simulation += dt;
        }
        viewer.render();
        glfwSwapBuffers(viewer.window);
        std::this_thread::sleep_until(frame_start + std::chrono::milliseconds(16));
    }
    sim.stop();
    viewer.trace("finished");
    if (!screenshot.empty()) {
        viewer.render();
        viewer.png(screenshot);
    }
}
}  // namespace tech_core
