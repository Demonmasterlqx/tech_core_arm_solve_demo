#include "tech_core/robot_model.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <set>
#include <sstream>
#include <stdexcept>

namespace tech_core {
std::string robot_text(const mjModel* m, const std::string& name, bool required) {
    const int id = mj_name2id(m, mjOBJ_TEXT, name.c_str());
    if (id < 0) {
        if (required) throw std::invalid_argument("Missing robot metadata: " + name);
        return {};
    }
    return m->text_data + m->text_adr[id];
}
std::vector<std::string> robot_words(const mjModel* m, const std::string& name, bool required) {
    std::istringstream stream(robot_text(m, name, required));
    std::vector<std::string> result;
    for (std::string word; stream >> word;) result.push_back(word);
    return result;
}
std::vector<double> robot_numbers(const mjModel* m, const std::string& name, bool required) {
    const int id = mj_name2id(m, mjOBJ_NUMERIC, name.c_str());
    if (id < 0) {
        if (required) throw std::invalid_argument("Missing robot numeric: " + name);
        return {};
    }
    const auto* begin = m->numeric_data + m->numeric_adr[id];
    std::vector<double> result(begin, begin + m->numeric_size[id]);
    for (double value : result)
        if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite robot numeric: " + name);
    return result;
}
RobotModel RobotModel::read(const mjModel* m) {
    RobotModel r;
    r.id = robot_text(m, "robot_id");
    r.root_body = robot_text(m, "robot_root");
    r.tool_site = robot_text(m, "robot_tool");
    r.joint_names = robot_words(m, "robot_joints");
    r.velocity_limits = robot_numbers(m, "robot_velocity_limits");
    r.planning_home = robot_numbers(m, "robot_planning_home");
    const auto preset = robot_numbers(m, "robot_demo_start_at_preset", false);
    if (!preset.empty()) {
        if (preset.size() != 1 || (preset[0] != 0 && preset[0] != 1))
            throw std::invalid_argument("Invalid demo start pose flag");
        r.demo_start_at_preset = preset[0] != 0;
    }
    const auto recovery = robot_numbers(m, "robot_recovery_joint_order", false);
    std::set<int> recovery_seen;
    for (double value : recovery) {
        if (value < 0 || value >= double(r.joint_names.size()) || std::floor(value) != value ||
            !recovery_seen.insert(static_cast<int>(value)).second) throw std::invalid_argument("Invalid recovery joint order");
        r.recovery_joint_order.push_back(static_cast<int>(value));
    }
    for (int i = static_cast<int>(r.joint_names.size())-1; i >= 0; --i)
        if (!recovery_seen.count(i)) r.recovery_joint_order.push_back(i);
    const auto posture = robot_numbers(m, "robot_posture_joint", false);
    if (!posture.empty()) {
        if (posture.size() != 1 || posture[0] < -1 || posture[0] >= double(r.joint_names.size()) ||
            std::floor(posture[0]) != posture[0]) throw std::invalid_argument("Invalid robot posture joint");
        r.posture_joint = static_cast<int>(posture[0]);
    }
    if (r.joint_names.empty() || r.velocity_limits.size() != r.joint_names.size() ||
        r.planning_home.size() != r.joint_names.size()) throw std::invalid_argument("Robot metadata dimension mismatch");
    const int root = mj_name2id(m, mjOBJ_BODY, r.root_body.c_str());
    const int tool = mj_name2id(m, mjOBJ_SITE, r.tool_site.c_str());
    if (root <= 0 || tool < 0) throw std::invalid_argument("Missing robot root body or tool site");
    auto descendant = [&](int body) {
        for (; body > 0; body = m->body_parentid[body]) if (body == root) return true;
        return false;
    };
    for(int b=m->body_parentid[root];b>0;b=m->body_parentid[b])
        if(m->body_jntnum[b]) throw std::invalid_argument("Robot root must have a fixed ancestor chain");
    if (!descendant(m->site_bodyid[tool])) throw std::invalid_argument("Robot tool is outside robot subtree");
    std::set<int> seen;
    for (std::size_t i = 0; i < r.joint_names.size(); ++i) {
        const int j = mj_name2id(m, mjOBJ_JOINT, r.joint_names[i].c_str());
        if (j < 0 || !seen.insert(j).second || !descendant(m->jnt_bodyid[j]) ||
            (m->jnt_type[j] != mjJNT_HINGE && m->jnt_type[j] != mjJNT_SLIDE) || !m->jnt_limited[j])
            throw std::invalid_argument("Robot requires unique, limited scalar joints in its subtree: " + r.joint_names[i]);
        const double lo = m->jnt_range[2*j], hi = m->jnt_range[2*j+1];
        if (!std::isfinite(lo) || !std::isfinite(hi) || lo >= hi || r.velocity_limits[i] <= 0 ||
            r.planning_home[i] < lo || r.planning_home[i] > hi)
            throw std::invalid_argument("Invalid robot limits or planning home: " + r.joint_names[i]);
        r.joints.push_back(j);
    }
    for (int j = 0; j < m->njnt; ++j)
        if (descendant(m->jnt_bodyid[j]) && !seen.count(j))
            throw std::invalid_argument("Undeclared moving joint in robot subtree");
    return r;
}
std::string robot_model_path(const std::string& robot, bool planning) {
    namespace fs = std::filesystem;
    fs::path directory;
    if (robot == "rm26") directory = fs::path(TECH_CORE_MODELS_DIR) / "rm26_arm";
    else if (fs::is_directory(robot)) directory = fs::absolute(robot);
    else directory = fs::path(TECH_CORE_MODELS_DIR) / robot;
    const auto path = robot == "rm26" && planning ? fs::path(TECH_CORE_MODELS_DIR) / "planning/scene.xml"
        : directory / (planning ? "planning.xml" : "scene.xml");
    if (!fs::is_regular_file(path)) throw std::invalid_argument("Robot model not found: " + path.string());
    return path.string();
}
std::vector<std::string> available_robots() {
    std::vector<std::string> result{"rm26"};
    for (const auto& entry : std::filesystem::directory_iterator(TECH_CORE_MODELS_DIR))
        if (entry.is_directory() && entry.path().filename() != "planning" && entry.path().filename() != "rm26_arm" &&
            std::filesystem::is_regular_file(entry.path() / "scene.xml")) result.push_back(entry.path().filename().string());
    std::sort(result.begin(), result.end());
    return result;
}
} // namespace tech_core
