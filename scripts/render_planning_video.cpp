#include "tech_core/planning.hpp"
#include <GL/osmesa.h>
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace tech_core;
using namespace tech_core::planning;
using Row = std::map<std::string, std::string>;

std::vector<std::string> fields(const std::string& line) {
    std::vector<std::string> out; std::string field; bool quoted = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (c == '"') {
            if (quoted && i + 1 < line.size() && line[i + 1] == '"') { field += c; ++i; }
            else quoted = !quoted;
        } else if (c == ',' && !quoted) { out.push_back(field); field.clear(); }
        else if (c != '\r') field += c;
    }
    if (quoted) throw std::runtime_error("Unterminated CSV field");
    out.push_back(field); return out;
}
std::vector<Row> read_csv(const std::filesystem::path& path) {
    std::ifstream in(path); if (!in) throw std::runtime_error("Cannot open " + path.string());
    std::string line; std::getline(in, line); const auto header = fields(line);
    std::vector<Row> rows;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        auto values = fields(line); if (values.size() != header.size()) throw std::runtime_error("Invalid CSV row");
        Row row; for (std::size_t i = 0; i < values.size(); ++i) row[header[i]] = values[i];
        rows.push_back(std::move(row));
    }
    return rows;
}
double number(const Row& r, const std::string& key) { return std::stod(r.at(key)); }
Phase phase(const std::string& name) {
    for (int i = 0; i < 6; ++i) if (name == phase_name(static_cast<Phase>(i))) return static_cast<Phase>(i);
    throw std::runtime_error("Invalid phase");
}

int main(int argc, char** argv) {
    try {
        if (argc != 10) throw std::runtime_error("Usage: renderer BENCHMARK METHOD CASE REPEAT WIDTH HEIGHT FPS SPEED STILL_TIME_OR_MINUS_ONE");
        const std::filesystem::path directory = argv[1];
        const std::string method = argv[2], case_id = argv[3], repeat = argv[4];
        const int width = std::stoi(argv[5]), height = std::stoi(argv[6]), fps = std::stoi(argv[7]);
        const double speed = std::stod(argv[8]), still = std::stod(argv[9]);
        if (width < 960 || height < 600 || fps < 1 || !std::isfinite(speed) || speed <= 0)
            throw std::runtime_error("Invalid render settings");
        Row run;
        for (const auto& r : read_csv(directory / "runs.csv"))
            if (r.at("method") == method && r.at("case") == case_id && r.at("repeat") == repeat) run = r;
        if (run.empty() || run.at("success") != "1" || run.at("dense_recheck_valid") != "1")
            throw std::runtime_error("Replay requires a successful, dense-checked trajectory");
        Scenario scenario; bool found = false;
        for (const auto& r : read_csv(directory / "scenarios.csv")) if (r.at("case") == case_id) {
            found = true; scenario.id = case_id; scenario.split = r.at("split");
            scenario.core = {number(r,"core_x"),number(r,"core_y"),number(r,"core_z"),
                number(r,"core_theta_degrees"),number(r,"core_phi_degrees"),number(r,"core_alpha_degrees")};
            scenario.target_degrees = number(r,"target_degrees");
            for (int j = 0; j < 7; ++j) scenario.start[j] = number(r,"start_q" + std::to_string(j));
            scenario.assembly_translation = {number(r,"assembly_translation_x"),number(r,"assembly_translation_y"),number(r,"assembly_translation_z")};
            scenario.assembly_yaw = number(r,"assembly_yaw_radians");
            scenario.arm_base_z = r.count("arm_base_z") ? number(r,"arm_base_z") : 0;
            scenario.q_sweep = r.count("q_sweep") && r.at("q_sweep") == "1";
            scenario.q_best_effort = r.count("q_best_effort") && r.at("q_best_effort") == "1";
            scenario.blocking_obstacle = r.at("blocking_obstacle") == "1";
        }
        if (!found) throw std::runtime_error("Scenario missing");
        std::vector<Knot> knots;
        for (const auto& r : read_csv(directory / run.at("trajectory"))) {
            Knot k; k.time = number(r,"time"); k.phase = phase(r.at("phase"));
            k.mechanism = {number(r,"insertion"),number(r,"lift"),number(r,"flip"),number(r,"twist")};
            for (int j = 0; j < 7; ++j) k.q[j] = number(r,"q" + std::to_string(j));
            k.target.position = {number(r,"target_x"),number(r,"target_y"),number(r,"target_z")};
            k.target.direction = {number(r,"direction_x"),number(r,"direction_y"),number(r,"direction_z")};
            knots.push_back(k);
        }
        PlanningScene sim; sim.configure(scenario);
        PlanOptions check; check.validation_joint_step /= 2; check.validation_min_substeps *= 2;
        check.motion_speed_scale = run.count("motion_speed_scale") ? number(run,"motion_speed_scale") : 1;
        const auto validation = validate_trajectory(sim, knots, check);
        if (!validation.valid) throw std::runtime_error("Current-model recheck failed: " + validation.reason);
        std::cerr << "DENSE_RECHECK passed samples=" << validation.samples << '\n';
        sim.configure(scenario);
        OSMesaContext os = OSMesaCreateContextExt(OSMESA_RGBA, 24, 8, 0, nullptr);
        std::vector<unsigned char> framebuffer(std::size_t(width) * height * 4);
        if (!os || !OSMesaMakeCurrent(os, framebuffer.data(), GL_UNSIGNED_BYTE, width, height))
            throw std::runtime_error("OSMesa context failed");
        const std::string renderer = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
        std::cerr << "OPENGL_RENDERER " << renderer << '\n';
        if (renderer.find("llvmpipe") == std::string::npos && renderer.find("softpipe") == std::string::npos)
            throw std::runtime_error("Expected CPU software rendering");
        auto* model = sim.model(); model->vis.global.offwidth = width; model->vis.global.offheight = height;
        model->vis.quality.shadowsize = 1024; model->vis.quality.offsamples = 2;
        mjvOption option; mjv_defaultOption(&option);
        for (int i = 0; i < mjNGROUP; ++i) { option.geomgroup[i] = i <= 2; option.sitegroup[i] = 0; }
        mjvScene scene; mjv_defaultScene(&scene); mjv_makeScene(model, &scene, model->ngeom + 100);
        scene.flags[mjRND_REFLECTION] = 0;
        mjrContext context; mjr_defaultContext(&context); mjr_makeContext(model, &context, mjFONTSCALE_150);
        mjr_setBuffer(mjFB_OFFSCREEN, &context);
        mjvCamera wide, close; mjv_defaultCamera(&wide); mjv_defaultCamera(&close);
        const double mount_shift = scenario.assembly_translation.x() - .30;
        wide.lookat[0] = .22 + .5 * mount_shift; wide.lookat[1] = 0; wide.lookat[2] = .32 + .5 * scenario.arm_base_z;
        wide.distance = 1.45; wide.azimuth = 315; wide.elevation = -22;
        close.lookat[0] = .43 + mount_shift; close.lookat[1] = .015; close.lookat[2] = .37;
        close.distance = .82; close.azimuth = 310; close.elevation = -17;
        const int split = width * 55 / 100;
        const mjrRect full{0,0,width,height}, left{0,140,split-3,height-230}, right{split+3,140,width-split-3,height-230};
        const double duration = knots.back().time;
        const int frames = still >= 0 ? 1 : static_cast<int>(std::ceil((duration / speed + 2) * fps));
        const char* labels[] = {"APPROACH", "INSERT", "LIFT", "P FLIP", "Q ALIGN", "DWELL"};
        std::vector<unsigned char> rgb(std::size_t(width) * height * 3);
        std::size_t index = 1;
        for (int frame = 0; frame < frames; ++frame) {
            const double t = std::clamp(still >= 0 ? still : double(frame) / fps * speed, 0., duration);
            while (index + 1 < knots.size() && knots[index].time < t) ++index;
            const auto& a = knots[index-1]; const auto& b = knots[index];
            const double u = std::clamp((t-a.time)/(b.time-a.time),0.,1.), v = u*u*(3-2*u);
            const auto lerp = [v](double x,double y){return x+(y-x)*v;};
            MechanismState mechanism{lerp(a.mechanism.insertion,b.mechanism.insertion),lerp(a.mechanism.lift,b.mechanism.lift),
                lerp(a.mechanism.flip,b.mechanism.flip),lerp(a.mechanism.twist,b.mechanism.twist)};
            sim.set_state((1-v)*a.q+v*b.q, mechanism); sim.data()->time = t;
            mjr_rectangle(full,.055f,.075f,.10f,1);
            mjv_updateScene(model,sim.data(),&option,nullptr,&wide,mjCAT_ALL,&scene); mjr_render(left,&scene,&context);
            mjv_updateScene(model,sim.data(),&option,nullptr,&close,mjCAT_ALL,&scene); mjr_render(right,&scene,&context);
            std::ostringstream title, metric, status, values;
            title << "LEVEL 3 EXCHANGE  |  TRAJECTORY REPLAY\nBase Z "
                  << std::fixed << std::setprecision(0) << scenario.arm_base_z * 1000
                  << " mm | Pitch " << scenario.core.phi-90 << " deg | Roll " << scenario.core.alpha << " deg";
            const bool online=run.count("planning_mode") && run.at("planning_mode")=="online_fixed_start";
            metric << method << (online?"  |  Online planning ":"  |  Planning ") << std::fixed << std::setprecision(3) << number(run,"planning_seconds")
                   << " s\nMotion " << std::setprecision(0) << check.motion_speed_scale << "x | "
                   << std::setprecision(2) << duration << " s | Playback " << speed << "x";
            mjr_overlay(mjFONT_NORMAL,mjGRID_TOPLEFT,{0,height-86,width,86},title.str().c_str(),nullptr,&context);
            mjr_overlay(mjFONT_NORMAL,mjGRID_TOPRIGHT,{0,height-86,width,86},metric.str().c_str(),nullptr,&context);
            mjr_overlay(mjFONT_NORMAL,mjGRID_TOPLEFT,left,"FULL ARM",nullptr,&context);
            mjr_overlay(mjFONT_NORMAL,mjGRID_TOPLEFT,right,"GRIPPER / CORE",nullptr,&context);
            status << "t = " << std::fixed << std::setprecision(2) << t << " / " << duration << " s   |   "
                   << (t >= duration ? (scenario.q_best_effort && run.at("full_q_sweep")!="1" ?
                      "SAFE STOP - PARTIAL Q" : "FULL TASK COMPLETE") : labels[static_cast<int>(b.phase)]);
            values << "Insert " << std::setprecision(0) << std::fixed << mechanism.insertion*1000 << " mm    Lift " << mechanism.lift*1000
                   << " mm    P " << mechanism.flip*180/pi << " deg    Q " << mechanism.twist*180/pi << " / " << (scenario.q_best_effort?knots.back().mechanism.twist*180/pi:scenario.target_degrees) << " deg";
            if(scenario.q_best_effort) values << "   [0 > " << number(run,"q_positive_degrees") << " > " << number(run,"q_final_degrees") << "]";
            else if(scenario.q_sweep) values << "   [0 > +90 > -90]";
            mjr_overlay(mjFONT_NORMAL,mjGRID_TOPLEFT,{0,60,width,76},status.str().c_str(),nullptr,&context);
            mjr_overlay(mjFONT_NORMAL,mjGRID_BOTTOMLEFT,{0,60,width,76},values.str().c_str(),nullptr,&context);
            for (int i = 0; i < 6; ++i) {
                const bool reached = i <= static_cast<int>(b.phase);
                const mjrRect box{i*width/6+5,14,width/6-10,36};
                mjr_rectangle(box,reached?.10f:.13f,reached?.40f:.16f,reached?.35f:.20f,1);
                mjr_overlay(mjFONT_NORMAL,mjGRID_TOPLEFT,box,labels[i],nullptr,&context);
            }
            mjr_readPixels(rgb.data(),nullptr,full,&context);
            if (std::fwrite(rgb.data(),1,rgb.size(),stdout) != rgb.size()) throw std::runtime_error("Video pipe closed");
            if (frame % (fps*2) == 0 || frame+1 == frames)
                std::cerr << "FRAME " << frame+1 << '/' << frames << " trajectory_time=" << t << '\n';
        }
        std::fflush(stdout); mjr_freeContext(&context); mjv_freeScene(&scene); OSMesaDestroyContext(os);
        return 0;
    } catch (const std::exception& error) { std::cerr << "ERROR " << error.what() << '\n'; return 1; }
}
