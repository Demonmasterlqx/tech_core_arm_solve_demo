#include "tech_core/arm_simulation.hpp"
#ifdef TECH_CORE_VIEWER
#include "tech_core/arm_viewer.hpp"
#endif
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
double number(const std::string& text) {
    std::size_t used=0;
    const double value=std::stod(text,&used);
    if(used!=text.size() || !std::isfinite(value))
        throw std::invalid_argument("Expected a finite number: "+text);
    return value;
}
std::vector<double> numbers(const std::string& text) {
    std::vector<double> result;
    std::istringstream input(text);
    std::string token;
    if(text.empty() || text.back()==',') throw std::invalid_argument("Empty command value");
    while(std::getline(input,token,',')) result.push_back(number(token));
    return result;
}
void parent_directory(const std::string& path) {
    const auto parent=std::filesystem::path(path).parent_path();
    if(!parent.empty()) std::filesystem::create_directories(parent);
}
void usage() {
    std::cout<<"Robot arm | MuJoCo kinematics + ideal position/velocity drive\n"
      "  --headless              CPU simulation without a window\n"
      "  --duration SEC          finite run, (0,3600]; headless default 2\n"
      "  --position Q0,...,QN    joint position (rad or m, per joint type)\n"
      "  --velocity V0,...,VN    joint velocity (rad/s or m/s)\n"
      "  --cartesian VX,VY,VZ    world-frame tool velocity (headless, m/s)\n"
      "  --demo                  six-direction translation (headless)\n"
      "  --speed M/S             keyboard/demo speed, (0,0.1], default 0.10\n"
      "  --csv PATH              measured states at 50 Hz (headless)\n"
      "  --screenshot PATH.png   final image; needs OpenGL / software runner\n"
      "  --no-overlay            hide on-screen text\n"
      "  --robot ID|DIRECTORY    select robot package (default rm26)\n"
      "  --list-robots           list installed robots\n"
      "  --model PATH            override MJCF model\n"
      "Keys: W/Up +X, S/Down -X, A/Left +Y, D/Right -Y, Q +Z, E -Z.\n"
      "M: Cartesian/position/velocity; 1-9 / [ ]: joint; +/-: jog joint.\n"
      "Space: pause; R: home; Esc: exit. Mouse: orbit/pan/zoom.\n"
      "Fixed base; torque layer mocked. No contact-force response or grasping.\n";
}
void print_state(const char* label,const tech_core::ArmSimulation& sim) {
    const auto tool=sim.tool_position();
    const auto q=sim.joint_positions(),v=sim.joint_velocities();
    std::cout<<std::setprecision(12)<<"State "<<label<<": {\"time\":"<<sim.time()<<",\"tool\":[";
    for(std::size_t i=0;i<tool.size();++i) std::cout<<(i?",":"")<<tool[i];
    std::cout<<"],\"q\":[";
    for(std::size_t i=0;i<q.size();++i) std::cout<<(i?",":"")<<q[i];
    std::cout<<"],\"qd\":[";
    for(std::size_t i=0;i<v.size();++i) std::cout<<(i?",":"")<<v[i];
    std::cout<<"]}\n"<<std::flush;
}
void write_state(std::ostream& csv,const tech_core::ArmSimulation& sim) {
    const auto tool=sim.tool_position();
    const char* mode=sim.mode()==tech_core::ArmControlMode::Position?"position":
                     sim.mode()==tech_core::ArmControlMode::Velocity?"velocity":"cartesian";
    csv<<sim.time()<<','<<mode;
    for(double value:tool) csv<<','<<value;
    for(double value:sim.joint_positions()) csv<<','<<value;
    for(double value:sim.joint_velocities()) csv<<','<<value;
    csv<<'\n';
}
}

int main(int argc,char** argv) {
    try {
        bool headless=false,demo=false;
        [[maybe_unused]] bool overlay=true;
        double duration=0,speed=.10;
        std::string model,robot="rm26",csv_path,screenshot,command;
        bool robot_selected=false;
        std::vector<double> values;
        for(int i=1;i<argc;++i) {
            const std::string arg=argv[i];
            auto value=[&]() -> std::string {
                if(++i>=argc) throw std::invalid_argument("Missing value for "+arg);
                return argv[i];
            };
            if(arg=="--help" || arg=="-h") {usage();return 0;}
            else if(arg=="--headless") headless=true;
            else if(arg=="--duration") {
                duration=number(value());
                if(duration<=0 || duration>3600) throw std::invalid_argument("Duration must be in (0,3600]");
            } else if(arg=="--speed") {
                speed=number(value());
                if(speed<=0 || speed>.1) throw std::invalid_argument("Speed must be in (0,0.1] m/s");
            } else if(arg=="--robot") {robot=value();robot_selected=true;}
            else if(arg=="--list-robots") {for(const auto& id:tech_core::available_robots()) std::cout<<id<<'\n';return 0;}
            else if(arg=="--model") model=value();
            else if(arg=="--csv") csv_path=value();
            else if(arg=="--screenshot") screenshot=value();
            else if(arg=="--no-overlay") overlay=false;
            else if(arg=="--demo" || arg=="--position" || arg=="--velocity" || arg=="--cartesian") {
                if(!command.empty()) throw std::invalid_argument("Select only one motion command");
                command=arg;
                if(arg=="--demo") demo=true;
                else values=numbers(value());
            } else throw std::invalid_argument("Unknown option: "+arg);
        }
        if(!headless && (!command.empty() || !csv_path.empty()))
            throw std::invalid_argument("Motion command options and --csv require --headless; use keyboard in the viewer");
#ifndef TECH_CORE_VIEWER
        if(!headless || !screenshot.empty()) throw std::invalid_argument("This build requires --headless without --screenshot");
#endif
        if(headless && duration==0) duration=2;
        if(robot_selected && !model.empty()) throw std::invalid_argument("Use --robot or --model, not both");
        if(model.empty()) model=tech_core::robot_model_path(robot);
        tech_core::ArmSimulation sim(model);
        if(command=="--position") sim.set_joint_positions(values);
        else if(command=="--velocity") sim.set_joint_velocities(values);
        else if(command=="--cartesian") {
            if(values.size()!=3) throw std::invalid_argument("Cartesian velocity requires exactly three values");
            sim.set_cartesian_velocity({values[0],values[1],values[2]});
        }
        std::cout<<"MuJoCo "<<mj_versionString()<<" | CPU kinematics | ideal drive (torque mocked) | "
                 <<sim.joint_names().size()<<" arm joints | dt "<<sim.model()->opt.timestep<<" s\n";
        std::cout<<"Joint order:";
        for(const auto& name:sim.joint_names()) std::cout<<' '<<name;
        std::cout<<'\n';
        print_state("initial",sim);
        if(!screenshot.empty()) parent_directory(screenshot);
        if(headless) {
            std::ofstream csv;
            if(!csv_path.empty()) {
                parent_directory(csv_path);csv.open(csv_path);
                if(!csv) throw std::runtime_error("Cannot write CSV: "+csv_path);
                csv<<std::setprecision(12)<<"time_s,mode,tool_x_m,tool_y_m,tool_z_m";
                for(std::size_t i=0;i<sim.joint_names().size();++i)
                    csv<<','<<sim.joint_names()[i]<<(sim.model()->jnt_type[sim.robot().joints[i]]==mjJNT_SLIDE?"_m":"_rad");
                for(std::size_t i=0;i<sim.joint_names().size();++i)
                    csv<<','<<sim.joint_names()[i]<<(sim.model()->jnt_type[sim.robot().joints[i]]==mjJNT_SLIDE?"_m_s":"_rad_s");
                csv<<'\n';write_state(csv,sim);
            }
            double next_csv=.02,last_csv=0;
            while(sim.time()+.5*sim.model()->opt.timestep<duration) {
                if(demo) {
                    const int segment=std::min(5,static_cast<int>(6*sim.time()/duration));
                    std::array<double,3> velocity{};
                    velocity[segment/2]=segment%2?-speed:speed;
                    sim.set_cartesian_velocity(velocity);
                }
                sim.step();
                if(!sim.finite()) throw std::runtime_error("Nonfinite arm state");
                if(csv.is_open() && sim.time()+1e-9>=next_csv) {
                    write_state(csv,sim);last_csv=sim.time();next_csv+=.02;
                }
            }
            if(csv.is_open()) {
                if(sim.time()>last_csv+1e-9) write_state(csv,sim);
                csv.flush();if(!csv) throw std::runtime_error("CSV write failed");
            }
#ifdef TECH_CORE_VIEWER
            if(!screenshot.empty()) tech_core::save_arm_snapshot(sim,screenshot,{speed,overlay});
#endif
        } else {
#ifdef TECH_CORE_VIEWER
            tech_core::run_arm_viewer(sim,duration,screenshot,{speed,overlay});
#endif
        }
        print_state("final",sim);
        return sim.finite()?0:2;
    } catch(const std::exception& error) {
        std::cerr<<"Error: "<<error.what()<<'\n';return 1;
    }
}
