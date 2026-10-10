#include "tech_core/simulation.hpp"
#include "tech_core/viewer.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>

namespace {
void usage() {
    std::cout << "Technology core | MuJoCo + C++\n"
      "  --headless                 run CPU physics without a window\n"
      "  --demo                     virtual gripper performs assembly\n"
      "  --level 1|2|3|4            selected training difficulty (default 3)\n"
      "  --target DEG --target2 DEG Q targets in [-90,90] (default 35,-40)\n"
      "  --pose X,Y,Z,THETA,PHI,ALPHA  metres/degrees, table 5-10\n"
      "  --secondary-orientation THETA,PHI,ALPHA (default 30,90,0)\n"
      "  --seed N                   reproducible translation and Q targets\n"
      "  --duration SEC             finite simulation time (headless default 15)\n"
      "  --csv PATH                 measured trajectory (headless, 50 Hz)\n"
      "  --screenshot PATH.png      capture final state (needs OpenGL context)\n"
      "  --view PRESET              iso|front|rear|side|top|marker|unit (default iso)\n"
      "  --distance METRES          positive camera distance; default by preset\n"
      "  --no-overlay               hide viewer text in windows and screenshots\n"
      "  --hide-unit                inspection: hide energy-unit visuals only\n"
      "  --model PATH               override MJCF model\n"
      "Keys: Tab select core; I/K insert/withdraw; U/J lift/lower;\n"
      "      O/L flip up/return; A/D twist; Enter confirm; Space pause;\n"
      "      G demo; C card present; X release gripper; R reset; Esc exit.\n"
      "Training mode selects a difficulty directly; match unlock/economy\n"
      "and free robot grasping are outside this executable.\n";
}
double number(const std::string& s) {
    std::size_t used=0; double value=std::stod(s,&used);
    if(used!=s.size() || !std::isfinite(value)) throw std::invalid_argument("Invalid number: "+s);
    return value;
}
void prepare_parent(const std::string& p) {
    const auto parent=std::filesystem::path(p).parent_path();
    if(!parent.empty()) std::filesystem::create_directories(parent);
}
}

int main(int argc,char** argv) {
    try {
        tech_core::SimulationConfig config;
        tech_core::ViewOptions view_options;
        bool headless=false,demo=false;
        double duration=0;
        std::string csv_path,screenshot;
        for(int i=1;i<argc;++i) {
            const std::string arg=argv[i];
            auto value=[&]() -> std::string {
                if(++i>=argc) throw std::invalid_argument("Missing value for "+arg);
                return argv[i];
            };
            if(arg=="--help" || arg=="-h") {usage();return 0;}
            else if(arg=="--headless") headless=true;
            else if(arg=="--demo") demo=true;
            else if(arg=="--level") {
                const double level=number(value());
                if(level<1 || level>4 || std::floor(level)!=level) throw std::invalid_argument("Level must be 1..4");
                config.level=static_cast<int>(level);
            }
            else if(arg=="--target") config.target_degrees[0]=number(value());
            else if(arg=="--target2") config.target_degrees[1]=number(value());
            else if(arg=="--duration") {
                duration=number(value());
                if(duration<=0 || duration>3600) throw std::invalid_argument("Duration must be in (0,3600] seconds");
            }
            else if(arg=="--model") config.model_path=value();
            else if(arg=="--csv") csv_path=value();
            else if(arg=="--screenshot") screenshot=value();
            else if(arg=="--no-overlay") view_options.overlay=false;
            else if(arg=="--hide-unit") view_options.hide_unit=true;
            else if(arg=="--view") {
                view_options.view=value();
                const std::array<std::string,7> presets{"iso","front","rear","side","top","marker","unit"};
                if(std::find(presets.begin(),presets.end(),view_options.view)==presets.end())
                    throw std::invalid_argument("View must be iso, front, rear, side, top, marker, or unit");
            }
            else if(arg=="--distance") {
                view_options.distance=number(value());
                if(view_options.distance<=0) throw std::invalid_argument("Camera distance must be positive metres");
            }
            else if(arg=="--seed") {
                const double seed=number(value());
                if(seed<0 || seed>4294967295.0 || std::floor(seed)!=seed) throw std::invalid_argument("Seed must be uint32");
                std::mt19937 rng(static_cast<unsigned>(seed));
                config.pose.x=std::uniform_real_distribution<double>(-.1,-.06)(rng);
                config.pose.y=std::uniform_real_distribution<double>(.1,.3)(rng);
                config.pose.z=std::uniform_real_distribution<double>(.5,.7)(rng);
                for(auto& t:config.target_degrees) t=5*std::uniform_int_distribution<int>(-18,18)(rng);
            }
            else if(arg=="--secondary-orientation") {
                std::string text=value();std::replace(text.begin(),text.end(),',',' ');
                std::istringstream in(text);auto& a=config.secondary_angles;
                if(!(in>>a[0]>>a[1]>>a[2])) throw std::invalid_argument("Secondary orientation requires three numbers");
                std::string extra;if(in>>extra) throw std::invalid_argument("Secondary orientation has extra values");
            }
            else if(arg=="--pose") {
                std::string text=value();std::replace(text.begin(),text.end(),',',' ');
                std::istringstream in(text); auto& p=config.pose;
                if(!(in>>p.x>>p.y>>p.z>>p.theta>>p.phi>>p.alpha)) throw std::invalid_argument("Pose requires six numbers");
                std::string extra;if(in>>extra) throw std::invalid_argument("Pose has extra values");
            }
            else throw std::invalid_argument("Unknown option: "+arg);
        }
        if(!headless && !csv_path.empty()) throw std::invalid_argument("--csv requires --headless");
        if(headless && duration==0) duration=15;
        tech_core::Simulation sim(config);
        std::cout<<"MuJoCo "<<mj_versionString()<<" | CPU physics | level "<<config.level
                 <<" | dt "<<sim.model()->opt.timestep<<" s | Q targets "<<config.target_degrees[0]<<", "<<config.target_degrees[1]<<" deg\n";
        if(headless) {
            std::ofstream csv;
            if(!csv_path.empty()) {
                prepare_parent(csv_path);csv.open(csv_path);
                if(!csv) throw std::runtime_error("Cannot write CSV: "+csv_path);
                csv<<"time_s,core,stage,insertion_m,lift_m,P_rad,Q_rad,Q_target_rad,safety_locked,return_torque_Nm,holding_torque_Nm,contacts\n";
                csv<<std::setprecision(10);
            }
            std::array<tech_core::Stage,2> previous{tech_core::Stage::Moving,tech_core::Stage::Moving};
            double next_csv=0;
            while(sim.time()+sim.model()->opt.timestep*.5<duration) {
                if(demo) sim.drive_demo();
                sim.step();
                for(std::size_t i=0;i<sim.rules().core_count();++i) {
                    const auto stage=sim.rules().stage(i);
                    if(stage!=previous[i]) {
                        std::cout<<std::fixed<<std::setprecision(3)<<sim.time()<<"s core"<<i<<" "<<tech_core::stage_name(stage)<<'\n';
                        previous[i]=stage;
                    }
                }
                if(csv && sim.time()+1e-9>=next_csv) {
                    for(std::size_t i=0;i<sim.rules().core_count();++i) {
                        auto o=sim.observation(i);
                        csv<<sim.time()<<','<<i<<','<<tech_core::stage_name(sim.rules().stage(i))<<','
                           <<o.insertion<<','<<o.lift<<','<<o.flip<<','<<o.twist<<','<<sim.rules().target(i)<<','
                           <<sim.rules().safety_locked(i)<<','<<sim.return_torque(i)<<','<<sim.holding_torque(i)<<','<<sim.data()->ncon<<'\n';
                    }
                    next_csv+=.02;
                }
                if(sim.rules().failed()) break;
            }
            if(csv.is_open()) { csv.flush(); if(!csv) throw std::runtime_error("CSV write failed"); }
            if(!screenshot.empty()) {
#ifdef TECH_CORE_VIEWER
                prepare_parent(screenshot);tech_core::save_snapshot(sim,screenshot,view_options);
#else
                throw std::runtime_error("Screenshot requires -DTECH_CORE_VIEWER=ON");
#endif
            }
        } else {
#ifdef TECH_CORE_VIEWER
            if(!screenshot.empty()) prepare_parent(screenshot);
            tech_core::run_viewer(sim,demo,duration,screenshot,view_options);
#else
            throw std::runtime_error("Viewer disabled at build; use --headless");
#endif
        }
        std::cout<<"Result: "<<(sim.rules().successful()?"SUCCESS":sim.rules().failed()?"FAILED":"INCOMPLETE")
                 <<" at "<<sim.time()<<" s";
        if(sim.rules().failed()) std::cout<<" ("<<sim.rules().failure_reason()<<")";
        std::cout<<'\n';
        return headless && !sim.rules().successful()?2:0;
    } catch(const std::exception& e) {std::cerr<<"Error: "<<e.what()<<'\n';return 1;}
}
