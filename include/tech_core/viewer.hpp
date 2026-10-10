#pragma once
#include "tech_core/simulation.hpp"
#include <string>
namespace tech_core {
void run_viewer(Simulation& sim, bool demo, double duration, const std::string& screenshot);
void save_snapshot(Simulation& sim, const std::string& path);
}
