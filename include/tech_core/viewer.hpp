#pragma once
#include "tech_core/simulation.hpp"
#include <string>
namespace tech_core {
struct ViewOptions {
    std::string view = "iso";
    bool overlay = true;
    bool hide_unit = false;
    // Zero selects the preset distance; positive values are metres.
    double distance = 0;
};
void run_viewer(Simulation& sim, bool demo, double duration, const std::string& screenshot,
                const ViewOptions& options = {});
void save_snapshot(Simulation& sim, const std::string& path, const ViewOptions& options = {});
}
