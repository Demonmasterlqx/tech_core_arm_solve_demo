#include "tech_core/planning.hpp"

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

// A complete CPU-only library call. It plans and revalidates with caches disabled;
// it does not execute the returned trajectory or create a rendering context.
int main(int argc, char** argv) {
    using namespace tech_core::planning;
    try {
        Method backend = Method::RigidCache;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--help") {
                std::cout << "rm26_plan_example [--backend baseline|rigid_cache]\n"
                             "Plan from seven-joint zero to nominal assembly; no execution.\n";
                return 0;
            }
            if (arg != "--backend" || ++i == argc)
                throw std::invalid_argument("Use --backend baseline|rigid_cache or --help");
            backend = parse_method(argv[i]);
            if (backend != Method::Baseline && backend != Method::RigidCache)
                throw std::invalid_argument("This example uses baseline or rigid_cache");
        }

        Scenario request = scenario_suite().front();
        request.start.setZero();                 // Replace with actual measured joint angles.
        request.target_degrees = -90;
        request.q_sweep = true;
        request.q_best_effort = true;
        request.enforce_core_pose_limits = false; // Same freely edited pose contract as the Demo.

        PlanOptions options;
        options.method = backend;
        options.timeout_seconds = 5;
        options.motion_speed_scale = 4;
        PlanningScene scene;                     // One independent workspace per worker.
        const PlanResult result = plan(scene, request, options);
        std::cout << "backend=" << method_name(backend)
                  << " success=" << result.success << " reason=" << result.reason
                  << " knots=" << result.knots.size()
                  << " planning_seconds=" << result.planning_seconds << '\n';
        if (!result.success) {
            // Failed candidates may be shown, but must never be executed.
            std::cerr << "Planning failed; returned knots are preview-only.\n";
            return 2;
        }

        scene.configure(request); // Clears and disables both collision caches.
        const ValidationResult audit = validate_trajectory(scene, result.knots, options);
        std::cout << "uncached_valid=" << audit.valid << " reason=" << audit.reason
                  << " Q=0 -> +" << result.q_positive_degrees
                  << " -> 0 -> " << result.q_final_degrees << " deg\n";
        if (!audit.valid) return 2;
        // result.knots now contains q, task/mechanism state and timestamps.
        // An application must explicitly choose when to execute these knots.
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
