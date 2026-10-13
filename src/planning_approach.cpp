#include "tech_core/planning.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace tech_core::planning {
namespace {
struct Node { JointVector q; int parent = -1; };
using Tree = std::vector<Node>;

std::vector<JointVector> branch(const Tree& tree, int index) {
    std::vector<JointVector> path;
    for (; index >= 0; index = tree[index].parent) path.push_back(tree[index].q);
    std::reverse(path.begin(), path.end());
    return path;
}
double length(const std::vector<JointVector>& path) {
    double total = 0;
    for (std::size_t i = 1; i < path.size(); ++i) total += (path[i]-path[i-1]).norm();
    return total;
}
}

ApproachResult optimize_approach(PlanningScene& scene, const JointVector& start,
                               const JointVector& goal, const PlanOptions& options,
                               const std::function<bool()>& should_stop) {
    ApproachResult out;
    const auto begin = std::chrono::steady_clock::now();
    auto stopped = [&] {
        return (should_stop && should_stop()) ||
            std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count() >= options.timeout_seconds;
    };
    if (start.size() != scene.dof() || goal.size() != scene.dof() || !start.allFinite() || !goal.allFinite() ||
        !std::isfinite(options.timeout_seconds) || options.timeout_seconds <= 0 ||
        !std::isfinite(options.validation_joint_step) || options.validation_joint_step <= 0 ||
        options.validation_min_substeps < 1) {
        out.reason = "invalid_approach_options"; return out;
    }
    if (stopped()) { out.reason = "deadline_during_approach"; return out; }
    const auto initial = scene.collision(start, {});
    if (!initial.valid) { out.reason = "approach_start_collision_"+initial.pair; return out; }
    out.path.push_back(start);
    if (stopped()) { out.reason = "deadline_during_approach"; return out; }
    const auto endpoint = scene.collision(goal, {});
    if (!endpoint.valid) { out.reason = "approach_goal_collision_"+endpoint.pair; return out; }

    // Check precisely the sample grid used by the exported dense knot chain
    // and final validation. Check endpoints too; no corner-cutting shortcuts.
    auto clear = [&](const JointVector& a, const JointVector& b) {
        const int knots = std::max(1, static_cast<int>(std::ceil((b-a).cwiseAbs().maxCoeff()/.02)));
        const JointVector step = (b-a)/knots;
        const int samples = std::max(options.validation_min_substeps,
            static_cast<int>(std::ceil(1.5*step.cwiseAbs().maxCoeff()/options.validation_joint_step)));
        for (int i = 0; i < knots; ++i) for (int j = 1; j <= samples; ++j) {
            if (stopped()) return false;
            const double u = double(j)/samples, blend = u*u*(3-2*u);
            const JointVector q = a+((i+blend)/knots)*(b-a);
            if (!scene.collision(q, {}).valid) return false;
        }
        return true;
    };
    if (clear(start, goal)) {
        out.path.push_back(goal); out.success = true; out.reason = "direct_collision_checked";
        out.initial_length = out.optimized_length = (goal-start).norm();
        return out;
    }
    if (stopped()) { out.reason = "deadline_during_approach"; return out; }

    Tree from_start{{start, -1}}, from_goal{{goal, -1}};
    std::mt19937 random(20261011);
    std::uniform_real_distribution<double> unit(0, 1);
    std::normal_distribution<double> normal(0, 1);
    auto nearest = [](const Tree& tree, const JointVector& q) {
        int best = 0;
        for (std::size_t i = 1; i < tree.size(); ++i)
            if ((tree[i].q-q).squaredNorm() < (tree[best].q-q).squaredNorm()) best = static_cast<int>(i);
        return best;
    };
    // Bounded steps allow both trees to explore around collisions. Samples
    // mix a corridor around the original line with the entire joint box.
    auto extend = [&](Tree& tree, const JointVector& target, int parent) {
        const JointVector delta = target-tree[parent].q;
        const double distance = delta.norm();
        if (distance < 1e-12) return parent;
        const JointVector next = distance <= .3 ? target : (tree[parent].q+(.3/distance)*delta).eval();
        if (!clear(tree[parent].q, next)) return -1;
        tree.push_back({next, parent});
        return static_cast<int>(tree.size()-1);
    };
    for (std::size_t iteration = 0; iteration < 2000 && !stopped(); ++iteration) {
        ++out.iterations;
        const bool forward = iteration%2 == 0;
        Tree& a = forward ? from_start : from_goal;
        Tree& b = forward ? from_goal : from_start;
        JointVector target(scene.dof());
        const double kind = unit(random);
        if (kind < .15) target = b.front().q;
        else if (kind < .85) {
            target = start+unit(random)*(goal-start);
            const double radius = iteration < 200 ? .45 : .9;
            for (int j = 0; j < scene.dof(); ++j) target[j] += radius*normal(random);
            target = target.cwiseMax(scene.limits().lower).cwiseMin(scene.limits().upper);
        } else for (int j = 0; j < scene.dof(); ++j)
            target[j] = scene.limits().lower[j]+unit(random)*(scene.limits().upper[j]-scene.limits().lower[j]);
        const int added = extend(a, target, nearest(a, target));
        if (added < 0) continue;
        int other = nearest(b, a[added].q);
        while (!stopped()) {
            const int next = extend(b, a[added].q, other);
            if (next < 0) break;
            other = next;
            if ((b[other].q-a[added].q).norm() < 1e-12) {
                auto left = branch(forward ? a : b, forward ? added : other);
                auto right = branch(forward ? b : a, forward ? other : added);
                left.insert(left.end(), right.rbegin()+1, right.rend());
                out.path = std::move(left); out.success = true;
                break;
            }
        }
        if (out.success) break;
    }
    if (!out.success) {
        out.path = branch(from_start, nearest(from_start, goal));
        out.reason = stopped() ? "deadline_during_approach" : "approach_search_exhausted";
        out.initial_length = out.optimized_length = length(out.path);
        return out;
    }
    out.initial_length = length(out.path);
    // Deterministic longest-first shortcuts, then bounded random shortcuts.
    // A replacement is accepted only when shorter and collision-free.
    for (int attempt = 0; attempt < 80 && out.path.size() > 2 && !stopped(); ++attempt) {
        const std::size_t a = attempt == 0 ? 0 : random()%(out.path.size()-2);
        const std::size_t b = attempt == 0 ? out.path.size()-1 : a+2+random()%(out.path.size()-a-2);
        double before = 0;
        for (std::size_t i = a+1; i <= b; ++i) before += (out.path[i]-out.path[i-1]).norm();
        if ((out.path[b]-out.path[a]).norm() >= before-1e-10) continue;
        if (clear(out.path[a], out.path[b])) {
            out.path.erase(out.path.begin()+a+1, out.path.begin()+b);
            ++out.shortcuts;
        }
    }
    out.optimized_length = length(out.path);
    out.reason = "collision_checked_shortcuts";
    return out;
}
} // namespace tech_core::planning
