/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

/*
    Modified for MMP: trimmed to the self-contained RRT* implementation only.
    Original SFC helpers (convexCover, shortCut, firi, geo_utils) were removed
    once SDF-based planning replaced the corridor pipeline.
*/

#ifndef SFC_GEN_HPP
#define SFC_GEN_HPP

#include <algorithm>
#include <memory>
#include <random>
#include <vector>
#include <Eigen/Eigen>

namespace sfc_gen
{
    // ===================== Self-contained RRT* =====================

    struct RRTNode
    {
        Eigen::Vector3d pos;
        int parent;
        double cost;
    };

    // Check if a straight line from a to b is collision-free
    // Map must provide: mapPtr->query(Eigen::Vector3d) returning 0 if free
    template <typename Map>
    inline bool isSegmentFree(const Eigen::Vector3d &a,
                              const Eigen::Vector3d &b,
                              const Map *mapPtr,
                              const double step = 0.5)
    {
        const Eigen::Vector3d diff = b - a;
        const double dist = diff.norm();
        if (dist < 1e-6) return true;
        const int n_checks = std::max(2, (int)std::ceil(dist / step));
        for (int i = 0; i <= n_checks; ++i)
        {
            const double t = (double)i / (double)n_checks;
            const Eigen::Vector3d p = a + t * diff;
            if (mapPtr->query(p) != 0)
                return false;
        }
        return true;
    }

    // Compute risk-weighted edge cost along segment a→b
    // cost = ∫ (1 + α·T(x)) ds ≈ dist × avg_multiplier
    // Uses trapezoidal sampling for accuracy
    template <typename Map>
    inline double segmentRiskCost(const Eigen::Vector3d &a,
                                    const Eigen::Vector3d &b,
                                    const Map *mapPtr,
                                    const int n_samples = 3)
    {
        const double dist = (b - a).norm();
        if (dist < 1e-6) return 0.0;
        double sum = 0.0;
        for (int i = 0; i <= n_samples; ++i)
        {
            const double t = (double)i / (double)n_samples;
            const Eigen::Vector3d p = a + t * (b - a);
            double w = (i == 0 || i == n_samples) ? 0.5 : 1.0;
            sum += w * mapPtr->getRiskCostMultiplier(p);
        }
        return dist * sum / n_samples;
    }

    template <typename Map>
    inline double planPath(const Eigen::Vector3d &s,
                           const Eigen::Vector3d &g,
                           const Eigen::Vector3d &lb,
                           const Eigen::Vector3d &hb,
                           const Map *mapPtr,
                           const double &timeout,
                           std::vector<Eigen::Vector3d> &p)
    {
        // RRT* parameters
        const int max_iter = 30000;
        const double step_size = 2.0;
        const double goal_threshold = step_size * 2.0;
        const double rewire_radius_base = 3.0;

        // Z sampling: allow climbing up to the bbox ceiling so RRT* can route
        // over terrain obstacles (mountains) when start/goal are near-sea-level.
        // Lower bound clipped near start/goal to avoid wasting samples underground.
        const double z_lo = std::max(lb(2), std::min(s(2), g(2)) - 5.0);
        const double z_hi = hb(2);

        std::mt19937_64 rng(std::chrono::steady_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<double> dist_x(lb(0), hb(0));
        std::uniform_real_distribution<double> dist_y(lb(1), hb(1));
        std::uniform_real_distribution<double> dist_z(z_lo, z_hi);
        std::uniform_real_distribution<double> dist_01(0.0, 1.0);

        // Check start/goal validity
        if (mapPtr->query(s) != 0 || mapPtr->query(g) != 0)
        {
            return INFINITY;
        }

        std::vector<RRTNode> tree;
        tree.reserve(max_iter + 1);
        tree.push_back({s, -1, 0.0});

        // Children index for fast cost propagation during rewiring
        std::vector<std::vector<int>> children;
        children.resize(max_iter + 2);

        int best_goal_idx = -1;
        double best_cost = INFINITY;
        bool has_Risk = false;
        bool direct_solution_valid = false;  // true iff best is direct s→g line

        // If direct connection is collision-free, use it as initial reference.
        // When risk cost exists, let RRT* try to find a cheaper detour.
        if (isSegmentFree(s, g, mapPtr, step_size * 0.5))
        {
            double direct_cost = segmentRiskCost(s, g, mapPtr);
            double pure_dist = (g - s).norm();
            // No risk along direct path: return immediately (it's optimal)
            if (direct_cost <= pure_dist * 1.01) {
                p.clear();
                p.push_back(s);
                p.push_back(g);
                return direct_cost;
            }
            // Otherwise: direct path has risk cost; set as initial best.
            // Do NOT inject goal into tree — its high cost would corrupt rewiring.
            has_Risk = true;
            best_cost = direct_cost;
            direct_solution_valid = true;
        }

        auto start_time = std::chrono::steady_clock::now();

        for (int iter = 0; iter < max_iter; ++iter)
        {
            // Timeout check
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - start_time).count();
            if (elapsed > timeout && best_goal_idx >= 0)
                break;

            // Sample random point with goal bias and risk-aware lateral bias.
            // Lateral bias (informed-sampling style) stabilizes RRT* when risk
            // zones force detours through narrow lateral corridors; without it
            // uniform sampling yields high-variance results (Gammell et al. 2014).
            Eigen::Vector3d rand_pt;
            double r = dist_01(rng);
            if (r < 0.15)
            {
                rand_pt = g;
            }
            else if (has_Risk && r < 0.30)
            {
                // Sample along start-goal axis with lateral offset
                double t_along = dist_01(rng);
                Eigen::Vector3d midpt = s + t_along * (g - s);
                double lateral = (dist_01(rng) - 0.5) * 2.0 * (hb(1) - lb(1)) * 0.5;
                rand_pt = Eigen::Vector3d(midpt(0), midpt(1) + lateral, midpt(2));
                rand_pt(0) = std::max(lb(0), std::min(hb(0), rand_pt(0)));
                rand_pt(1) = std::max(lb(1), std::min(hb(1), rand_pt(1)));
                rand_pt(2) = std::max(z_lo, std::min(z_hi, rand_pt(2)));
            }
            else
            {
                rand_pt = Eigen::Vector3d(dist_x(rng), dist_y(rng), dist_z(rng));
            }

            // Find nearest node
            int nearest_idx = 0;
            double nearest_dist = (tree[0].pos - rand_pt).norm();
            for (int i = 1; i < (int)tree.size(); ++i)
            {
                double d = (tree[i].pos - rand_pt).norm();
                if (d < nearest_dist)
                {
                    nearest_dist = d;
                    nearest_idx = i;
                }
            }

            // Steer towards random point
            Eigen::Vector3d new_pt;
            if (nearest_dist <= step_size)
            {
                new_pt = rand_pt;
            }
            else
            {
                new_pt = tree[nearest_idx].pos +
                         (rand_pt - tree[nearest_idx].pos).normalized() * step_size;
            }

            // Bounds check
            if (new_pt(0) < lb(0) || new_pt(0) > hb(0) ||
                new_pt(1) < lb(1) || new_pt(1) > hb(1) ||
                new_pt(2) < lb(2) || new_pt(2) > hb(2))
                continue;

            // Collision check
            if (mapPtr->query(new_pt) != 0)
                continue;
            if (!isSegmentFree(tree[nearest_idx].pos, new_pt, mapPtr, step_size * 0.5))
                continue;

            // Find neighbors for rewiring
            // Use larger fixed radius; the log(n)/n formula underestimates for large n
            // With risk zones, extra radius helps find cheaper detour parents
            const double rewire_radius = has_Risk ? step_size * 6.0 : step_size * 3.0;

            int best_parent = nearest_idx;
            double best_new_cost = tree[nearest_idx].cost + segmentRiskCost(tree[nearest_idx].pos, new_pt, mapPtr);

            std::vector<int> near_idxs;
            for (int i = 0; i < (int)tree.size(); ++i)
            {
                double d = (tree[i].pos - new_pt).norm();
                if (d < rewire_radius)
                {
                    near_idxs.push_back(i);
                    // Quick check: even pure distance can't beat current best?
                    if (tree[i].cost + d >= best_new_cost)
                        continue;
                    if (isSegmentFree(tree[i].pos, new_pt, mapPtr, step_size * 0.5))
                    {
                        double potential_cost = tree[i].cost + segmentRiskCost(tree[i].pos, new_pt, mapPtr);
                        if (potential_cost < best_new_cost)
                        {
                            best_parent = i;
                            best_new_cost = potential_cost;
                        }
                    }
                }
            }

            // Add new node
            int new_idx = tree.size();
            tree.push_back({new_pt, best_parent, best_new_cost});
            if (best_parent >= 0) children[best_parent].push_back(new_idx);

            // Rewire neighbors with OMPL-style cost propagation (updateChildCosts)
            for (int ni : near_idxs)
            {
                double d = (new_pt - tree[ni].pos).norm();
                if (best_new_cost + d >= tree[ni].cost)
                    continue;
                if (isSegmentFree(new_pt, tree[ni].pos, mapPtr, step_size * 0.5))
                {
                    double potential_cost = best_new_cost + segmentRiskCost(new_pt, tree[ni].pos, mapPtr);
                    if (potential_cost < tree[ni].cost)
                    {
                        // Remove ni from old parent's children
                        int old_parent = tree[ni].parent;
                        if (old_parent >= 0) {
                            auto &ch = children[old_parent];
                            ch.erase(std::remove(ch.begin(), ch.end(), ni), ch.end());
                        }

                        tree[ni].parent = new_idx;
                        tree[ni].cost = potential_cost;
                        children[new_idx].push_back(ni);

                        // OMPL-style updateChildCosts:
                        // Recompute each descendant's cost from its parent's cost + actual edge cost
                        std::vector<int> queue;
                        queue.push_back(ni);
                        size_t qi = 0;
                        while (qi < queue.size())
                        {
                            int pid = queue[qi++];
                            for (int cid : children[pid])
                            {
                                // Recompute actual cost: parent cost + edge cost
                                tree[cid].cost = tree[pid].cost +
                                    segmentRiskCost(tree[pid].pos, tree[cid].pos, mapPtr);
                                queue.push_back(cid);
                            }
                        }
                    }
                }
            }

            // Check if we reached the goal
            double dist_to_goal = (new_pt - g).norm();
            if (dist_to_goal < goal_threshold)
            {
                if (isSegmentFree(new_pt, g, mapPtr, step_size * 0.5))
                {
                    double goal_cost = best_new_cost + segmentRiskCost(new_pt, g, mapPtr);
                    if (goal_cost < best_cost)
                    {
                        best_cost = goal_cost;
                        best_goal_idx = new_idx;
                    }
                }
            }
        }

        // No solution at all
        if (best_goal_idx < 0 && !direct_solution_valid)
        {
            return INFINITY;
        }

        // RRT* did not improve over initial direct solution — return straight line
        if (best_goal_idx < 0)
        {
            p.clear();
            p.push_back(s);
            p.push_back(g);
            return best_cost;
        }

        // Reconstruct path from RRT* tree
        p.clear();
        p.push_back(g);
        int idx = best_goal_idx;
        while (idx >= 0)
        {
            p.push_back(tree[idx].pos);
            idx = tree[idx].parent;
        }
        std::reverse(p.begin(), p.end());

        // Shortcutting removed: downstream path_shortening (SDF + risk)
        // handles redundant-waypoint removal with better information. Keeping
        // RRT*'s raw tree path preserves waypoint density so MINCO/L-BFGS
        // have enough inner points to shape the trajectory.

        return best_cost;
    }

} // namespace sfc_gen

#endif
