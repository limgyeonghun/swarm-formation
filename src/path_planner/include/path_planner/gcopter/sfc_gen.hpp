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
    Modified for MMP: Replaced OMPL-based planPath with a self-contained RRT* implementation.
    convexCover and shortCut are kept unchanged from the original GCOPTER.
*/

#ifndef SFC_GEN_HPP
#define SFC_GEN_HPP

#include "geo_utils.hpp"
#include "firi.hpp"

#include <deque>
#include <memory>
#include <random>
#include <vector>
#include <unordered_set>
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
        const int max_iter = 20000;
        const double step_size = 2.0;
        const double goal_threshold = step_size * 1.5;
        const double rewire_radius_base = 3.0;

        std::mt19937_64 rng(std::chrono::steady_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<double> dist_x(lb(0), hb(0));
        std::uniform_real_distribution<double> dist_y(lb(1), hb(1));
        std::uniform_real_distribution<double> dist_z(lb(2), hb(2));
        std::uniform_real_distribution<double> dist_01(0.0, 1.0);

        // Check start/goal validity
        if (mapPtr->query(s) != 0 || mapPtr->query(g) != 0)
        {
            return INFINITY;
        }

        // Try direct connection first
        if (isSegmentFree(s, g, mapPtr, step_size * 0.5))
        {
            p.clear();
            p.push_back(s);
            p.push_back(g);
            return (g - s).norm();
        }

        std::vector<RRTNode> tree;
        tree.reserve(max_iter + 1);
        tree.push_back({s, -1, 0.0});

        int best_goal_idx = -1;
        double best_cost = INFINITY;

        auto start_time = std::chrono::steady_clock::now();

        for (int iter = 0; iter < max_iter; ++iter)
        {
            // Timeout check
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - start_time).count();
            if (elapsed > timeout && best_goal_idx >= 0)
                break;

            // Sample random point (10% goal bias)
            Eigen::Vector3d rand_pt;
            if (dist_01(rng) < 0.1)
            {
                rand_pt = g;
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
            const double n_nodes = (double)tree.size();
            const double rewire_radius = std::min(
                rewire_radius_base * std::pow(std::log(n_nodes + 1.0) / (n_nodes + 1.0), 1.0 / 3.0),
                step_size * 3.0);

            int best_parent = nearest_idx;
            double best_new_cost = tree[nearest_idx].cost + (tree[nearest_idx].pos - new_pt).norm();

            std::vector<int> near_idxs;
            for (int i = 0; i < (int)tree.size(); ++i)
            {
                double d = (tree[i].pos - new_pt).norm();
                if (d < rewire_radius)
                {
                    near_idxs.push_back(i);
                    double potential_cost = tree[i].cost + d;
                    if (potential_cost < best_new_cost)
                    {
                        if (isSegmentFree(tree[i].pos, new_pt, mapPtr, step_size * 0.5))
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

            // Rewire neighbors
            for (int ni : near_idxs)
            {
                double potential_cost = best_new_cost + (tree[ni].pos - new_pt).norm();
                if (potential_cost < tree[ni].cost)
                {
                    if (isSegmentFree(new_pt, tree[ni].pos, mapPtr, step_size * 0.5))
                    {
                        tree[ni].parent = new_idx;
                        tree[ni].cost = potential_cost;
                    }
                }
            }

            // Check if we reached the goal
            double dist_to_goal = (new_pt - g).norm();
            if (dist_to_goal < goal_threshold)
            {
                if (isSegmentFree(new_pt, g, mapPtr, step_size * 0.5))
                {
                    double goal_cost = best_new_cost + dist_to_goal;
                    if (goal_cost < best_cost)
                    {
                        best_cost = goal_cost;
                        best_goal_idx = new_idx;
                    }
                }
            }
        }

        if (best_goal_idx < 0)
        {
            return INFINITY;
        }

        // Reconstruct path
        p.clear();
        p.push_back(g);
        int idx = best_goal_idx;
        while (idx >= 0)
        {
            p.push_back(tree[idx].pos);
            idx = tree[idx].parent;
        }
        std::reverse(p.begin(), p.end());

        // Shortcutting: remove redundant waypoints where direct connection is free
        if (p.size() > 2)
        {
            std::vector<Eigen::Vector3d> shortened;
            shortened.push_back(p.front());
            size_t i = 0;
            while (i < p.size() - 1)
            {
                // Find the farthest point directly reachable from p[i]
                size_t farthest = i + 1;
                for (size_t j = p.size() - 1; j > i + 1; --j)
                {
                    if (isSegmentFree(p[i], p[j], mapPtr, step_size * 0.5))
                    {
                        farthest = j;
                        break;
                    }
                }
                shortened.push_back(p[farthest]);
                i = farthest;
            }
            p = shortened;
        }

        return best_cost;
    }

    // ===================== convexCover (unchanged from GCOPTER) =====================

    inline void convexCover(const std::vector<Eigen::Vector3d> &path,
                            const std::vector<Eigen::Vector3d> &points,
                            const Eigen::Vector3d &lowCorner,
                            const Eigen::Vector3d &highCorner,
                            const double &progress,
                            const double &range,
                            std::vector<Eigen::MatrixX4d> &hpolys,
                            const double eps = 1.0e-6)
    {
        hpolys.clear();
        const int n = path.size();
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;
        bd(1, 0) = -1.0;
        bd(2, 1) = 1.0;
        bd(3, 1) = -1.0;
        bd(4, 2) = 1.0;
        bd(5, 2) = -1.0;

        Eigen::MatrixX4d hp, gap;
        Eigen::Vector3d a, b = path[0];
        std::vector<Eigen::Vector3d> valid_pc;
        std::vector<Eigen::Vector3d> bs;
        valid_pc.reserve(points.size());
        for (int i = 1; i < n;)
        {
            a = b;
            if ((a - path[i]).norm() > progress)
            {
                b = (path[i] - a).normalized() * progress + a;
            }
            else
            {
                b = path[i];
                i++;
            }
            bs.emplace_back(b);

            bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
            bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
            bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
            bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
            bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));
            bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));

            valid_pc.clear();
            for (const Eigen::Vector3d &p : points)
            {
                if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0)
                {
                    valid_pc.emplace_back(p);
                }
            }
            Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3, valid_pc.size());

            firi::firi(bd, pc, a, b, hp);

            if (hpolys.size() != 0)
            {
                const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
                if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() +
                             ((hpolys.back() * ah).array() > -eps).cast<int>().sum())
                {
                    firi::firi(bd, pc, a, a, gap, 1);
                    hpolys.emplace_back(gap);
                }
            }

            hpolys.emplace_back(hp);
        }
    }

    // ===================== shortCut (unchanged from GCOPTER) =====================

    inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys)
    {
        std::vector<Eigen::MatrixX4d> htemp = hpolys;
        if (htemp.size() == 1)
        {
            Eigen::MatrixX4d headPoly = htemp.front();
            htemp.insert(htemp.begin(), headPoly);
        }
        hpolys.clear();

        int M = htemp.size();
        Eigen::MatrixX4d hPoly;
        bool overlap;
        std::deque<int> idices;
        idices.push_front(M - 1);
        for (int i = M - 1; i >= 0; i--)
        {
            for (int j = 0; j < i; j++)
            {
                if (j < i - 1)
                {
                    overlap = geo_utils::overlap(htemp[i], htemp[j], 0.01);
                }
                else
                {
                    overlap = true;
                }
                if (overlap)
                {
                    idices.push_front(j);
                    i = j + 1;
                    break;
                }
            }
        }
        for (const auto &ele : idices)
        {
            hpolys.push_back(htemp[ele]);
        }
    }

}

#endif
