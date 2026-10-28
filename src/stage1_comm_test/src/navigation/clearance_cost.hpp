#pragma once

#include "dijkstra.hpp"

#include <stdexcept>

namespace campus {
// Soft preference only: hard collision checking still uses the inflated grid.
// Narrow ramps remain traversable. Distances are an octile approximation and
// must never be used as a replacement for collision checking.
inline std::vector<double> wallPreference(const DijkstraGrid& grid, double preferred, double weight) {
    if (!grid.valid() || !std::isfinite(preferred) || preferred <= 0 || !std::isfinite(weight) || weight < 0) {
        throw std::invalid_argument("Invalid wall preference");
    }

    std::vector<double> distance(grid.cells.size(), INFINITY);
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;

    for (int i = 0; i < static_cast<int>(grid.cells.size()); ++i) {
        int x = i % grid.width, y = i / grid.width;
        if (grid.cells[i] != 0 || x == 0 || y == 0 || x == grid.width - 1 || y == grid.height - 1) {
            distance[i] = grid.cells[i] != 0 ? 0 : grid.resolution / 2;
            queue.emplace(distance[i], i);
        }
    }

    while (!queue.empty()) {
        auto [value, i] = queue.top();
        queue.pop();
        if (value != distance[i] || value >= preferred + grid.resolution)
            continue;

        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) {
                int x = i % grid.width + dx, y = i / grid.width + dy;
                if ((!dx && !dy) || x < 0 || y < 0 || x >= grid.width || y >= grid.height)
                    continue;
                int next = y * grid.width + x;
                double candidate = value + grid.resolution * (dx && dy ? std::sqrt(2.) : 1.);

                if (candidate < distance[next]) {
                    distance[next] = candidate;
                    queue.emplace(candidate, next);
                }
            }
    }

    for (double& value : distance) {
        double proximity = std::max(0., 1. - std::max(0., value - grid.resolution / 2) / preferred);
        value = weight * proximity * proximity;
    }

    return distance;
}
} // namespace campus