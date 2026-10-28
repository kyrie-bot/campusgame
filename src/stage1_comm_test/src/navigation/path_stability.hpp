#pragma once

#include "navigation/dijkstra.hpp"

namespace campus {
// Keep a currently traversable route unless a new Dijkstra result improves
// its remaining weighted cost by 8%. Every retained edge is checked again.
inline bool retainGridRoute(const DijkstraGrid& grid, const std::vector<double>& risk, const std::vector<int>& previous,
                            GridPlan& fresh) {
    if (fresh.status != "OK" || fresh.cells.empty() || previous.empty() || previous.back() != fresh.cells.back() ||
        (!risk.empty() && risk.size() != grid.cells.size()))
        return false;
    auto start = std::find(previous.begin(), previous.end(), fresh.cells.front());
    if (start == previous.end())
        return false;
    double cost = 0, length = 0;

    for (auto it = start; it != previous.end(); ++it) {
        int cell = *it;
        if (cell < 0 || cell >= static_cast<int>(grid.cells.size()) || grid.cells[cell] != 0)
            return false;
        if (it == start)
            continue;
        int before = *(it - 1);

        if (!grid.canStep(before % grid.width, before / grid.width, cell % grid.width, cell / grid.width))
            return false;
        double step = std::hypot(cell % grid.width - before % grid.width, cell / grid.width - before / grid.width) *
                      grid.resolution;
        double weight = risk.empty() ? 1 : 1 + (risk[before] + risk[cell]) / 2;

        if (!std::isfinite(weight) || weight < 1)
            return false;
        length += step;
        cost += step * weight;
    }

    if (fresh.total_cost < cost * .92)
        return false;
    fresh.cells.assign(start, previous.end());
    fresh.total_cost = cost;
    fresh.length_m = length;
    return true;
}
} // namespace campus