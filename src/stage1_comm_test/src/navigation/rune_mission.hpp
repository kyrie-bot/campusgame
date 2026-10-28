#pragma once

#include "navigation/assault_mission.hpp"

namespace campus {
// Coarse centre of the rune on the public symmetric field drawing. This
// selects a search position; visual geometry supplies the actual shooting pose.
inline PlanPoint runeUnityCentre(int player) {
    (void)player;
    return {-1.5, 0.};
}

inline bool chooseRunePosition(const DijkstraGrid& grid, const DijkstraGrid& raw, const std::vector<double>& costs,
                               PlanPoint centre, PlanPoint outward, PlanPoint& goal,
                               const std::vector<int64_t>& ground_excluded, const std::vector<int64_t>& low_ledges) {
    if (costs.size() != grid.cells.size())
        return false;
    // Missing ground classification must never silently authorize a highland station.
    if (ground_excluded.empty())
        return false;
    auto ground = grid;

    for (auto cell : ground_excluded)
        if (cell >= 0 && static_cast<size_t>(cell) < ground.cells.size())
            ground.cells[cell] = 100;
    // A low platform lip blocks chassis motion, but not the elevated rotor.
    // Keep all other occupied/unknown cells in the firing-ray check.
    auto sight = raw;
    for (auto cell : low_ledges)
        if (cell >= 0 && static_cast<size_t>(cell) < sight.cells.size() && sight.cells[cell] > 0)
            sight.cells[cell] = 0;

    // Prefer the fixture normal before route length. A nearby side view
    // must not win over the reachable frontal point farther along the route.
    for (int tier = 0; tier <= 3; ++tier) {
        double best = INFINITY;
        for (double r : {3.5, 4., 4.5, 5., 5.5, 6., 6.5, 7.})
            for (int i = -tier; i <= tier; ++i) {
                if (std::abs(i) != tier)
                    continue;
                double a = std::atan2(outward.y, outward.x) + i * M_PI / 48;
                int cell = grid.cellAt({centre.x + r * std::cos(a), centre.y + r * std::sin(a)});

                if (cell < 0 || grid.cells[cell] != 0 || !std::isfinite(costs[cell]))
                    continue;
                auto p = grid.centre(cell);
                if (!segmentWithReserve(ground, p, p, .4) || !attackRayFree(sight, p, centre, .9))
                    continue;
                double score = costs[cell] + 2 * std::abs(r - 3.5) + std::abs(i) * .12;

                if (score < best) {
                    best = score;
                    goal = p;
                }
            }

        if (std::isfinite(best))
            return true;
    }

    return false;
}

// Temporary loss of routing/state is not a completed rune opportunity.
inline bool runeVisitInterrupted(const std::string& reason) {
    return reason == "MISSION_LEFT" || reason == "CANCELLED" || reason == "REFILL_REQUIRED";
}

// Repeated terminal status is lifecycle feedback, never a new activation.
inline bool runeRequestStartsVisit(bool current_goal, uint64_t request, bool finished, bool visit_done) {
    return current_goal && request != 0 && !finished && !visit_done;
}

// A failed accepted big opportunity has no effect to use on the robots.
// Keep the station and start the next visit, only after its 20 s window ends.
inline bool retryBigRuneAtStation(unsigned opportunities, bool effect_active, bool uncertain,
                                  const std::string& reason) {
    return opportunities >= 3 && opportunities < 5 && !effect_active && !uncertain &&
           reason == "ACTIVATION_TIMEOUT";
}

// Use each active effect for combat: robots first, then outpost.
// Before base combat, finish remaining opportunities and wait at the station
// for an active effect to expire; the rune controller owns cooldown/request gating.
// Expiry reopens the visit in BuffStatus; retain the retry delay for failed visits.
inline bool runeVisitDue(bool exhausted, bool visit_done, bool effect_active, unsigned opportunities,
                         double retry_age, bool before_base = false) {
    return !exhausted && (before_base || (!effect_active &&
           (opportunities == 0 || !visit_done || retry_age >= 10.)));
}
} // namespace campus
