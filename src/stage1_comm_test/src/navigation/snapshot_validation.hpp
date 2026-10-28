#pragma once

#include <cmath>
#include <string>

namespace campus {
// The public broadcast has six units (2..7); confirmed vision adds unit 1.
template <class Snapshot> std::string enemySnapshotReason(const Snapshot& snapshot) {
    if (snapshot.enemy_centres.size() > 7)
        return "TOO_MANY_ENEMIES";
    if (snapshot.enemy_centres.size() != snapshot.enemy_radii.size() ||
        snapshot.enemy_centres.size() != snapshot.enemy_mobile.size())
        return "ENEMY_ARRAY_LENGTH";
    if (!snapshot.enemy_hard_radii.empty() && snapshot.enemy_hard_radii.size() != snapshot.enemy_centres.size())
        return "ENEMY_HARD_ARRAY_LENGTH";

    if (!std::isfinite(snapshot.enemy_speed_bound_mps) || snapshot.enemy_speed_bound_mps < 0 ||
        snapshot.enemy_speed_bound_mps > 20)
        return "ENEMY_SPEED_BOUND";
    for (std::size_t i = 0; i < snapshot.enemy_centres.size(); ++i) {
        if (!std::isfinite(snapshot.enemy_centres[i].x) || !std::isfinite(snapshot.enemy_centres[i].y))
            return "ENEMY_POSITION";
        if (!std::isfinite(snapshot.enemy_radii[i]) || snapshot.enemy_radii[i] <= 0 || snapshot.enemy_radii[i] > 100)
            return "ENEMY_RADIUS";

        if (!snapshot.enemy_hard_radii.empty() &&
            (!std::isfinite(snapshot.enemy_hard_radii[i]) || snapshot.enemy_hard_radii[i] <= 0 ||
             snapshot.enemy_hard_radii[i] > snapshot.enemy_radii[i]))
            return "ENEMY_HARD_RADIUS";
    }

    return {};
}
} // namespace campus