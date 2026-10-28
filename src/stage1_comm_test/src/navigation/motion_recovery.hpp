#pragma once

#include "navigation/path_following.hpp"

#include <optional>

namespace campus {
// Keep physical progress across refreshed route IDs and brief transport stops.
// A refreshed Dijkstra path is not evidence that the chassis has moved.
class MotionRecovery {
  public:
    void reset() {
        origin_.reset();
        elapsed_ = 0;
        attempts_ = 0;
    }

    void observe(PlanPoint position, double dt) {
        if (!origin_ || pointDistance(*origin_, position) >= .08) {
            origin_ = position;
            elapsed_ = 0;
            attempts_ = 0;
        } else
            elapsed_ += std::clamp(dt, 0., .05);
    }

    bool needed() const {
        return elapsed_ >= 2.;
    }

    bool active() const {
        return attempts_ > 0;
    }

    bool exhausted() const {
        return attempts_ >= 3;
    }

    std::optional<PlanPoint> probe(const PathFollower& controller, const DijkstraGrid& grid,
                                   const std::vector<LocalObstacle>& obstacles, PlanPoint position, PlanPoint velocity,
                                   PlanPoint target, double age) {
        if (!needed() || exhausted() || pointDistance(position, target) < .005 ||
            std::hypot(velocity.x, velocity.y) > .03)
            return {};
        double heading = std::atan2(target.y - position.y, target.x - position.x);
        // First clear the lip to the right of the blocked travel direction,
        // then try its left side or back out. Every complete brake is checked.
        double offsets[] = {-M_PI / 2, M_PI / 2, M_PI};
        double offset = offsets[attempts_++];
        elapsed_ = 0;
        double centre = heading + offset;
        // In a narrow rectilinear passage, a perpendicular to an off-centre
        // pursuit vector may point into its side wall. Test the nearby grid
        // axis as well; do not keep retrying the same diagonal into the lip.
        double cardinal = std::round(centre / (M_PI / 2)) * (M_PI / 2);

        // A ramp may have only 6–10 cm of lateral reserve. A short checked
        // step can clear contact when every 20 cm probe hits a mapped wall.
        for (double direction : {cardinal, centre, centre - M_PI / 6, centre + M_PI / 6})
            for (double distance : {.4, .3, .2, .1, .06}) {
                PlanPoint goal{position.x + distance * std::cos(direction),
                               position.y + distance * std::sin(direction)};
                const double speed = std::min(.25, distance);
                PlanPoint trial{speed * std::cos(direction), speed * std::sin(direction)};
                std::vector<PlanPoint> rollout;

                if (segmentWithReserve(grid, position, goal, .03) &&
                    controller.rollout(grid, obstacles, position, velocity, trial, rollout, age)) {
                    travel_heading_ = std::round(heading / (M_PI / 2)) * (M_PI / 2);
                    return goal;
                }
            }

        return {};
    }

    // Complete a dogleg before letting pursuit cut diagonally back into the
    // same contact. In narrow passages even a 6 cm checked leg can centre the
    // chassis above the lip. It is actual motion, not a changed map cell.
    std::optional<PlanPoint> rejoin(const PathFollower& controller, const DijkstraGrid& grid,
                                    const std::vector<LocalObstacle>& obstacles, PlanPoint position, PlanPoint velocity,
                                    double age) const {
        for (double distance : {.3, .2, .1, .06}) {
            PlanPoint goal{position.x + distance * std::cos(travel_heading_),
                           position.y + distance * std::sin(travel_heading_)};
            const double speed = std::min(.25, distance);
            PlanPoint trial{speed * std::cos(travel_heading_), speed * std::sin(travel_heading_)};
            std::vector<PlanPoint> rollout;

            if (segmentWithReserve(grid, position, goal, .03) &&
                controller.rollout(grid, obstacles, position, velocity, trial, rollout, age))
                return goal;
        }

        return {};
    }

  private:
    std::optional<PlanPoint> origin_;
    double elapsed_ = 0;
    int attempts_ = 0;
    double travel_heading_ = 0;
};
} // namespace campus