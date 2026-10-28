#pragma once

#include "navigation/dijkstra.hpp"

#include <stdexcept>
#include <optional>

namespace campus {
inline double pointDistance(PlanPoint a, PlanPoint b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

inline PlanPoint boundedVector(PlanPoint value, double limit) {
    double magnitude = std::hypot(value.x, value.y);
    if (magnitude > limit && magnitude > 0) {
        value.x *= limit / magnitude;
        value.y *= limit / magnitude;
    }

    return value;
}

inline double curvatureSpeedLimit(const std::vector<PlanPoint>& path, std::size_t nearest, double maximum,
                                  double lateral_acceleration = .9) {
    if (nearest >= path.size())
        return maximum;
        
    std::size_t middle = nearest, last = nearest;
    double length = 0;

    for (std::size_t i = nearest + 1; i < path.size(); ++i) {
        length += pointDistance(path[i - 1], path[i]);
        if (length <= .45)
            middle = i;
        last = i;

        if (length >= 1.25)
            break;
    }

    auto a = path[nearest], b = path[middle], c = path[last];
    double denominator = pointDistance(a, b) * pointDistance(b, c) * pointDistance(a, c);
    if (denominator < 1e-6)
        return maximum;
    double curvature = 2 * std::abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) / denominator;
    return std::min(maximum, std::sqrt(lateral_acceleration / std::max(curvature, 1e-6)));
}

// 遍历线段经过的每个格子。跨角时同时检查相邻两格，不能抄近路穿墙。
inline bool segmentFree(const DijkstraGrid& grid, PlanPoint first, PlanPoint last) {
    int begin = grid.cellAt(first);
    int end = grid.cellAt(last);
    if (begin < 0 || end < 0 || grid.cells[begin] != 0 || grid.cells[end] != 0) {
        return false;
    }

    int x = begin % grid.width;
    int y = begin / grid.width;
    int end_x = end % grid.width;
    int end_y = end / grid.width;
    double dx = last.x - first.x;
    double dy = last.y - first.y;
    int step_x = dx > 0 ? 1 : -1;
    int step_y = dy > 0 ? 1 : -1;

    double infinity = std::numeric_limits<double>::infinity();
    double next_x = infinity;
    double next_y = infinity;
    double delta_x = infinity;
    double delta_y = infinity;

    if (dx != 0) {
        double boundary = grid.origin_x + (x + (dx > 0 ? 1 : 0)) * grid.resolution;
        next_x = (boundary - first.x) / dx;
        delta_x = grid.resolution / std::abs(dx);
    }

    if (dy != 0) {
        double boundary = grid.origin_y + (y + (dy > 0 ? 1 : 0)) * grid.resolution;
        next_y = (boundary - first.y) / dy;
        delta_y = grid.resolution / std::abs(dy);
    }

    for (int count = 0; count <= grid.width + grid.height; ++count) {
        if (x == end_x && y == end_y) {
            return true;
        }

        int new_x = x;
        int new_y = y;
        if (std::abs(next_x - next_y) < 1e-12) {
            new_x += step_x;
            new_y += step_y;
            next_x += delta_x;
            next_y += delta_y;
        } else if (next_x < next_y) {
            new_x += step_x;
            next_x += delta_x;
        } else {
            new_y += step_y;
            next_y += delta_y;
        }

        if (!grid.canStep(x, y, new_x, new_y)) {
            return false;
        }

        x = new_x;
        y = new_y;
    }

    return false;
}

inline bool segmentWithReserve(const DijkstraGrid& grid, PlanPoint first, PlanPoint last, double reserve) {
    if (!segmentFree(grid, first, last))
        return false;
    if (reserve <= 0)
        return true;
    for (double x : {-reserve, 0., reserve})
        for (double y : {-reserve, 0., reserve})
            if (!segmentFree(grid, {first.x + x, first.y + y}, {last.x + x, last.y + y}))
                return false;
    return true;
}

inline double availableTrackingReserve(const DijkstraGrid& grid, PlanPoint point, double maximum) {
    if (!segmentFree(grid, point, point))
        return -1.;
    double low = 0, high = maximum;
    for (int i = 0; i < 14; ++i) {
        double middle = (low + high) / 2;

        if (segmentWithReserve(grid, point, point, middle))
            low = middle;
        else
            high = middle;
    }

    return low;
}

struct LocalObstacle {
    PlanPoint centre;
    double radius = 0;
    double speed_bound = 0;
    double hard_radius = -1;
};

struct FollowerConfig {
    double max_speed = 0.6;
    double acceleration = 1.0;
    double braking = 0.7;
    double lateral_acceleration = .9;
    double lookahead = 0.45;
    double horizon = 0.6;
    double tolerance = 0.03;
    double release_tolerance = 0.06;

    double stop_speed = 0.03;
    double settle_time = 0.35;
    double path_deviation = 0.8;
    double response_time = 0.10;
    double tracking_reserve = 0.;
    double feedback_timeout = .2;

    void validate() const {
        double values[] = {max_speed,   acceleration,   braking,           lookahead,
                           horizon,     tolerance,      release_tolerance, stop_speed,
                           settle_time, path_deviation, response_time,     lateral_acceleration};
        for (double value : values) {
            if (!std::isfinite(value) || value <= 0 || value > 10) {
                throw std::invalid_argument("Invalid path follower parameters");
            }
        }

        if (release_tolerance <= tolerance || path_deviation < lookahead) {
            throw std::invalid_argument("Invalid follower tolerance or deviation limit");
        }

        if (!std::isfinite(feedback_timeout) || feedback_timeout <= 0 || feedback_timeout > .3)
            throw std::invalid_argument("Invalid feedback prediction window");
        if (!std::isfinite(tracking_reserve) || tracking_reserve < 0 || tracking_reserve > .1)
            throw std::invalid_argument("Invalid continuous tracking reserve");
    }
};

struct FollowResult {
    std::string status;
    PlanPoint command;
    PlanPoint target;

    std::vector<PlanPoint> trajectory;
    std::string blocking_reason;
};

class PathFollower {
  public:
    explicit PathFollower(const FollowerConfig& config = FollowerConfig()) : config_(config) {
        config_.validate();
    }

    void reset(PlanPoint measured_velocity = {}) {
        command_ = std::isfinite(measured_velocity.x) && std::isfinite(measured_velocity.y)
                       ? boundedVector(measured_velocity, config_.max_speed)
                       : PlanPoint{};
        settled_ = 0;
        arrived_ = false;
    }

    // obstacles 的半径以本次控制时刻为准，试算时还要扩展未来运动的不确定区域。
    bool collisionFree(const DijkstraGrid& grid, const std::vector<LocalObstacle>& obstacles, PlanPoint first,
                       PlanPoint last, double time) const {
        if (!segmentWithReserve(grid, first, last, config_.tracking_reserve)) {
            return false;
        }

        for (const LocalObstacle& obstacle : obstacles) {
            double dx = last.x - first.x;
            double dy = last.y - first.y;
            double square = dx * dx + dy * dy;
            double fraction = 0;

            if (square > 0) {
                fraction = ((obstacle.centre.x - first.x) * dx + (obstacle.centre.y - first.y) * dy) / square;
                fraction = std::clamp(fraction, 0.0, 1.0);
            }

            PlanPoint closest{first.x + fraction * dx, first.y + fraction * dy};

            const double prediction = std::min(time, config_.horizon + config_.response_time + config_.feedback_timeout);
            if (pointDistance(closest, obstacle.centre) <= obstacle.radius + obstacle.speed_bound * prediction) {
                return false;
            }
        }

        return true;
    }

    bool rollout(const DijkstraGrid& grid, const std::vector<LocalObstacle>& obstacles, PlanPoint position,
                 PlanPoint measured_velocity, PlanPoint candidate, std::vector<PlanPoint>& trajectory,
                 double feedback_age = 0., std::optional<PlanPoint> stop_goal = std::nullopt) const {
        if (!std::isfinite(feedback_age) || feedback_age < 0 || feedback_age > config_.feedback_timeout)
            return false;
        trajectory.clear();
        trajectory.push_back(position);
        // 先覆盖驱动响应延迟，再加速到候选速度，最后完整试算制动到零。
        double maximum =
            std::max(std::hypot(measured_velocity.x, measured_velocity.y), std::hypot(candidate.x, candidate.y));

        double horizon = std::hypot(candidate.x, candidate.y) < 1e-6
                             ? 0
                             : (std::min(config_.horizon, .25) + (config_.horizon - std::min(config_.horizon, .25)) *
                                                                     std::clamp((maximum - .3) / .3, 0., 1.));
        double response = config_.response_time + feedback_age;
        double duration = response + horizon + maximum / config_.braking + 0.1;
        double step = std::min(0.04, grid.resolution / (4 * std::max(maximum, 0.01)));
        PlanPoint velocity = measured_velocity;

        for (double time = 0; time < duration; time += step) {
            double interval = std::min(step, duration - time);
            PlanPoint target = candidate;
            double acceleration = config_.acceleration;

            if (time < response) {
                target = measured_velocity;
            } else if (time >= response + horizon) {
                target = {};
                acceleration = config_.braking;
            }

            if(stop_goal && time>=response && time<response+horizon) {
                PlanPoint direction{stop_goal->x-trajectory.front().x,stop_goal->y-trajectory.front().y};
                double length=std::hypot(direction.x,direction.y);
                double remaining=length>1e-6 ? std::max(0.,((stop_goal->x-position.x)*direction.x+
                    (stop_goal->y-position.y)*direction.y)/length) : 0.;

                target=boundedVector(target,std::sqrt(2*(.8*config_.braking)*remaining));
                if(std::hypot(target.x,target.y)<std::hypot(velocity.x,velocity.y)) acceleration=config_.braking;
            }
            PlanPoint change = boundedVector({target.x - velocity.x, target.y - velocity.y}, acceleration * interval);
            velocity.x += change.x;
            velocity.y += change.y;
            PlanPoint next{position.x + velocity.x * interval, position.y + velocity.y * interval};

            if (!collisionFree(grid, obstacles, position, next, std::max(0., time + interval - feedback_age))) {
                return false;
            }

            position = next;
            trajectory.push_back(position);
        }

        return true;
    }

    FollowResult recoverTrackingReserve(const DijkstraGrid& grid, const std::vector<LocalObstacle>& obstacles,
                                        PlanPoint position, PlanPoint velocity, PlanPoint goal, double dt, double age,
                                        double limit) {
        FollowResult result;
        result.status = "LOCAL_BLOCKED";
        result.blocking_reason = "START_TRACKING_RESERVE";
        const double initial = availableTrackingReserve(grid, position, config_.tracking_reserve);

        if (initial < 0 || limit <= 0 || std::hypot(velocity.x, velocity.y) > .15)
            return result;
        auto recovery_config = config_;
        recovery_config.tracking_reserve = 0;
        PathFollower recovery(recovery_config);
        double best = -INFINITY;

        for (int i = 0; i < 16; ++i) {
            double angle = i * 2 * M_PI / 16;
            PlanPoint desired{std::min(.15, limit) * std::cos(angle), std::min(.15, limit) * std::sin(angle)};
            auto change = boundedVector({desired.x - velocity.x, desired.y - velocity.y}, config_.acceleration * dt);
            PlanPoint trial{velocity.x + change.x, velocity.y + change.y};

            PlanPoint immediate{position.x + trial.x * dt, position.y + trial.y * dt};

            if (availableTrackingReserve(grid, immediate, config_.tracking_reserve) + 1e-6 < initial ||
                !segmentWithReserve(grid, position, immediate, std::max(0., initial - 1e-6)))
                continue;

            std::vector<PlanPoint> trajectory;
            if (!recovery.rollout(grid, obstacles, position, velocity, trial, trajectory, age))
                continue;

            constexpr double margin_noise = .0001;
            double margin = initial;
            bool increasing = true;

            for (std::size_t j = 1; j < trajectory.size(); ++j) {
                double next = availableTrackingReserve(grid, trajectory[j], config_.tracking_reserve);
                if (next + margin_noise < margin || next + margin_noise < initial ||
                    !segmentWithReserve(grid, trajectory[j - 1], trajectory[j], std::max(0., margin - margin_noise))) {
                    increasing = false;
                    break;
                }

                margin = std::max(margin, next);
            }

            const double required_gain = std::min(.0005, (config_.tracking_reserve - initial) * .5);
            if (!increasing ||
                (margin < initial + required_gain &&
                 !segmentWithReserve(grid, trajectory.back(), trajectory.back(), config_.tracking_reserve)))
                continue;
            double score = 1000 * (margin - initial) - pointDistance(trajectory.back(), goal);

            if (score > best) {
                best = score;
                result.command = trial;
                result.target = trajectory.back();
                result.trajectory = std::move(trajectory);
                result.status = "CLEARANCE_RECOVERY";
                result.blocking_reason = "RESTORING_TRACKING_RESERVE";
            }
        }

        command_ = result.command;
        return result;
    }

    std::optional<FollowResult> recoverStationaryMargin(const DijkstraGrid& grid,
                                                        const std::vector<LocalObstacle>& obstacles, PlanPoint position,
                                                        PlanPoint velocity, PlanPoint goal, double dt, double age,
                                                        double limit) {
        auto checked = obstacles;
        std::vector<std::size_t> recovering;
        for (std::size_t i = 0; i < obstacles.size(); ++i) {
            const auto& o = obstacles[i];
            double distance = pointDistance(position, o.centre);

            if (o.speed_bound == 0 && o.hard_radius > 0 && o.hard_radius < o.radius && distance > o.hard_radius &&
                distance <= o.radius) {
                recovering.push_back(i);
                checked[i].radius = o.hard_radius;
            }
        }

        if (recovering.empty())
            return {};
        FollowResult result;
        result.status = "LOCAL_BLOCKED";
        result.blocking_reason = "STATIONARY_MARGIN_NO_ESCAPE";

        if (std::hypot(velocity.x, velocity.y) > .15 || limit <= 0)
            return result;
        double best = -INFINITY;
        for (int i = 0; i < 32; ++i) {
            double angle = i * 2 * M_PI / 32;
            PlanPoint desired{std::min(.15, limit) * std::cos(angle), std::min(.15, limit) * std::sin(angle)};
            auto change = boundedVector({desired.x - velocity.x, desired.y - velocity.y}, config_.acceleration * dt);
            PlanPoint trial{velocity.x + change.x, velocity.y + change.y};
            PlanPoint immediate{position.x + trial.x * dt, position.y + trial.y * dt};

            if (!collisionFree(grid, checked, position, immediate, dt))
                continue;

            std::vector<PlanPoint> trajectory;
            if (!rollout(grid, checked, position, velocity, trial, trajectory, age))
                continue;
            bool outward = true;
            double gain = 0;

            for (auto j : recovering) {
                const auto& o = obstacles[j];
                double initial = pointDistance(position, o.centre), previous = initial;
                if (pointDistance(immediate, o.centre) < initial) {
                    outward = false;
                    break;
                }

                for (auto p : trajectory) {
                    double distance = pointDistance(p, o.centre);

                    if (distance + 1e-4 < previous || distance + 1e-4 < initial) {
                        outward = false;
                        break;
                    }

                    previous = std::max(previous, distance);
                }

                double increase = pointDistance(trajectory.back(), o.centre) - initial;
                if (increase < .0001)
                    outward = false;
                gain += increase;
            }

            if (!outward)
                continue;
            double score = 10 * gain - pointDistance(trajectory.back(), goal);
            if (score > best) {
                best = score;
                result.command = trial;
                result.target = trajectory.back();
                result.trajectory = std::move(trajectory);
                result.status = "STATIONARY_MARGIN_RECOVERY";
                result.blocking_reason = "EXIT_STATIONARY_UNCERTAINTY";
            }
        }

        command_ = result.command;
        return result;
    }

    // A moving body or a newly aged broadcast can cover the current pose with
    // its planning margin. Replanning cannot repair a start that is inside that
    // margin. Permit a slow, strictly outward escape, never a path through the
    // body's physical footprint or through the static map. Only the initially
    // overlapping margins are relaxed; all other obstacles retain full rollout.
    std::optional<FollowResult> recoverEnemyMargin(const DijkstraGrid& grid,
            const std::vector<LocalObstacle>& obstacles, PlanPoint position, PlanPoint velocity,
            PlanPoint goal, double dt, double age, double margin) const {
        if (!grid.valid() || !std::isfinite(margin) || margin < 0 || margin > .3 ||
            !std::isfinite(dt) || dt <= 0 || dt > .2 || !std::isfinite(age) || age < 0 ||
            age > config_.feedback_timeout || !std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(velocity.x) || !std::isfinite(velocity.y) ||
            std::hypot(velocity.x,velocity.y) > .4 ||
            !segmentWithReserve(grid,position,position,config_.tracking_reserve)) return {};
        auto checked=obstacles;
        std::vector<std::size_t> escaping;
        for(std::size_t i=0;i<obstacles.size();++i) {
            const auto& o=obstacles[i];
            if (!std::isfinite(o.radius) || !std::isfinite(o.hard_radius) ||
                !std::isfinite(o.centre.x) || !std::isfinite(o.centre.y) ||
                !std::isfinite(o.speed_bound) || o.radius <= 0 || o.hard_radius <= margin ||
                o.hard_radius>o.radius || o.speed_bound < 0)
                return {};
            const double d=pointDistance(position,o.centre);
            // Match the planner's CURRENT hard-block raster, not a future
            // reachable area. A distant moving target must never make normal
            // approach reverse into an "escape". Non-overlapping obstacles
            // keep their complete uncertainty/speed envelope in rollout.
            if (d>o.hard_radius+std::sqrt(2.)*grid.resolution) continue;
            const double core=o.hard_radius-margin+std::max(.02,config_.tracking_reserve);
            if(d<=core) return {}; // Never authorize an escape from the physical body itself.
            checked[i].radius=core;
            checked[i].speed_bound=0;
            escaping.push_back(i);
        }
        if(escaping.empty())return {};
        FollowResult best;best.status="ENEMY_MARGIN_RECOVERY";best.blocking_reason="OUTWARD_ENEMY_MARGIN_ESCAPE";
        double best_score=-INFINITY;
        for(int direction=0;direction<32;++direction) for(double speed:{.35,.20,.10}) {
            const double a=direction*2*M_PI/32;
            auto dv=boundedVector({speed*std::cos(a)-velocity.x,speed*std::sin(a)-velocity.y},config_.acceleration*dt);
            PlanPoint trial{velocity.x+dv.x,velocity.y+dv.y};
            trial=boundedVector(trial,std::min(.35,config_.max_speed));
            std::vector<PlanPoint> trajectory;
            if(!rollout(grid,checked,position,velocity,trial,trajectory,age))continue;
            bool outward=true;double gain=0;
            for(auto i:escaping) {
                double initial=pointDistance(position,obstacles[i].centre),previous=initial;
                for(auto p:trajectory) {
                    double d=pointDistance(p,obstacles[i].centre);
                    if(d+1e-5<previous) {outward=false;break;}
                    previous=std::max(previous,d);
                }
                double increase=pointDistance(trajectory.back(),obstacles[i].centre)-initial;
                if(!outward || increase<.001) {outward=false;break;}
                gain+=increase;
            }
            if(!outward)continue;
            double score=10*gain;
            if(std::isfinite(goal.x)&&std::isfinite(goal.y))score-=pointDistance(trajectory.back(),goal);
            if(score>best_score) {
                best_score=score;best.command=trial;best.trajectory=std::move(trajectory);best.target=best.trajectory.back();
            }
        }
        return std::isfinite(best_score)?std::optional<FollowResult>(std::move(best)):std::nullopt;
    }

    FollowResult update(const DijkstraGrid& grid, const std::vector<PlanPoint>& path,
                        const std::vector<LocalObstacle>& obstacles, PlanPoint position, PlanPoint velocity,
                        PlanPoint goal, double dt, double feedback_dt, double speed_limit = INFINITY,
                        double pursuit_limit = INFINITY, double feedback_age = 0., double goal_tolerance = INFINITY) {
        FollowResult result;
        result.status = "WAIT_PATH";
        if (!grid.valid() || path.empty() || !std::isfinite(dt) || dt <= 0 || dt > 0.2 || !std::isfinite(position.x) ||
            !std::isfinite(position.y) || !std::isfinite(velocity.x) || !std::isfinite(velocity.y) ||
            !std::isfinite(feedback_age) || feedback_age < 0 || feedback_age > config_.feedback_timeout) {
            reset();
            return result;
        }

        double distance = pointDistance(position, goal);
        const double tolerance = std::isfinite(goal_tolerance) && goal_tolerance>0
            ? std::min(config_.tolerance,goal_tolerance) : config_.tolerance;
        speed_limit = std::isnan(speed_limit) ? 0 : std::clamp(speed_limit, 0.0, config_.max_speed);
        double speed = std::hypot(velocity.x, velocity.y);

        if (auto recovery =
                recoverStationaryMargin(grid, obstacles, position, velocity, goal, dt, feedback_age, speed_limit))
            return *recovery;
        if (!segmentFree(grid, position, position)) {
            reset();
            result.status = "LOCAL_BLOCKED";
            result.blocking_reason = "START_IN_INFLATED_OBSTACLE";
            return result;
        }

        if (!segmentWithReserve(grid, position, position, config_.tracking_reserve)) {
            return recoverTrackingReserve(grid, obstacles, position, velocity, goal, dt, feedback_age, speed_limit);
        }

        if (arrived_ && distance <= config_.release_tolerance && speed <= config_.stop_speed) {
            result.status = "ARRIVED";
            return result;
        }

        arrived_ = false;
        PlanPoint control_position{position.x + velocity.x * feedback_age, position.y + velocity.y * feedback_age};
        std::size_t nearest = 0;
        double deviation = INFINITY;

        for (std::size_t i = 0; i < path.size(); ++i) {
            double current = pointDistance(control_position, path[i]);
            if (current < deviation) {
                deviation = current;
                nearest = i;
            }
        }

        if (deviation > config_.path_deviation) {
            reset();
            result.status = "PATH_DEVIATION";
            return result;
        }

        speed_limit =
            std::min(speed_limit, curvatureSpeedLimit(path,nearest,config_.max_speed,config_.lateral_acceleration));

        double lookahead = std::min(config_.lookahead, std::max(grid.resolution, pursuit_limit));
        PlanPoint near_chord{}, far_chord{};
        double preview = 0;

        for (std::size_t i = nearest + 1; i < path.size(); ++i) {
            preview += pointDistance(path[i - 1], path[i]);
            far_chord = {path[i].x - path[nearest].x, path[i].y - path[nearest].y};
            if (preview <= .3)
                near_chord = far_chord;

            if (preview >= lookahead)
                break;
        }

        double chord_product = std::hypot(near_chord.x, near_chord.y) * std::hypot(far_chord.x, far_chord.y);
        if (chord_product > 1e-6 && (near_chord.x * far_chord.x + near_chord.y * far_chord.y) / chord_product < .866) {
            lookahead = std::min(lookahead, std::max(.35, speed_limit * .5));
        }

        result.target = path[nearest];
        double along = 0;
        for (std::size_t i = nearest + 1; i < path.size(); ++i) {
            along += pointDistance(path[i - 1], path[i]);

            if (along > lookahead || !segmentWithReserve(grid, control_position, path[i], config_.tracking_reserve)) {
                break;
            }

            result.target = path[i];
        }

        if (pointDistance(control_position, goal) <= config_.lookahead &&
            segmentWithReserve(grid, control_position, goal, config_.tracking_reserve)) {
            result.target = goal;
        }

        const bool terminal=pointDistance(result.target,goal)<1e-6 &&
            segmentWithReserve(grid,control_position,goal,config_.tracking_reserve);
        const double gain=terminal ? 4.0 : 2.4;
        PlanPoint desired{gain * (result.target.x - control_position.x) - 0.25 * velocity.x,
                          gain * (result.target.y - control_position.y) - 0.25 * velocity.y};

        double remaining = pointDistance(control_position, goal);
        if(!terminal) {
            remaining=pointDistance(control_position,path[nearest]);
            for(size_t i=nearest+1;i<path.size();++i) remaining+=pointDistance(path[i-1],path[i]);
            remaining+=pointDistance(path.back(),goal);
        }

        const double approach_braking=.8*config_.braking;

        double lag_brake = approach_braking * (config_.response_time + feedback_age);
        double brake_speed = std::sqrt(lag_brake * lag_brake + 2 * approach_braking * remaining) - lag_brake;
        desired = boundedVector(desired, std::min(speed_limit, brake_speed));

        if (distance <= tolerance) {
            desired = {};
            result.status = "SETTLING";
            if (speed <= config_.stop_speed) {
                settled_ += std::max(0.0, feedback_dt);
            } else {
                settled_ = 0;
            }

            if (settled_ >= config_.settle_time) {
                reset();
                arrived_ = true;
                result.status = "ARRIVED";
                return result;
            }
        } else {
            settled_ = 0;
            result.status = "FOLLOWING";
        }

        double best_score = INFINITY;
        // 全向底盘：尝试前进、左右绕行和减速，无须先转底盘朝向。
        double angles[] = {0, 0.35, -0.35, 0.7, -0.7, 1.2, -1.2, 1.57, -1.57};
        double factors[] = {1.0, 0.65, 0.3, 0.0};

        for (double angle : angles) {
            for (double factor : factors) {
                PlanPoint requested{factor * (desired.x * std::cos(angle) - desired.y * std::sin(angle)),
                                    factor * (desired.x * std::sin(angle) + desired.y * std::cos(angle))};
                PlanPoint change =
                    boundedVector({requested.x - command_.x, requested.y - command_.y}, config_.acceleration * dt);
                PlanPoint candidate{command_.x + change.x, command_.y + change.y};
                candidate = boundedVector(candidate, speed_limit);

                std::vector<PlanPoint> trajectory;
                if (!rollout(grid, obstacles, position, velocity, candidate, trajectory, feedback_age,
                             terminal ? std::optional<PlanPoint>(goal) : std::nullopt)) {
                    continue;
                }

                PlanPoint end = trajectory.back();
                // The rollout includes complete braking, so its endpoint can
                // legitimately lie beyond the lookahead point. Penalizing that
                // overshoot made an unobstructed 1.2 m/s cruise settle at 0.78.
                // Prefer staying on the route, rather than braking at every
                // temporary lookahead point; collision checks still cover all
                // intervening motion and the final stop.
                double route_distance = INFINITY;
                for (std::size_t i = nearest; i < path.size(); ++i)
                    route_distance = std::min(route_distance, pointDistance(end, path[i]));
                double score = pointDistance(candidate, desired) + 0.5 * route_distance;

                if (score < best_score) {
                    best_score = score;
                    result.command = candidate;
                    result.trajectory = trajectory;
                }
            }
        }

        if (!std::isfinite(best_score)) {
            command_ = {};
            result.status = "LOCAL_BLOCKED";
            result.command = {};
            result.blocking_reason = "NO_COLLISION_FREE_ROLLOUT";
            return result;
        }

        command_ = result.command;
        if (std::hypot(command_.x, command_.y) < 1e-6 && distance > tolerance) {
            result.status = "WAIT_OBSTACLE";
        }

        return result;
    }

  private:
    FollowerConfig config_;
    PlanPoint command_;
    double settled_ = 0;
    bool arrived_ = false;
};
} // namespace campus
