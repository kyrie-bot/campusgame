#pragma once
#include "geometry/aim_geometry.hpp"

namespace campus {
struct BallisticSolution {
    bool valid = false;
    double yaw = 0;
    double pitch = 0;
    double flight_time = 0;
};//保存弹道解算结果

inline BallisticSolution solveBallistics(const cv::Vec3d& delta, double speed, double gravity = 9.81) {
    BallisticSolution result;
    double distance = std::hypot(delta[0], delta[1]);
    if (!finite(delta) || !std::isfinite(speed) || speed <= 0 || distance < 1e-4 || gravity < 0 ||
        !std::isfinite(gravity)) {
        return result;  x(10) = 0
    }

    double angle;
    if (gravity < 1e-6) {
        angle = std::atan2(delta[2], distance);
    } else {
        double v2 = speed * speed;
        double discriminant = v2 * v2 - gravity * (gravity * distance * distance + 2 * delta[2] * v2);
        if (discriminant < 0) {
            return result;
        }

        double tangent =
            (gravity * distance * distance + 2 * delta[2] * v2) / (distance * (v2 + std::sqrt(discriminant)));
        angle = std::atan(tangent);
    }
    result.yaw = std::atan2(-delta[1], delta[0]) * 180 / CV_PI;
    result.pitch = -angle * 180 / CV_PI;
    result.flight_time = distance / (speed * std::cos(angle));
    result.valid = std::isfinite(result.flight_time) && result.flight_time > 0 && std::abs(result.pitch) <= 60;
    return result;
}

struct TargetMotion {
    cv::Vec3d center{0, 0, 0};
    cv::Vec3d velocity{0, 0, 0};
    double yaw = 0;
    double yaw_rate = 0;
    double radius = 0.26;
    double radius_odd = 0.26;
    double dz = 0;
    double dz_second = 0;
    int count = 4;

    double angle(int slot, double t) const {
        return wrap(yaw + yaw_rate * t + slot * 2 * CV_PI / count);
    }

    cv::Vec3d position(int slot, double t) const {
        double a = angle(slot, t);
        double r = radius;
        if (count == 1) {
            r = 0;
        } else if (count == 4 && slot % 2) {
            r = radius_odd;
        }

        double height_offset = 0;
        if (count == 4 && slot % 2) {
            height_offset = dz;
        } else if (count == 3) {
            if (slot == 1) {
                height_offset = dz;
            } else if (slot == 2) {
                height_offset = dz_second;
            }
        }

        return center + velocity * t + cv::Vec3d(-r * std::cos(a), -r * std::sin(a), height_offset);
    }
};

struct InterceptSolution {
    BallisticSolution ballistic;
    cv::Vec3d point{0, 0, 0};
    cv::Vec3d origin{0, 0, 0};
    double horizon = 0;
    int slot = -1;
    double score = INFINITY;
    double facing = 0;
};//预测拦截结果

//计算对指定装甲板的预测拦截
inline InterceptSolution interceptSlot(const TargetMotion& target, int slot, const cv::Vec3d& origin, double speed,
                                       double age_and_latency, double gravity = 9.81, double max_horizon = 1.5,
                                       double min_facing = 0.2) {
    InterceptSolution result;
    if (target.count < 1 || target.count > 4 || slot < 0 || slot >= target.count || !finite(target.center) ||
        !finite(target.velocity) || !finite(origin) || !std::isfinite(target.yaw) || !std::isfinite(target.yaw_rate) ||
        !std::isfinite(target.radius) || !std::isfinite(target.radius_odd) || !std::isfinite(target.dz) ||
        !std::isfinite(target.dz_second) || target.radius < 0 || target.radius_odd < 0 ||
        !std::isfinite(age_and_latency) || age_and_latency < 0 || !std::isfinite(max_horizon) ||
        max_horizon <= age_and_latency || !std::isfinite(min_facing) || min_facing < 0 || min_facing > 1) {
        return result;
    }

    double horizon = age_and_latency;
    BallisticSolution b;
    bool converged = false;

    for (int n = 0; n < 20; ++n) {
        b = solveBallistics(target.position(slot, horizon) - origin, speed, gravity);

        if (!b.valid || age_and_latency + b.flight_time > max_horizon) {
            break;
        }

        double next = age_and_latency + b.flight_time;
        if (std::abs(next - horizon) < 1e-5) {
            converged = true;
            horizon = next;
            break;
        }
        horizon = 0.5 * (horizon + next);
    }

    if (!converged) {
        return result;
    }

    cv::Vec3d point = target.position(slot, horizon);
    cv::Vec3d direction = point - origin;
    double a = target.angle(slot, horizon);
    double distance = cv::norm(direction);
    double facing = direction.dot(cv::Vec3d(std::cos(a), std::sin(a), 0)) / distance;
    if (!std::isfinite(facing) || facing < min_facing) {
        return result;
    }
    b = solveBallistics(direction, speed, gravity);

    if (!b.valid || age_and_latency + b.flight_time > max_horizon) {
        return result;
    }
    result.ballistic = b;
    result.point = point;
    result.origin = origin;
    result.horizon = horizon;
    result.slot = slot;
    result.score = distance + 0.8 * (1 - facing);
    result.facing = facing;
    return result;
}

inline InterceptSolution intercept(const TargetMotion& target, const cv::Vec3d& origin, double speed,
                                   double age_and_latency, double gravity = 9.81, double max_horizon = 1.5,
                                   int preferred_slot = -1, double switch_margin = 0, double min_facing = 0.2,
                                   unsigned allowed_slots = 0xf) {
    InterceptSolution best;
    InterceptSolution preferred;
    if (target.count < 1 || target.count > 4 || !std::isfinite(switch_margin) || switch_margin < 0) {
        return best;
    }

    for (int slot = 0; slot < target.count; ++slot) {
        if (!(allowed_slots & (1u << slot))) continue;
        InterceptSolution candidate =
            interceptSlot(target, slot, origin, speed, age_and_latency, gravity, max_horizon, min_facing);

        if (!candidate.ballistic.valid) {
            continue;
        }

        if (slot == preferred_slot) {
            preferred = candidate;
        }

        if (candidate.score < best.score) {
            best = candidate;
        }
    }

    if (preferred.ballistic.valid && preferred.score <= best.score + switch_margin) {
        return preferred;
    }

    return best;
}

inline InterceptSolution interceptMovingShooter(const TargetMotion& target, const cv::Vec3d& pivot,
                                                const cv::Vec3d& shooter_velocity,
                                                const cv::Vec3d& optical_muzzle_offset, double yaw, double pitch,
                                                double speed, double delay, double gravity = 9.81,
                                                double max_horizon = 1.5, int preferred_slot = -1,
                                                double switch_margin = 0, double min_facing = 0.2,
                                                unsigned allowed_slots = 0xf) {
    InterceptSolution invalid;
    if (!finite(pivot) || !finite(shooter_velocity) || !finite(optical_muzzle_offset) || !std::isfinite(yaw) ||
        !std::isfinite(pitch) || !std::isfinite(delay) || delay < 0) {
        return invalid;
    }

    cv::Vec3d future_pivot = pivot + shooter_velocity * delay;

    for (int iteration = 0; iteration < 10; ++iteration) {
        CameraGeometry barrel(future_pivot, yaw, pitch, 1.0, optical_muzzle_offset);
        InterceptSolution result = intercept(target, barrel.origin, speed, delay, gravity, max_horizon, preferred_slot,
                                             switch_margin, min_facing, allowed_slots);

        if (!result.ballistic.valid) {
            return invalid;
        }
        CameraGeometry aimed_barrel(future_pivot, result.ballistic.yaw, result.ballistic.pitch, 1.0,
                                    optical_muzzle_offset);
        if (cv::norm(aimed_barrel.origin - result.origin) < 1e-6) {
            return result;
        }
        yaw = result.ballistic.yaw;
        pitch = result.ballistic.pitch;
    }

    return invalid;
}

} // namespace campus
