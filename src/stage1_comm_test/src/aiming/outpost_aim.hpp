#pragma once
#include "aiming/ballistics.hpp"

namespace campus {
inline InterceptSolution interceptOutpostPassage(const TargetMotion& target, int slot, const cv::Vec3d& pivot,
                                                 const cv::Vec3d& shooter_velocity, const cv::Vec3d& muzzle_offset,
                                                 double yaw, double pitch, double speed, double delay,
                                                 double gravity = 9.81, double max_horizon = 1.5) {
    InterceptSolution invalid;
    if (target.count != 3 || slot < 0 || slot >= 3 || !finite(target.center) || !finite(target.velocity) ||
        cv::norm(target.velocity) > 0.05 || !std::isfinite(target.radius) || target.radius <= 0 ||
        !std::isfinite(target.yaw) || !std::isfinite(target.yaw_rate) || !std::isfinite(target.dz) ||
        !std::isfinite(target.dz_second) || !finite(pivot) || !finite(shooter_velocity)) {
        return invalid;
    }

    cv::Vec3d future_pivot = pivot + shooter_velocity * delay;
    double front_yaw = std::atan2(target.center[1] - future_pivot[1], target.center[0] - future_pivot[0]);
    TargetMotion fixed;
    fixed.count = 1;
    fixed.yaw = front_yaw;
    fixed.center =
        target.center - cv::Vec3d(target.radius * std::cos(front_yaw), target.radius * std::sin(front_yaw), 0);
    if (slot == 1) {
        fixed.center[2] += target.dz;
    } else if (slot == 2) {
        fixed.center[2] += target.dz_second;
    }
    InterceptSolution result = interceptMovingShooter(fixed, pivot, shooter_velocity, muzzle_offset, yaw, pitch, speed,
                                                      delay, gravity, max_horizon);

    if (!result.ballistic.valid) {
        return invalid;
    }
    result.slot = slot;
    result.facing = std::cos(wrap(target.angle(slot, result.horizon) - front_yaw));
    return result;
}//前哨站经过指定位置时的拦截解

inline bool insideOutpostPassage(const TargetMotion& target, const InterceptSolution& aim, double width, double height,
                                 double tilt, double phase_sigma = 0) {
    if (!aim.ballistic.valid || target.count != 3 || aim.slot < 0 || aim.slot >= 3 || !std::isfinite(width) ||
        !std::isfinite(height) || !std::isfinite(tilt) || !std::isfinite(phase_sigma) || phase_sigma < 0 ||
        !finite(aim.point) || !std::isfinite(aim.horizon) || aim.horizon < 0) {
        return false;
    }

    cv::Vec3d armor = target.position(aim.slot, aim.horizon);
    if (!finite(armor)) {
        return false;
    }

    double front_yaw = std::atan2(target.center[1] - aim.point[1], target.center[0] - aim.point[0]);
    double facing = std::cos(wrap(target.angle(aim.slot, aim.horizon) - front_yaw));
    if (facing <= 0.5) {
        return false;
    }

    double half_width = width * 0.5 * facing - 0.0085 - target.radius * phase_sigma;
    double half_height = height * 0.5 * std::abs(std::cos(tilt)) - 0.0085;
    if (half_width <= 0 || half_height <= 0) {
        return false;
    }

    cv::Vec3d error = armor - aim.point;
    double horizontal = -error[0] * std::sin(front_yaw) + error[1] * std::cos(front_yaw);
    double horizontal_ratio = horizontal / half_width;
    double vertical_ratio = error[2] / half_height;
    return horizontal_ratio * horizontal_ratio + vertical_ratio * vertical_ratio < 1;
}//判断当前预测结果是否处于合适的装甲板通过窗口

inline InterceptSolution selectOutpostPassage(const TargetMotion& target, const std::array<bool, 4>& seen,
                                              int preferred_slot, bool multiple_heights, double width,
                                              const cv::Vec3d& pivot, const cv::Vec3d& shooter_velocity,
                                              const cv::Vec3d& muzzle_offset, double yaw, double pitch, double speed,
                                              double delay, double gravity = 9.81, double max_horizon = 1.5) {
    InterceptSolution best;

    if (!std::isfinite(target.yaw_rate) || std::abs(target.yaw_rate) < 0.01 || !std::isfinite(width) || width <= 0 ||
        !std::isfinite(target.radius) || target.radius <= 0) {
        return best;
    }

    double direction = target.yaw_rate > 0 ? 1 : -1;
    double exit_phase = std::atan2(std::max(0.0, width * 0.5 - 0.0085), target.radius) + 0.03;
    double best_wait = INFINITY;

    for (int slot = 0; slot < 3; ++slot) {
        if (!seen[slot] || (!multiple_heights && preferred_slot >= 0 && slot != preferred_slot)) {
            continue;
        }
        InterceptSolution candidate = interceptOutpostPassage(target, slot, pivot, shooter_velocity, muzzle_offset, yaw,
                                                              pitch, speed, delay, gravity, max_horizon);

        if (!candidate.ballistic.valid) {
            continue;
        }

        double front = std::atan2(target.center[1] - candidate.point[1], target.center[0] - candidate.point[0]);
        double phase = wrap(target.angle(slot, candidate.horizon) - front) * direction;
        if (slot == preferred_slot && std::abs(phase) <= exit_phase) {
            return candidate;
        }

        double wait = -phase;
        if (phase > exit_phase) {
            wait += 2 * CV_PI;
        }

        if (wait < best_wait) {
            best_wait = wait;
            best = candidate;
        }
    }

    return best;
}//选择拦截机会

inline double predictedPhaseVariance(double yaw_variance, double rate_variance, double covariance, double horizon,
                                     double acceleration_noise) {
    if (!std::isfinite(yaw_variance) || yaw_variance < 0 || !std::isfinite(rate_variance) || rate_variance < 0 ||
        !std::isfinite(covariance) || !std::isfinite(horizon) || horizon < 0 || !std::isfinite(acceleration_noise) ||
        acceleration_noise < 0 || covariance * covariance > yaw_variance * rate_variance + 1e-9) {
        return INFINITY;
    }

    return std::max(0.0, yaw_variance + 2 * horizon * covariance + horizon * horizon * rate_variance +
                             acceleration_noise * horizon * horizon * horizon / 3);
}//估计未来旋转相位的不确定性
} // namespace campus