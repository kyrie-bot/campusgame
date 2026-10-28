#pragma once
#include "aiming/ballistics.hpp"
#include <array>
#include <cstdint>

namespace campus {
inline constexpr double kMobilePlateEdgeMargin = .006;
inline constexpr double kMobileExecutionAge = .035;
class MobileFireContinuation {
public:
    void clear() {
        id_ = 0;
        slot_ = -1;
        time_ = -INFINITY;
    }

    void observedTrack(uint32_t id, int slot, double time) {
        id_ = id;
        slot_ = slot;
        time_ = time;
    }

    bool allows(uint32_t id, int slot, double time, double missing, double observation_age, double limit) const {
        return id && id == id_ && slot >= 0 && slot == slot_ && std::isfinite(time) && std::isfinite(missing) &&
               std::isfinite(observation_age) && std::isfinite(limit) && limit > 0 && limit <= .25 && time >= time_ &&
               time - time_ <= limit && missing >= 0 && missing <= limit && observation_age >= 0 &&
               observation_age <= .2 + limit;
    }//判断短时间是否允许射击

private:
    uint32_t id_ = 0;
    int slot_ = -1;
    double time_ = -INFINITY;
};//允许短时间沿用最近的有效移动目标观测

//计算预测装甲板位置的不确定性
inline std::array<double, 2> predictedPlateAxisVariances(const TargetMotion& motion, int slot, double horizon,
                                     const std::array<double, 144>& covariance, const cv::Vec3d& origin = {0, 0, 0}) {
    if (slot < 0 || slot >= motion.count || !std::isfinite(horizon) || horizon < 0)
        return {INFINITY, INFINITY};

    for (double v : covariance)
        if (!std::isfinite(v))
            return {INFINITY, INFINITY};
    double j[3][12]{};
    j[0][0] = j[1][2] = j[2][4] = 1.;
    j[0][1] = j[1][3] = j[2][5] = horizon;
    double a = motion.angle(slot, horizon);
    int radius_index = motion.count == 4 && slot % 2 ? 9 : 8;
    double radius = motion.count == 1 ? 0. : (radius_index == 9 ? motion.radius_odd : motion.radius);
    j[0][6] = radius * std::sin(a);
    j[1][6] = -radius * std::cos(a);
    j[0][7] = j[0][6] * horizon;
    j[1][7] = j[1][6] * horizon;
    if (motion.count != 1) {
        j[0][radius_index] = -std::cos(a);
        j[1][radius_index] = -std::sin(a);
    }

    if (motion.count == 4 && slot % 2)
        j[2][10] = 1.;
    else if (motion.count == 3 && slot > 0)
        j[2][9 + slot] = 1.;

    cv::Vec3d ray = motion.position(slot, horizon) - origin;
    double range = cv::norm(ray), ground = std::hypot(ray[0], ray[1]);
    if (!std::isfinite(range) || range < 1e-6 || ground < 1e-6)
        return {INFINITY, INFINITY};
    double tangents[2][3] = {
        {-ray[1] / ground, ray[0] / ground, 0},
        {-ray[0] * ray[2] / (range * ground), -ray[1] * ray[2] / (range * ground), ground / range}};
    std::array<double, 2> result{};

    for (int axis = 0; axis < 2; ++axis) {
        double projected[12]{};

        for (int col = 0; col < 12; ++col)
            for (int row = 0; row < 3; ++row)
                projected[col] += tangents[axis][row] * j[row][col];
        double v = 0.;

        for (int row = 0; row < 12; ++row)
            for (int col = 0; col < 12; ++col)
                v += projected[row] * covariance[row * 12 + col] * projected[col];
        if (!std::isfinite(v) || v < -.000000001)
            return {INFINITY, INFINITY};
        result[axis] = std::max(0., v);
    }

    return result;
}

inline double predictedPlateVariance(const TargetMotion& motion, int slot, double horizon,
                                     const std::array<double, 144>& covariance, const cv::Vec3d& origin = {0, 0, 0}) {
    const auto v = predictedPlateAxisVariances(motion, slot, horizon, covariance, origin);
    return std::max(v[0], v[1]) + std::pow(.25 * horizon * horizon, 2);
}

inline bool firePlateObserved(bool selected_observed, bool any_observed, bool geometry_observable,
                              bool require_current_plate) {
    if (!any_observed) {
        return false;
    }

    if (selected_observed) {
        return true;
    }

    return !require_current_plate && geometry_observable;
}//判断装甲板观测状态是否满足射击要求

inline bool insideFireWindow(double yaw_error, double pitch_error, double distance, double facing, double width,
                             double height, double tilt, double max_angle_deg) {
    double values[] = {yaw_error, pitch_error, distance, facing, width, height, tilt, max_angle_deg};

    for (double value : values) {
        if (!std::isfinite(value)) {
            return false;
        }
    }

    if (distance <= 0 || width <= 0 || height <= 0 || facing < 0.2 || max_angle_deg <= 0) {
        return false;
    }

    double half_width = width * 0.5 * std::min(facing, 1.0) - 0.0085;
    double half_height = height * 0.5 * std::abs(std::cos(tilt)) - 0.0085;
    if (half_width <= 0 || half_height <= 0) {
        return false;
    }

    double yaw_limit = std::min(max_angle_deg, std::atan2(half_width, distance) * 180 / CV_PI);
    double pitch_limit = std::min(max_angle_deg, std::atan2(half_height, distance) * 180 / CV_PI);
    double yaw_ratio = yaw_error / yaw_limit;
    double pitch_ratio = pitch_error / pitch_limit;
    return yaw_ratio * yaw_ratio + pitch_ratio * pitch_ratio < 1.0;
}

struct MobileFireWindow {
    double yaw_deg = 0, pitch_deg = 0;

    bool contains(double yaw, double pitch) const {
        return std::isfinite(yaw) && std::isfinite(pitch) && std::isfinite(yaw_deg) &&
               std::isfinite(pitch_deg) && yaw_deg > 0 && pitch_deg > 0 &&
               std::abs(yaw) < yaw_deg && std::abs(pitch) < pitch_deg;
    }
};

inline MobileFireWindow mobileFireWindow(double distance, double facing, double width, double height,
                                         double tilt, double cap, double scale) {
    for (double v : {distance, facing, width, height, tilt, cap, scale})
        if (!std::isfinite(v)) return {};
    if (distance <= 0 || facing < .2 || width <= 0 || height <= 0 || cap <= 0 || scale <= 0) return {};
    // A rectangular plate permits corners, but never enlarge the physical plate.
    scale = std::min(scale, 1.);
    double half_width = .5 * width * std::min(facing, 1.) * scale - kMobilePlateEdgeMargin;
    double half_height = .5 * height * std::abs(std::cos(tilt)) * scale - kMobilePlateEdgeMargin;
    if (half_width <= 0 || half_height <= 0) return {};
    return {std::min(cap, std::atan2(half_width, distance) * 180 / CV_PI),
            std::min(cap, std::atan2(half_height, distance) * 180 / CV_PI)};
}

inline bool insideMobileFireWindow(double yaw, double pitch, double distance, double facing,
                                  double width, double height, double tilt, double cap, double scale) {
    return mobileFireWindow(distance, facing, width, height, tilt, cap, scale).contains(yaw, pitch);
}

inline bool mobileCoastConfident(const std::array<double, 2>& variance, double configured, double facing,
                                 double width, double height, double tilt, double horizon, double missing) {
    for (double v : {configured, facing, width, height, tilt, horizon, missing, variance[0], variance[1]})
        if (!std::isfinite(v)) return false;
    if (configured <= 0 || facing < .2 || width <= 0 || height <= 0 || horizon < 0 ||
        missing < 0 || missing > horizon || variance[0] < 0 || variance[1] < 0) return false;
    const double limits[] = {std::min(configured, .5*width*std::min(facing,1.)-kMobilePlateEdgeMargin),
                             std::min(configured, .5*height*std::abs(std::cos(tilt))-kMobilePlateEdgeMargin)};
    // Bound the extra acceleration error accrued through the observation gap.
    // Keep horizontal and vertical uncertainty separate: yaw uncertainty must
    // not be compared against the much shorter vertical plate dimension.
    const double extra = std::pow(.25*(std::pow(horizon+missing,2)-horizon*horizon),2);
    return limits[0] > 0 && limits[1] > 0 && variance[0]+extra <= limits[0]*limits[0] &&
           variance[1]+extra <= limits[1]*limits[1];
}

inline bool mobileExecutionAligned(const MobileFireWindow& window, double desired_yaw, double desired_pitch,
                                    double command_yaw, double command_pitch, double actual_yaw,
                                    double actual_pitch, double evaluation_age) {
    return std::isfinite(evaluation_age) && evaluation_age >= -.005 && evaluation_age <= kMobileExecutionAge &&
           window.contains(std::remainder(desired_yaw - actual_yaw, 360.), desired_pitch - actual_pitch) &&
           window.contains(std::remainder(command_yaw - actual_yaw, 360.), command_pitch - actual_pitch);
}

// Intersect the actual ballistic ray with the translating/rotating plate, rather
// than assuming yaw and pitch errors map to two independent image axes.
inline bool mobileBallisticWindow(const TargetMotion& target, const InterceptSolution& aim, double tilt,
                                  double width, double height, const cv::Vec3d& muzzle_offset,
                                  double yaw, double pitch, double speed, double gravity) {
    if (!aim.ballistic.valid || aim.slot < 0 || aim.slot >= target.count || !finite(aim.origin) ||
        !finite(muzzle_offset) || !std::isfinite(yaw) || !std::isfinite(pitch) ||
        !std::isfinite(tilt) || !std::isfinite(width) || !std::isfinite(height) ||
        !std::isfinite(speed) || speed <= 0 || !std::isfinite(gravity) || gravity < 0 ||
        !std::isfinite(aim.horizon) || !std::isfinite(aim.ballistic.flight_time) ||
        aim.ballistic.flight_time <= 0 || width <= .017 || height <= .017) return false;
    CameraGeometry desired({}, aim.ballistic.yaw, aim.ballistic.pitch, 1.);
    CameraGeometry actual({}, yaw, pitch, 1.);
    const auto origin = aim.origin + (actual.rotation - desired.rotation) * muzzle_offset;
    const auto velocity = actual.rotation * cv::Vec3d(0, 0, speed);
    const cv::Vec3d acceleration(0, 0, -gravity);
    const double delay = aim.horizon - aim.ballistic.flight_time;
    if (delay < 0) return false;
    double flight = aim.ballistic.flight_time;
    for (int i = 0; i < 8; ++i) {
        double phase = target.angle(aim.slot, delay + flight);
        cv::Vec3d normal(std::cos(phase)*std::cos(tilt), std::sin(phase)*std::cos(tilt), -std::sin(tilt));
        cv::Vec3d normal_rate(-normal[1]*target.yaw_rate, normal[0]*target.yaw_rate, 0);
        double r = target.count == 1 ? 0. : (target.count == 4 && aim.slot % 2 ? target.radius_odd : target.radius);
        cv::Vec3d plate_velocity = target.velocity +
            cv::Vec3d(r*target.yaw_rate*std::sin(phase), -r*target.yaw_rate*std::cos(phase), 0);
        auto error = origin + velocity*flight + acceleration*(.5*flight*flight) -
                     target.position(aim.slot, delay + flight);
        double residual = error.dot(normal);
        double derivative = (velocity + acceleration*flight - plate_velocity).dot(normal) + error.dot(normal_rate);
        if (!finite(error) || !std::isfinite(derivative) || std::abs(derivative) < 1e-6) return false;
        if (std::abs(residual) < 1e-7) {
            cv::Vec3d right(std::sin(phase), -std::cos(phase), 0);
            cv::Vec3d down(-std::sin(tilt)*std::cos(phase), -std::sin(tilt)*std::sin(phase), -std::cos(tilt));
            return std::abs(error.dot(right)) < width*.5-kMobilePlateEdgeMargin &&
                   std::abs(error.dot(down)) < height*.5-kMobilePlateEdgeMargin;
        }
        flight -= residual/derivative;
        if (!std::isfinite(flight) || flight <= 0 || std::abs(flight-aim.ballistic.flight_time) > .05) return false;
    }
    return false;
}

inline bool immediateFrontalFire(int number,bool tracking,bool observed,double age,double distance,double facing) {
    return number>=1 && number<=7 && number!=6 && tracking && observed &&
           std::isfinite(age) && age>=0 && age<=.12 && std::isfinite(distance) && distance>0 && distance<=4.5 &&
           std::isfinite(facing) && facing>=.6;
}//判断是否满足正面目标快速射击条件

class FireConfirmation {
public:
    void clear() {
        aligned_since_ = last_time_ = -1;
        target_id_ = 0;
        slot_ = -1;
    }

    bool update(uint32_t target_id, int slot, bool permitted, double time, double settle_time) {
        if (!permitted || !std::isfinite(time) || !std::isfinite(settle_time) || settle_time < 0) {
            clear();
            return false;
        }

        if (target_id != target_id_ || slot != slot_ || last_time_ < 0 || time <= last_time_ ||
            time - last_time_ > 0.2) {
            target_id_ = target_id;
            slot_ = slot;
            aligned_since_ = time;
        }
        last_time_ = time;
        return time - aligned_since_ >= settle_time;
    }//更新射击确认状态

private:
    uint32_t target_id_ = 0;
    int slot_ = -1;
    double aligned_since_ = -1;
    double last_time_ = -1;
};//目标和装甲板的确认设计状态
} // namespace campus
