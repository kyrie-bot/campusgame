#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <optional>
#include <string>
#include <utility>

namespace campus {
inline bool robotHighlandFireAllowed(int number, bool terrain_known, bool on_highland) {
    return number != 4 || !terrain_known || on_highland;
}

inline double angleDifference(double a, double b) {
    return std::remainder(a - b, 360.);
}

inline double slewAngle(double current, double target, double dt, double rate = 180.) {
    return current + std::clamp(angleDifference(target, current), -rate * dt, rate * dt);
}//限制角度变化速度

inline double searchBearing(double command, double measured, double target, double dt) {
    double next = slewAngle(command, target, dt, 90.);
    return std::remainder(measured + std::clamp(angleDifference(next, measured), -15., 15.), 360.);
}//控制云台朝指定方向搜索

inline bool sentrySearchStage(const std::string& stage, const std::array<int32_t,7>& health, bool ignore_one) {
    return (stage=="CLEAR_ROBOTS" || stage=="ROBOT" || stage=="WAIT_CLEAR_ROBOTS_POSITION") &&
           (ignore_one || health[0]<=0) && health[1]<=0 && health[2]<=0 && health[3]<=0 && health[4]>0;
}//在限定角度范围内搜索哨兵

inline double boundedSentrySearch(double previous, double centre, double time, double dt) {
    return slewAngle(previous,centre+22.*std::sin(time*1.1),dt,90.);
}

inline bool freshSample(double receipt_age, double source_age, double receipt_limit, double source_limit) {
    return std::isfinite(receipt_age) && std::isfinite(source_age) && receipt_age >= 0 &&
           receipt_age <= receipt_limit && source_age >= -.1 && source_age <= source_limit;
}

inline bool navigationSearchAllowed(const std::string& follow, const std::string& mission, bool cancelled, int ammo,
                                    bool weakened) {
    if (cancelled || ammo <= 0 || weakened)
        return false;
    if (mission != "CLEAR_ROBOTS" && mission != "ROBOT" && mission != "OUTPOST" && mission != "BASE" &&
        mission != "WAIT_CLEAR_ROBOTS_POSITION" && mission != "WAIT_OUTPOST_POSITION" &&
        mission != "WAIT_BASE_POSITION")
        return false;
    return follow == "WAIT_PATH" || follow == "STALE_PATH" || follow == "WAIT_GOAL" || follow == "WAIT_MISSION_PATH" ||
           follow == "STUCK_SELECT_NEW_GOAL" || follow == "CANCELLED_SELECT_NEW_GOAL" || follow == "NO_PATH" ||
           follow == "START_BLOCKED" || follow == "GOAL_BLOCKED" || follow == "FOLLOWING" || follow == "ARRIVED" ||
           follow == "SETTLING" || follow == "LOCAL_BLOCKED" || follow == "WAIT_OBSTACLE";
}//判断导航过程中是否允许搜索

inline bool navigationFireAllowed(const std::string& follow, bool matching_goal, bool ready) {
    return matching_goal && ready && follow == "FOLLOWING";
}//判断导航过程中是否允许射击

inline bool stationaryDefenseAllowed(const std::string& follow, const std::string& mission, bool cancelled, int ammo,
                                     bool weakened, double speed, bool after_arrival) {
    if (cancelled || ammo <= 0 || weakened || !std::isfinite(speed) || speed > .03 || speed < 0 || after_arrival)
        return false;
    if (mission != "REFILL" && mission != "CLEAR_ROBOTS" && mission != "WAIT_CLEAR_ROBOTS_POSITION" &&
        mission != "BASE" && mission != "OPENING_RAMP")
        return false;
    return follow == "START_BLOCKED" || follow == "GOAL_BLOCKED" || follow == "NO_PATH" || follow == "WAIT_GOAL" ||
           follow == "LOCAL_BLOCKED" || follow == "WAIT_OBSTACLE" || follow == "COMBAT_EVADE_BLOCKED" ||
           follow == "STUCK_SELECT_NEW_GOAL";
}

inline bool tentativeMobilePitchAllowed(const std::string& reason, double pitch) {
    return std::isfinite(pitch) && (reason != "WAIT_TRACK_CONFIRM" || std::abs(pitch) <= 8.);
}

inline double mobileSearchPitch(double pitch, double age, double lease = 1.5) {
    return std::isfinite(pitch) && std::isfinite(age) && age >= 0 && age <= lease ? pitch : 0.;
}//搜索移动目标时的pitch

struct HeadingCommand {
    double yaw = 0, pitch = 0;
};

class TravelHeading {
public:
    void reset() {
        initialized_ = false;
    }

    HeadingCommand update(double yaw, double pitch, double target, double dt) {
        if (!initialized_) {
            command_ = {yaw, pitch};
            initialized_ = true;
        }
        command_.yaw = std::remainder(slewAngle(command_.yaw, target, dt), 360.);
        command_.pitch += std::clamp(-command_.pitch, -120 * dt, 120 * dt);
        return command_;
    }

private:
    bool initialized_ = false;
    HeadingCommand command_;
};//移动过程中控制云台朝向

inline std::pair<std::optional<double>, std::string> motionHeading(double forward, double left, double x, double z) {
    if (std::isfinite(forward) && std::isfinite(left) && std::hypot(forward, left) > .6)
        return {std::atan2(-left, forward) * 180 / M_PI, "FEEDBACK"};

    if (std::isfinite(x) && std::isfinite(z) && std::hypot(x, z) >= .03)
        return {std::atan2(x, z) * 180 / M_PI, "COMMAND"};
    return {std::nullopt, "NONE"};
}//判断特定目标是否满足高地位置射击条件

class ScanHeading {
public:
    void reset() {
        yaw_.reset();
    }

    double update(double measured, double dt, double rate) {
        if (!yaw_)
            yaw_ = measured;
        *yaw_ += rate * dt;
        *yaw_ = std::remainder(measured + std::clamp(angleDifference(*yaw_, measured), -15., 15.), 360.);
        return *yaw_;
    }

private:
    std::optional<double> yaw_;
};//控制云台水平扫描

class PitchSearch {
public:
    void reset() {
        pitch_.reset();
        direction_ = -1.;
    }

    double update(double measured, double dt, double low, double high, double rate = 12.) {
        if (!pitch_)
            pitch_ = measured;

        if (*pitch_ < low)
            direction_ = 1.;

        else if (*pitch_ > high)
            direction_ = -1.;

        double target = *pitch_ + direction_ * rate * dt;

        if (target <= low && *pitch_ >= low) {
            target = low;
            direction_ = 1.;
        }

        if (target >= high && *pitch_ <= high) {
            target = high;
            direction_ = -1.;
        }

        *pitch_ = std::clamp(target, measured - 6., measured + 6.);
        return *pitch_;
    }

private:
    std::optional<double> pitch_;
    double direction_ = -1.;
};//控制云台俯仰扫描
} // namespace campus