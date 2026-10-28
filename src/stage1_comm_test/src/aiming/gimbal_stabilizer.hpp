#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace campus {
inline double steadySeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline double wrapDegrees(double value) {
    return std::remainder(value, 360.0);
}

struct GimbalStabilizerConfig {
    bool enabled = true;
    double time_constant_s = 0;
    double yaw_deadband_deg = 0.03;
    double pitch_deadband_deg = 0.03;
    double max_yaw_rate_deg_s = 180;
    double max_pitch_rate_deg_s = 120;
    double max_dt_s = 0.05;
    double reset_gap_s = 0.2;
};//保存云台相关参数

struct GimbalCommand {
    bool valid = false;
    double yaw = 0;
    double pitch = 0;
    double yaw_residual = 0;
    double pitch_residual = 0;
};//云台指令和角度信息

class GimbalStabilizer {
public:
    explicit GimbalStabilizer(GimbalStabilizerConfig config = {}) : config_(config) {
        if (!std::isfinite(config.time_constant_s) || config.time_constant_s < 0 ||
            !std::isfinite(config.yaw_deadband_deg) || config.yaw_deadband_deg < 0 ||
            !std::isfinite(config.pitch_deadband_deg) || config.pitch_deadband_deg < 0 ||
            !std::isfinite(config.max_yaw_rate_deg_s) || config.max_yaw_rate_deg_s <= 0 ||
            !std::isfinite(config.max_pitch_rate_deg_s) || config.max_pitch_rate_deg_s <= 0 ||
            !std::isfinite(config.max_dt_s) || config.max_dt_s <= 0 || !std::isfinite(config.reset_gap_s) ||
            config.reset_gap_s <= config.max_dt_s) {
            throw std::invalid_argument("Invalid gimbal stabilizer parameters");
        }
    }

    void reset() {
        initialized_ = false;
    }

    //根据目标角度和当前云台角度生成稳定的控制指令
    GimbalCommand update(double raw_yaw, double raw_pitch, double current_yaw, double current_pitch, double time) {
        GimbalCommand result;
        if (!std::isfinite(raw_yaw) || !std::isfinite(raw_pitch) || !std::isfinite(current_yaw) ||
            !std::isfinite(current_pitch) || !std::isfinite(time) || std::abs(raw_pitch) > 60 ||
            std::abs(current_pitch) > 60) {
            reset();
            return result;
        }

        if (!config_.enabled) {
            yaw_ = wrapDegrees(raw_yaw);
            pitch_ = raw_pitch;
        } else {
            double dt = time - last_time_;
            if (!initialized_ || dt > config_.reset_gap_s || dt < 0) {
                yaw_ = wrapDegrees(current_yaw);
                pitch_ = current_pitch;
                dt = std::min(1.0 / 120.0, config_.max_dt_s);
            } else {
                dt = std::min(dt, config_.max_dt_s);
            }

            double alpha = 1;
            if (config_.time_constant_s > 0) {
                alpha = -std::expm1(-dt / config_.time_constant_s);
            }

            double yaw_error = wrapDegrees(raw_yaw - yaw_);
            double pitch_error = raw_pitch - pitch_;
            if (std::abs(yaw_error) <= config_.yaw_deadband_deg) {
                yaw_error = 0;
            }

            if (std::abs(pitch_error) <= config_.pitch_deadband_deg) {
                pitch_error = 0;
            }
            yaw_ = wrapDegrees(yaw_ + std::clamp(alpha * yaw_error, -config_.max_yaw_rate_deg_s * dt,
                                                 config_.max_yaw_rate_deg_s * dt));
            pitch_ = std::clamp(pitch_ + std::clamp(alpha * pitch_error, -config_.max_pitch_rate_deg_s * dt,
                                                    config_.max_pitch_rate_deg_s * dt),
                                -60.0, 60.0);
        }
        initialized_ = true;
        last_time_ = time;
        result.valid = true;
        result.yaw = yaw_;
        result.pitch = pitch_;
        result.yaw_residual = wrapDegrees(raw_yaw - yaw_);
        result.pitch_residual = raw_pitch - pitch_;
        return result;
    }

private:
    GimbalStabilizerConfig config_;
    bool initialized_ = false;
    double last_time_ = 0;
    double yaw_ = 0;
    double pitch_ = 0;
};

inline bool gimbalCommandSettled(const GimbalCommand& command, double tolerance_deg) {
    return command.valid && std::isfinite(command.yaw) && std::isfinite(command.pitch) &&
           std::isfinite(command.yaw_residual) && std::isfinite(command.pitch_residual) &&
           std::isfinite(tolerance_deg) && tolerance_deg > 0 && tolerance_deg < 1.2 &&
           std::hypot(command.yaw_residual, command.pitch_residual) < tolerance_deg;
}//判断云台指令是否已经满足稳定条件

inline bool gimbalAligned(double raw_yaw, double raw_pitch, const GimbalCommand& command, double actual_yaw,
                          double actual_pitch, double tolerance_deg) {
    if (!command.valid || !std::isfinite(actual_yaw) || !std::isfinite(actual_pitch) || !std::isfinite(tolerance_deg) ||
        tolerance_deg <= 0) {
        return false;
    }

    double raw_error = std::hypot(wrapDegrees(raw_yaw - actual_yaw), raw_pitch - actual_pitch);
    double command_error = std::hypot(wrapDegrees(command.yaw - actual_yaw), command.pitch - actual_pitch);
    double lag_error = std::hypot(command.yaw_residual, command.pitch_residual);
    return raw_error < tolerance_deg && command_error < tolerance_deg && lag_error < tolerance_deg;
}//判断云台是否瞄准
} // namespace campus