#pragma once

#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace campus {
struct CameraMotionNoiseConfig {
    bool enabled = false;
    double timing_sigma_s = 0;
    double max_dt_s = 0.2;
    double max_rate_rad_s = 20;
    double max_bearing_sigma_rad = 0.03;
};

struct CameraMotionUncertainty {
    bool rate_valid = false;
    double yaw_rate = 0;
    double pitch_rate = 0;
    double bearing_variance = 0;
};

class CameraMotionNoise {
  public:
    explicit CameraMotionNoise(CameraMotionNoiseConfig config = {}) : config_(config) {
        if (!std::isfinite(config.timing_sigma_s) || config.timing_sigma_s < 0 || !std::isfinite(config.max_dt_s) ||
            config.max_dt_s <= 0 || !std::isfinite(config.max_rate_rad_s) || config.max_rate_rad_s <= 0 ||
            !std::isfinite(config.max_bearing_sigma_rad) || config.max_bearing_sigma_rad <= 0) {
            throw std::invalid_argument("Invalid camera motion noise parameters");
        }
    }

    void reset() {
        initialized_ = false;
    }

    CameraMotionUncertainty update(double yaw_deg, double pitch_deg, double time) {
        CameraMotionUncertainty result;
        if (!std::isfinite(yaw_deg) || !std::isfinite(pitch_deg) || !std::isfinite(time)) {
            reset();
            return result;
        }

        constexpr double radians = 3.14159265358979323846 / 180.0;
        double dt = time - last_time_;
        if (initialized_ && dt > 1e-6 && dt <= config_.max_dt_s) {
            result.yaw_rate = std::remainder(yaw_deg - last_yaw_, 360.0) * radians / dt;
            result.pitch_rate = (pitch_deg - last_pitch_) * radians / dt;
            double rate = std::hypot(result.yaw_rate, result.pitch_rate);
            result.rate_valid = std::isfinite(rate) && rate <= config_.max_rate_rad_s;

            if (config_.enabled && result.rate_valid) {
                double sigma = std::min(rate * config_.timing_sigma_s, config_.max_bearing_sigma_rad);
                result.bearing_variance = sigma * sigma;
            }
        }

        if (!initialized_ || time > last_time_) {
            last_yaw_ = yaw_deg;
            last_pitch_ = pitch_deg;
            last_time_ = time;
            initialized_ = true;
        }

        return result;
    }

  private:
    CameraMotionNoiseConfig config_;
    bool initialized_ = false;
    double last_yaw_ = 0;
    double last_pitch_ = 0;
    double last_time_ = 0;
};
} // namespace campus