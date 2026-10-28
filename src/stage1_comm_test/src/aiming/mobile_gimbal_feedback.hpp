#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>

namespace campus {
struct MobileGimbalEstimate {
    bool valid = false;
    double stamp = 0, yaw = 0, pitch = 0, yaw_bound = 0, pitch_bound = 0;
};

// Armor interception is evaluated at release time. A delayed camera-state yaw
// must not be treated as the barrel direction at that time. Extrapolate measured
// angles only during smooth motion, and reserve the estimated error inside the
// physical plate window. This has no connection to the rune prediction model.
class MobileGimbalFeedback {
    struct Sample { double t; std::array<double, 2> angle; };
    std::deque<Sample> samples_;

public:
    void reset() { samples_.clear(); }

    void update(double stamp, double yaw, double pitch) {
        if (!std::isfinite(stamp) || !std::isfinite(yaw) || !std::isfinite(pitch)) {
            reset();
            return;
        }
        if (!samples_.empty()) {
            if (stamp == samples_.back().t) return;
            if (stamp < samples_.back().t || stamp - samples_.back().t > .2) reset();
            else yaw = samples_.back().angle[0] + std::remainder(yaw - samples_.back().angle[0], 360.);
        }
        samples_.push_back({stamp, {yaw, pitch}});
        while (samples_.size() > 8 || stamp - samples_.front().t > .24) samples_.pop_front();
    }

    MobileGimbalEstimate predict(double now, double delay) const {
        if (samples_.size() < 3 || !std::isfinite(now) || !std::isfinite(delay) || delay < 0 || delay > .1)
            return {};
        const double end = samples_.back().t, age = now - end, span = end - samples_.front().t;
        if (age < .025 || age > .2 || span < .06) return {};
        const double horizon = age + delay, n = samples_.size();
        double st = 0, stt = 0;
        for (const auto& s : samples_) { double t = s.t - end; st += t; stt += t*t; }
        const double den = n*stt - st*st;
        if (den < 1e-8) return {};
        MobileGimbalEstimate result;
        result.stamp = end;
        for (int axis = 0; axis < 2; ++axis) {
            double sx = 0, stx = 0;
            for (const auto& s : samples_) { sx += s.angle[axis]; stx += (s.t-end)*s.angle[axis]; }
            const double rate = (n*stx-st*sx)/den, intercept = (sx-rate*st)/n;
            double residual = 0;
            for (const auto& s : samples_)
                residual = std::max(residual, std::abs(s.angle[axis]-intercept-rate*(s.t-end)));
            const auto& first = samples_.front();
            const auto& middle = samples_[samples_.size()/2];
            const auto& last = samples_.back();
            const double early = (middle.angle[axis]-first.angle[axis])/(middle.t-first.t);
            const double late = (last.angle[axis]-middle.angle[axis])/(last.t-middle.t);
            const double acceleration = 2*std::abs(late-early)/span;
            const double angle = intercept + rate*horizon;
            const double bound = .02 + 2*residual + .5*acceleration*horizon*(span+horizon);
            // A reversal, noisy feedback or large extrapolation cannot authorize
            // a shot. In those cases callers retain the original measured gate.
            if (!std::isfinite(angle) || !std::isfinite(bound) || residual > .08 ||
                std::abs(rate) > 90 || std::abs(angle-last.angle[axis]) > 8 || bound > .6 ||
                (early*late < 0 && std::min(std::abs(early),std::abs(late)) > 1)) return {};
            if (axis == 0) { result.yaw = angle; result.yaw_bound = bound; }
            else { result.pitch = angle; result.pitch_bound = bound; }
        }
        result.valid = true;
        return result;
    }
};
} // namespace campus
