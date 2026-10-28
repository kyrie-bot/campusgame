#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <array>

namespace campus {
struct RuneGimbalEstimate {
    bool valid = false;
    double yaw = 0, pitch = 0;
};

class RuneGimbalFeedback {
    struct Sample {
        double t, y, p;
    };
    std::deque<Sample> samples_;

public:
    void reset() {
        samples_.clear();
    }

    void update(double t, double yaw, double pitch) {
        if (!std::isfinite(t) || !std::isfinite(yaw) || !std::isfinite(pitch)) {
            reset();
            return;
        }

        if (!samples_.empty()) {
            if (t <= samples_.back().t)
                return;
            if (t - samples_.back().t > .28)
                reset();
            else
                yaw = samples_.back().y + std::remainder(yaw - samples_.back().y, 360.);
        }
        samples_.push_back({t, yaw, pitch});

        while (samples_.size() > 12 || (!samples_.empty() && t - samples_.front().t > .24))
            samples_.pop_front();
    }//更新云台yaw,pitch

    RuneGimbalEstimate predict(double now, double command_delay = 0.) const {
        auto estimate = predictWindow(now, command_delay);
        if (estimate.valid) return estimate;

        for (size_t count : {size_t(5), size_t(4)}) {
            if (samples_.size() <= count) continue;
            auto recent = *this;
            while (recent.samples_.size() > count) recent.samples_.pop_front();
            estimate = recent.predictWindow(now, command_delay);
            if (estimate.valid) return estimate;
        }
        return {};
    }//预测云台姿态

private:
    RuneGimbalEstimate predictWindow(double now, double command_delay) const {
        if (samples_.size() < 4)
            return {};
        double end = samples_.back().t, age = now - end;
        if (!std::isfinite(age) || age < 0 || age > .28 || end - samples_.front().t < .06 ||
            !std::isfinite(command_delay) || command_delay < 0 || command_delay > .1)
            return {};
        age += command_delay;
        double st = 0, stt = 0, sy = 0, sp = 0, sty = 0, stp = 0, n = samples_.size();

        for (auto s : samples_) {
            double t = s.t - end;
            st += t;
            stt += t * t;
            sy += s.y;
            sp += s.p;
            sty += t * s.y;
            stp += t * s.p;
        }

        double den = n * stt - st * st;
        if (den < 1e-8)
            return {};
        double vy = (n * sty - st * sy) / den, vp = (n * stp - st * sp) / den;
        double y = (sy - vy * st) / n, p = (sp - vp * st) / n, error = 0;

        for (auto s : samples_)
            error += std::pow(s.y - y - vy * (s.t - end), 2) + std::pow(s.p - p - vp * (s.t - end), 2);

        if (samples_.size() >= 7 && end - samples_.front().t >= .12) {
            const double span = end - samples_.front().t;
            auto fit = [&](size_t count, auto& cy, auto& cp) {
                const int degree = cy.size();
                double a[4][6]{};

                for (size_t k = 0; k < count; ++k) {
                    auto s = samples_[k];
                    double t = (s.t - end) / span, x[4]{1, t, t * t, t*t*t};

                    for (int i = 0; i < degree; ++i) {
                        for (int j = 0; j < degree; ++j)
                            a[i][j] += x[i] * x[j];
                        a[i][degree] += x[i] * s.y;
                        a[i][degree+1] += x[i] * s.p;
                    }
                }

                for (int i = 0; i < degree; ++i) {
                    int pivot = i;

                    for (int j = i + 1; j < degree; ++j)
                        if (std::abs(a[j][i]) > std::abs(a[pivot][i]))
                            pivot = j;
                    if (std::abs(a[pivot][i]) < 1e-8)
                        return false;

                    for (int j = 0; j < degree+2; ++j)
                        std::swap(a[i][j], a[pivot][j]);
                    double divisor = a[i][i];

                    for (int j = i; j < degree+2; ++j)
                        a[i][j] /= divisor;

                    for (int k = 0; k < degree; ++k)
                        if (k != i) {
                            double f = a[k][i];

                            for (int j = i; j < degree+2; ++j)
                                a[k][j] -= f * a[i][j];
                        }
                }

                for (int i = 0; i < degree; ++i) {
                    cy[i] = a[i][degree];
                    cp[i] = a[i][degree+1];
                }

                return true;
            };
            auto value = [](const auto& c, double t) { double value = 0;
                for (int i=int(c.size())-1;i>=0;--i) value=value*t+c[i];
                return value; };

            if (samples_.size() >= 8 && span >= .12) {
                std::array<double,4> cy{},cp{},hy{},hp{};

                if (fit(samples_.size(),cy,cp) && fit(samples_.size()-2,hy,hp)) {
                    double residual=0,heldout=0,maximum_speed=0;

                    for(size_t i=0;i<samples_.size();++i) {
                        auto s=samples_[i];double t=(s.t-end)/span;
                        residual+=std::pow(s.y-value(cy,t),2)+std::pow(s.p-value(cp,t),2);

                        if(i>=samples_.size()-2)
                            heldout+=std::pow(s.y-value(hy,t),2)+std::pow(s.p-value(hp,t),2);

                        maximum_speed=std::max(maximum_speed,std::hypot(
                            (cy[1]+2*cy[2]*t+3*cy[3]*t*t)/span,
                            (cp[1]+2*cp[2]*t+3*cp[3]*t*t)/span));
                    }

                    double dt=age/span,py=value(cy,dt),pp=value(cp,dt);
                    double speed=std::hypot((cy[1]+2*cy[2]*dt+3*cy[3]*dt*dt)/span,
                                            (cp[1]+2*cp[2]*dt+3*cp[3]*dt*dt)/span);
                    double acceleration=std::hypot(2*cy[2]+6*cy[3]*dt,2*cp[2]+6*cp[3]*dt)/(span*span);
                    double jerk=6*std::hypot(cy[3],cp[3])/(span*span*span);

                    if(std::sqrt(residual/n)<.035 && std::sqrt(heldout/2)<.06 &&
                       maximum_speed<=35 && speed<=35 && acceleration<=100 && jerk<=400 &&
                       std::hypot(py-cy[0],pp-cp[0])<=5.)
                        return {true,std::remainder(py,360.),pp};
                }
            }

            std::array<double, 3> cy{}, cp{}, hy{}, hp{};

            if (fit(samples_.size(), cy, cp) && fit(samples_.size() - 2, hy, hp)) {
                double residual = 0, heldout = 0;

                for (size_t i = 0; i < samples_.size(); ++i) {
                    auto s = samples_[i];
                    double t = (s.t - end) / span;
                    residual += std::pow(s.y - value(cy, t), 2) + std::pow(s.p - value(cp, t), 2);

                    if (i >= samples_.size() - 2)
                        heldout += std::pow(s.y - value(hy, t), 2) + std::pow(s.p - value(hp, t), 2);
                }

                double ay = 2 * cy[2] / (span * span), ap = 2 * cp[2] / (span * span), dt = age / span;
                double py = value(cy, dt), pp = value(cp, dt);

                if (std::sqrt(residual / n) < .035 && std::sqrt(heldout / 2) < .06 && std::hypot(ay, ap) <= 60 &&
                    std::hypot(cy[1] / span + ay * age, cp[1] / span + ap * age) <= 35 &&
                    std::hypot(py - cy[0], pp - cp[0]) <= 5.)
                    return {true, std::remainder(py, 360.), pp};
            }
        }

        if (std::sqrt(error / n) > .15 || std::hypot(vy, vp) > 35 || std::hypot(vy * age, vp * age) > 5.)
            return {};
        return {true, std::remainder(y + vy * age, 360.), p + vp * age};
    }
};
}
