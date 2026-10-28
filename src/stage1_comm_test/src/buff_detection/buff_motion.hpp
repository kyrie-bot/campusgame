#pragma once
#include "aiming/ballistics.hpp"
#include <deque>
#include "big_rune_motion.hpp"

namespace campus_buff {
class SmallRotorMotion {
public:
    void reset() {samples_.clear();valid_=false;error_=INFINITY;}
    bool update(double angle,double time) {
        if(!std::isfinite(angle)||!std::isfinite(time)){reset();return false;}
        if(!samples_.empty()&&time<=samples_.back().t)return false;
        if(!samples_.empty()&&time-samples_.back().t>.22)reset();
        double phase=angle;
        if(!samples_.empty()) {
            double expected=samples_.back().a+(valid_?velocity_:0)*(time-samples_.back().t);
            phase=expected+std::remainder(angle-expected,2*CV_PI/5);
            if(std::abs(phase-samples_.back().a)>2.8*(time-samples_.back().t)+.045)reset();
        }
        samples_.push_back({time,phase});
        while(!samples_.empty()&&time-samples_.front().t>.3)samples_.pop_front();
        valid_=false;
        if(samples_.size()>=4&&time-samples_.front().t>=.07)fitLinear();
        return true;
    }
    double stamp() const {return samples_.empty()?NAN:samples_.back().t;}
    bool ready() const {return valid_;}
    double error() const {return error_;}
    double advance(double h) const {
        return valid_&&std::isfinite(h)&&h>=0&&h<=.65?std::copysign(CV_PI/3,velocity_)*h:NAN;}
private:
    struct Sample {double t,a;};
    std::deque<Sample> samples_;
    bool valid_=false;
    double velocity_=0,acceleration_=0,error_=INFINITY;
    void fitLinear() {
        double sum = 0, sum2 = 0, sy = 0, sxy = 0;
        double n = samples_.size(), end = samples_.back().t;

        for (auto s : samples_) {
            double t = s.t - end;
            sum += t;
            sum2 += t * t;
            sy += s.a;
            sxy += t * s.a;
        }

        double denom = n * sum2 - sum * sum;
        if (denom < 1e-8)
            return;
        velocity_ = (n * sxy - sum * sy) / denom;
        double b = (sy - velocity_ * sum) / n;
        error_ = 0;

        for (auto s : samples_)
            error_ += std::pow(s.a - b - velocity_ * (s.t - end), 2);
        error_ = std::sqrt(error_ / n);
        acceleration_ = 0;
        valid_ = std::abs(std::abs(velocity_) - CV_PI / 3) < .18 && error_ < .018;
    }

};

class RotorMotion {
public:
    void reset() {small_.reset();big_.reset();have_mode_=false;}
    bool update(double angle,double time,bool big) {
        if(have_mode_&&big!=big_mode_)reset();
        have_mode_=true;big_mode_=big;
        return big?big_.update(angle,time):small_.update(angle,time);
    }
    bool ready() const {return have_mode_&&(big_mode_?big_.ready():small_.ready());}
    bool readyToFire() const {return have_mode_&&(big_mode_?big_.readyToFire():small_.ready());}
    double error() const {return big_mode_?big_.error():small_.error();}
    double stamp() const {return big_mode_?big_.stamp():small_.stamp();}
    double filteredPhase(double p) const {return big_mode_?big_.filteredPhase(p):p;}
    double advance(double h) const {return big_mode_?big_.advance(h):small_.advance(h);}
    const BigRuneMotion& bigModel() const {return big_;}
private:
    SmallRotorMotion small_;
    BigRuneMotion big_;
    bool have_mode_=false,big_mode_=false;
};

struct RunePlane {
    cv::Vec3d center, normal, right;

    static RunePlane forPlayer(int player) {
        double n = std::sqrt(.5);
        if (player == 1)
            return {{0, 1.5, 1.}, {-n, n, 0}, {n, n, 0}};
        return {{0, 1.5, 1.}, {n, -n, 0}, {-n, -n, 0}};
    }

    cv::Matx33d rectification(const campus::CameraGeometry& camera, const cv::Matx33d& k) const {
        double yaw = std::atan2(normal[1], -normal[0]) * 180 / CV_PI;

        campus::CameraGeometry front({0, 0, 0}, yaw, 0, 1.);
        cv::Matx33d h = k * front.rotation.t() * camera.rotation * k.inv();
        cv::Vec3d c = h * cv::Vec3d(k(0, 2), k(1, 2), 1);
        if (!campus::finite(c) || std::abs(c[2]) < .1)
            return cv::Matx33d::eye();
        double facing = std::abs(normal.dot(camera.rotation * cv::Vec3d(0, 0, 1)));
        double scale = std::clamp(facing * facing, .25, 1.);
        cv::Matx33d centre(scale, 0, k(0, 2) - scale * c[0] / c[2], 0, scale, k(1, 2) - scale * c[1] / c[2], 0, 0, 1);
        return centre * h;
    }

    static cv::Point2f mapPixel(const cv::Matx33d& h, cv::Point2f p) {
        cv::Vec3d v = h * cv::Vec3d(p.x, p.y, 1);
        if (!campus::finite(v) || std::abs(v[2]) < 1e-6)
            return {NAN, NAN};
        return {float(v[0] / v[2]), float(v[1] / v[2])};
    }

    bool solveHub(cv::Point2f r_pixel, cv::Point2f leaf_pixel, const campus::CameraGeometry& camera,
                  const cv::Matx33d& k, const cv::Mat& d, double physical_radius, cv::Vec3d& hub,
                  cv::Vec3d& leaf) const {
        if (!std::isfinite(physical_radius) || physical_radius < .5 || physical_radius > .9)
            return false;
        std::vector<cv::Point2f> input{r_pixel, leaf_pixel}, points;
        cv::undistortPoints(input, points, k, d);
        cv::Vec3d r = camera.rotation * cv::Vec3d(points[0].x, points[0].y, 1);
        cv::Vec3d t = camera.rotation * cv::Vec3d(points[1].x, points[1].y, 1);
        double nr = r.dot(normal), nt = t.dot(normal);
        if (nr >= -.3 || nt >= -.3)
            return false;
        double ratio = nr / nt;
        cv::Vec3d difference = t * ratio - r;
        double length = cv::norm(difference);
        if (length < .02)
            return false;
        double depth = physical_radius / length;
        if (depth < 1.5 || depth > 12 || ratio <= 0)
            return false;
        hub = camera.origin + r * depth;
        leaf = camera.origin + t * (depth * ratio);
        return campus::finite(hub) && campus::finite(leaf);
    }

    bool intersect(cv::Point2f pixel, const campus::CameraGeometry& camera, const cv::Matx33d& k, const cv::Mat& d,
                   cv::Vec3d& point) const {
        std::vector<cv::Point2f> input{pixel}, norm;
        cv::undistortPoints(input, norm, k, d);
        cv::Vec3d ray = camera.rotation * cv::Vec3d(norm[0].x, norm[0].y, 1);
        double denom = ray.dot(normal), distance = (camera.origin - center).dot(normal);
        if (distance < 1 || denom >= -.25)
            return false;
        double t = -distance / denom;
        if (t <= 0 || t > 20)
            return false;
        point = camera.origin + ray * t;
        return campus::finite(point);
    }

    bool trackedLeaf(cv::Point2f r_pixel, cv::Point2f leaf_pixel,
                     const campus::CameraGeometry& camera, const cv::Matx33d& k,
                     const cv::Mat& d, double radius, cv::Vec3d& leaf) const {
        cv::Vec3d measured_hub;
        if (!std::isfinite(radius) || radius < .5 || radius > .9 ||
            !intersect(r_pixel, camera, k, d, measured_hub) ||
            cv::norm(measured_hub - center) > .12 ||
            !intersect(leaf_pixel, camera, k, d, leaf)) return false;
        return std::abs(cv::norm(leaf - measured_hub) - radius) <= .10;
    }

    cv::Vec3d target(const cv::Vec3d& hub, double phase, double radius = .7) const {
        return hub + right * (radius * std::cos(phase)) + cv::Vec3d(0, 0, radius * std::sin(phase));
    }

    double phase(const cv::Vec3d& delta) const {
        return std::atan2(delta[2], delta.dot(right));
    }
};

inline double runeProjectileSpeed(double scene_speed, double nominal = 23.5) {
    return std::isfinite(scene_speed) && scene_speed > 0 ? scene_speed / 3. : nominal;
}

struct RuneIntercept {
    campus::BallisticSolution ballistic;
    double horizon = 0;
    cv::Vec3d target{};
};
inline RuneIntercept intercept(const RunePlane& plane, const cv::Vec3d& hub, double phase, const RotorMotion& motion,
                               const cv::Vec3d& pivot, double yaw, double pitch, double speed, double age,
                               double latency, double radius = .7) {
    RuneIntercept out;
    if (!std::isfinite(radius) || radius < .59 || radius > .81 || !motion.ready() || !campus::finite(hub) ||
        !campus::finite(pivot) || !std::isfinite(age) || age < 0 || age > .28 || !std::isfinite(latency) ||
        latency < 0 || latency > .1)
        return out;
    double horizon = age + latency;

    for (int i = 0; i < 20; ++i) {
        double advance = motion.advance(horizon);
        if (!std::isfinite(advance))
            return {};
        auto target = plane.target(hub, motion.filteredPhase(phase) + advance, radius);

        campus::CameraGeometry barrel(pivot, yaw, pitch, 1., {0, 0, .0833333});
        auto b = campus::solveBallistics(target - barrel.origin, speed);

        if (!b.valid)
            return {};
        double next = age + latency + b.flight_time;
        if (std::abs(next - horizon) < 1e-5 && std::hypot(b.yaw - yaw, b.pitch - pitch) < .001)
            return {b, next, target};
        horizon = next;
        yaw = b.yaw;
        pitch = b.pitch;
    }

    return out;
}
} // namespace campus_buff
