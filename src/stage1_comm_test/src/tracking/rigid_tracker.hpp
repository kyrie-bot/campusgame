#pragma once

#include "geometry/aim_geometry.hpp"
#include "classification/armor_identity.hpp"

#include <Eigen/Dense>
#include <string>

namespace campus {
struct Observation {
    cv::Vec3d position{0, 0, 0};
    cv::Vec3d view_origin{0, 0, 0};
    cv::Vec3d alternative_position{0, 0, 0};

    std::array<cv::Point2f, 4> image_points;
    cv::Matx33d camera_rotation = cv::Matx33d::eye();
    cv::Matx33d camera_matrix = cv::Matx33d::eye();
    cv::Mat distortion;
    bool has_pixels = false;

    double yaw = 0;
    double position_variance = 0.0025;
    double yaw_variance = 0.03;
    double tangential_variance = 0;
    double alternative_yaw = 0;
    double width = 0.135;
    double height = 0.055;
    double tilt = 0;

    int number = 0;
    bool has_view = false;
    bool has_alternative = false;
};//一次装甲板观测结果

struct TrackerConfig {
    bool use_pixel_update = true;

    double pixel_sigma = 2.0;
    double pixel_gate = 40.0;
    double radius = 0.26;
    double outpost_radius = 0.28;
    double min_radius = 0.12;
    double max_radius = 0.4;
    double initial_radius_sigma = 0.08;
    double initial_height_sigma = 0.20;

    double max_height_difference = 0.50;
    double match_distance = 0.6;
    double match_yaw = 1.2;
    double mahalanobis_gate = 25;
    double q_xy = 0.5;
    double q_z = 0.1;
    double q_yaw = 0.5;
    double q_outpost_yaw = 0.08;

    double q_radius = 0.00001;

    double outpost_center_gain_scale = 1.0;
    double outpost_center_warmup_s = 1.5;
    double outpost_center_max_sigma_m = 0.03;
    double outpost_center_max_residual_m = 0.08;
    double outpost_center_q = 1e-5;

    double lost_timeout = 0.35;
    double min_confirm_time = 0.05;
    int confirm_frames = 5;
    int field_confirm_frames = 0;//确认跟踪需要的匹配帧数 
    double field_confirm_time = .03;
    int outpost_number = 6;
    int forced_count = 0;
    int max_tracks = 1;
    bool track_unknown = false;
    bool stationary_outpost = true;

    int confirmationFrames(int number) const {
        return field_confirm_frames > 0 && number >= 2 && number <= 4 ? field_confirm_frames : confirm_frames;
    }//返回目标确认跟踪帧数

    double confirmationTime(int number) const {
        return field_confirm_frames > 0 && number >= 2 && number <= 4 ? field_confirm_time : min_confirm_time;
    }//返回目标确认跟踪的时间

    int armorCount(int number) const {
        if (forced_count != 0) {
            return forced_count;
        }

        if (number == outpost_number) {
            return 3;
        }

        if (number == 8) {
            return 1;
        }

        return 4;
    }
};//跟踪算法配置参数

//估计整个刚体的运动状态
class RigidTarget {
  public:
    typedef Eigen::Matrix<double, 12, 1> State;

    typedef Eigen::Matrix<double, 12, 12> Covariance;

    uint32_t id;
    int number;
    int count;
    int confirmed = 1;
    int last_slot = 0;
    int missed_frames = 0;
    double last_seen;
    double created_at;

    double width;
    double height;
    double tilt;
    bool geometry_observable = false;
    std::string state = "DETECTING";

    std::array<bool, 4> observed{true, false, false, false};
    std::array<bool, 4> seen{true, false, false, false};
    State x = State::Zero();
    Covariance p = Covariance::Identity();

    RigidTarget(uint32_t track_id, const Observation& o, double time, const TrackerConfig& config)
        : id(track_id), number(o.number), count(config.armorCount(o.number)), last_seen(time), created_at(time),
          width(o.width), height(o.height), tilt(o.tilt), config_(config) {
        double radius = config.radius;
        if (count == 3) {
            radius = config.outpost_radius;
        } else if (count == 1) {
            radius = 0;
        }

        x(0) = o.position[0] + radius * std::cos(o.yaw);
        x(1) = 0;
        x(2) = o.position[1] + radius * std::sin(o.yaw);
        x(3) = 0;
        x(4) = o.position[2];
        x(5) = 0;

        x(6) = o.yaw;
        x(7) = 0;
        x(8) = radius;
        x(9) = radius;
        x(10) = 0;//奇数槽位相对基准高度的高度差
        x(11) = 0;//某块装甲板相对基准高度的差值，用于前哨站

        p.setZero();
        p(0, 0) = 0.04;
        p(1, 1) = 1;
        p(2, 2) = 0.04;
        p(3, 3) = 1;
        p(4, 4) = 0.02;
        p(5, 5) = 0.3;

        p(6, 6) = 0.15;
        p(7, 7) = 16;
        p(8, 8) = 0.0004;
        p(9, 9) = 0.0004;
        p(10, 10) = 0.0001;
        p(11, 11) = 0.0001;//状态协方差矩阵，描述当前状态的不确定性

        if (config.confirmationFrames(number) <= 1 && config.confirmationTime(number) == 0) {
            state = "TRACKING";
        }
    }

    double yaw(int slot, double horizon = 0) const {
        return wrap(x(6) + x(7) * horizon + slot * 2 * CV_PI / count);
    }//horizon：预测未来多少秒

    double radius(int slot) const {
        if (count == 1) {
            return 0;
        }

        if (count == 4 && slot % 2) {
            return x(9);
        }

        return x(8);
    }

    cv::Vec3d position(int slot, double horizon = 0) const {
        double a = yaw(slot, horizon);
        double r = radius(slot);
        double dz = 0;

        if (count == 4 && slot % 2) {
            dz = x(10);
        } else if (count == 3 && slot > 0) {
            dz = x(9 + slot);
        }

        return {x(0) + x(1) * horizon - r * std::cos(a), x(2) + x(3) * horizon - r * std::sin(a),
                x(4) + x(5) * horizon + dz};
    }//未来某块装甲板的位置

    cv::Vec3d armorVelocity(int slot) const {
        double a = yaw(slot);
        double r = radius(slot);
        return {x(1) + r * std::sin(a) * x(7), x(3) - r * std::cos(a) * x(7), x(5)};
    }//通过位置对时间的导数算速度

    void predict(double dt) {
        Covariance f = Covariance::Identity();//状态转移矩阵
        Covariance q = Covariance::Zero();//过程噪声协方差矩阵
        bool stationary = count == 3 && config_.stationary_outpost;//当前目标是否按“中心固定的前哨站”模型处理

        for (int i = 0; i <= 6; i += 2) {
            if (stationary && i != 6) {
                x(i + 1) = 0;
                f.row(i + 1).setZero();
                q(i, i) = config_.outpost_center_q * dt;
                continue;
            }

            f(i, i + 1) = dt;
            double density = config_.q_xy;
            if (i == 6) {
                density = config_.q_yaw;

                if (stationary) {
                    density = config_.q_outpost_yaw;
                }
            } else if (i == 4) {
                density = config_.q_z;
            }

            q(i, i) = dt * dt * dt * density / 3;
            q(i, i + 1) = dt * dt * density / 2;
            q(i + 1, i) = q(i, i + 1);
            q(i + 1, i + 1) = dt * density;
        }

        if (geometry_observable) {
            for (int i = 8; i <= 11; i += 1) {
                q(i, i) = config_.q_radius * dt;
            }
        }

        x = f * x;

        x(6) = wrap(x(6));
        p = f * p * f.transpose() + q;
        observed.fill(false);
    }

    Eigen::Matrix<double, 4, 12> jacobian(int slot) const {
        Eigen::Matrix<double, 4, 12> h = Eigen::Matrix<double, 4, 12>::Zero();
        double a = yaw(slot);
        double r = radius(slot);
        int radius_index = 8;

        if (count == 4 && slot % 2) {
            radius_index = 9;
        }

        h(0, 0) = 1;
        h(1, 2) = 1;
        h(2, 4) = 1;
        h(3, 6) = 1;
        h(0, 6) = r * std::sin(a);
        h(1, 6) = -r * std::cos(a);

        if (count > 1) {
            h(0, radius_index) = -std::cos(a);
            h(1, radius_index) = -std::sin(a);
        }

        if (count == 4 && slot % 2) {
            h(2, 10) = 1;
        }

        if (count == 3 && slot > 0) {
            h(2, 9 + slot) = 1;
        }

        return h;
    }

    Eigen::Vector4d innovation(const Observation& o, int slot) const {
        cv::Vec3d pos = position(slot);
        return {o.position[0] - pos[0], o.position[1] - pos[1], o.position[2] - pos[2], wrap(o.yaw - yaw(slot))};
    }

    static Eigen::Matrix4d noise(const Observation& o) {
        Eigen::Matrix4d r = Eigen::Matrix4d::Zero();
        double radial = std::max(1e-6, o.position_variance);
        Eigen::Matrix3d position_noise = Eigen::Matrix3d::Identity() * radial;
        cv::Vec3d delta = o.position - o.view_origin;

        if (o.has_view && o.tangential_variance > 0 && cv::norm(delta) > 0.1) {
            double tangent = std::max(1e-6, o.tangential_variance);
            Eigen::Vector3d n(delta[0], delta[1], delta[2]);
            n.normalize();
            position_noise = tangent * Eigen::Matrix3d::Identity() + (radial - tangent) * n * n.transpose();
        }

        r.topLeftCorner<3, 3>() = position_noise;
        r(3, 3) = std::max(0.003, o.yaw_variance);
        return r;
    }

    void applyGeometryPrior(Covariance& covariance, int slot) const {
        if (count <= 1) {
            return;
        }

        if (!geometry_observable) {
            double variance = config_.initial_radius_sigma * config_.initial_radius_sigma;
            covariance(8, 8) = std::max(covariance(8, 8), variance);
            if (count == 4) {
                covariance(9, 9) = std::max(covariance(9, 9), variance);
            }
        }

        int height_index = -1;
        if (count == 3 && slot > 0 && !seen[slot]) {
            height_index = 9 + slot;
        } else if (count == 4 && slot % 2 && !seen[1] && !seen[3]) {
            height_index = 10;
        }

        if (height_index >= 0) {
            double variance = config_.initial_height_sigma * config_.initial_height_sigma;
            covariance(height_index, height_index) = std::max(covariance(height_index, height_index), variance);
        }
    }

    typedef Eigen::Matrix<double, 8, 1> Pixels;

    typedef Eigen::Matrix<double, 8, 12> PixelJacobian;

    typedef Eigen::Matrix<double, 8, 8> PixelCovariance;

    //重投影
    bool projectPixels(const State& value, const Observation& o, int slot, Pixels& projected) const {
        double angle = value(6) + slot * 2 * CV_PI / count;
        double r = value(count == 4 && slot % 2 ? 9 : 8);
        double z = value(4);

        if (count == 3 && slot > 0) {
            z += value(9 + slot);
        } else if (count == 4 && slot % 2) {
            z += value(10);
        }

        if (count == 1) {
            r = 0;
        }

        cv::Vec3d center(value(0) - r * std::cos(angle), value(2) - r * std::sin(angle), z);
        std::array<cv::Vec3d, 4> corners = armorCorners(center, angle, width, height, tilt);
        std::vector<cv::Point3d> camera_points;

        for (int i = 0; i < 4; ++i) {
            cv::Vec3d point = o.camera_rotation.t() * (corners[i] - o.view_origin);
            if (!finite(point) || point[2] <= 0.01) {
                return false;
            }

            camera_points.emplace_back(point[0], point[1], point[2]);
        }

        std::vector<cv::Point2d> pixels;
        cv::projectPoints(camera_points, cv::Vec3d(), cv::Vec3d(), o.camera_matrix, o.distortion, pixels);
        for (int i = 0; i < 4; ++i) {
            projected(2 * i) = pixels[i].x;
            projected(2 * i + 1) = pixels[i].y;
        }

        return projected.allFinite();
    }

    bool pixelMeasurement(const Observation& o, int slot, Pixels& residual, PixelJacobian& h) const {
        Pixels projected;
        if (!projectPixels(x, o, slot, projected)) {
            return false;
        }

        for (int i = 0; i < 4; ++i) {
            residual(2 * i) = o.image_points[i].x - projected(2 * i);
            residual(2 * i + 1) = o.image_points[i].y - projected(2 * i + 1);
        }

        h.setZero();
        int indices[] = {0, 2, 4, 6, 8, 9, 10, 11};
        const double step = 0.0001;

        for (int index : indices) {
            State plus = x;
            State minus = x;
            plus(index) += step;
            minus(index) -= step;
            Pixels upper;
            Pixels lower;

            if (!projectPixels(plus, o, slot, upper) || !projectPixels(minus, o, slot, lower)) {
                return false;
            }

            h.col(index) = (upper - lower) / (2 * step);
        }

        return residual.allFinite() && h.allFinite();
    }

    bool usePixels(const Observation& o, int slot) const {
        return config_.use_pixel_update && o.has_pixels && geometry_observable && seen[slot];
    }

    PixelCovariance pixelNoise(const Observation& o) const {
        double variance = config_.pixel_sigma * config_.pixel_sigma;
        double range = cv::norm(o.position - o.view_origin);
        if (o.has_view && std::isfinite(range) && range > 0.1 && std::isfinite(o.tangential_variance) &&
            o.tangential_variance > 0) {
            double focal = std::max(o.camera_matrix(0, 0), o.camera_matrix(1, 1));
            variance = std::max(variance, o.tangential_variance * focal * focal / (range * range));
        }

        return PixelCovariance::Identity() * variance;
    }

    double pixelCost(const Observation& o, int slot) const {
        Pixels residual;
        PixelJacobian h;
        if (!pixelMeasurement(o, slot, residual, h)) {
            return INFINITY;
        }

        PixelCovariance r = pixelNoise(o);
        PixelCovariance s = h * p * h.transpose() + r;
        Eigen::LDLT<PixelCovariance> solve(s);

        if (solve.info() != Eigen::Success || !solve.isPositive()) {
            return INFINITY;
        }

        double value = residual.dot(solve.solve(residual));
        if (!std::isfinite(value) || value >= config_.pixel_gate) {
            return INFINITY;
        }

        return value + (slot != last_slot ? 0.15 : 0);
    }

    bool updatePixels(const Observation& o, int slot) {
        Pixels residual;
        PixelJacobian h;
        if (!pixelMeasurement(o, slot, residual, h)) {
            return false;
        }

        PixelCovariance r = pixelNoise(o);
        PixelCovariance s = h * p * h.transpose() + r;
        Eigen::LDLT<PixelCovariance> solve(s);

        if (solve.info() != Eigen::Success || !solve.isPositive()) {
            return false;
        }

        Eigen::Matrix<double, 12, 8> gain = solve.solve((p * h.transpose()).transpose()).transpose();
        if (count == 3 && config_.stationary_outpost) {
            for (int index = 1; index <= 5; index += 2) {
                gain.row(index).setZero();
            }
        }

        bool slow_center = count == 3 && config_.stationary_outpost && state == "TRACKING" &&
                           last_seen - created_at >= config_.outpost_center_warmup_s &&
                           std::max({p(0, 0), p(2, 2), p(4, 4)}) <=
                               config_.outpost_center_max_sigma_m * config_.outpost_center_max_sigma_m &&
                           innovation(o, slot).head<3>().norm() <= config_.outpost_center_max_residual_m;
        if (slow_center) {
            gain.row(0) *= config_.outpost_center_gain_scale;
            gain.row(2) *= config_.outpost_center_gain_scale;
            gain.row(4) *= config_.outpost_center_gain_scale;
        }

        State next = x + gain * residual;
        next(6) = wrap(next(6));
        Covariance correction = Covariance::Identity() - gain * h;
        Covariance covariance = correction * p * correction.transpose() + gain * r * gain.transpose();
        covariance = (covariance + covariance.transpose()).eval() * 0.5;

        if (!next.allFinite() || !covariance.allFinite() ||
            (count > 1 && (next(8) < config_.min_radius || next(8) > config_.max_radius)) ||
            (count == 4 && (next(9) < config_.min_radius || next(9) > config_.max_radius)) ||
            std::abs(next(10)) > config_.max_height_difference || std::abs(next(11)) > config_.max_height_difference) {
            return false;
        }

        x = next;
        p = covariance;
        if (count == 3) {
            x(9) = x(8);
        }

        observed[slot] = true;
        last_slot = slot;
       
        return true;
    }

    double primaryCost(const Observation& o, int slot) const {
        if (!validArmorIdentity(o.number)) {
            return INFINITY;
        }

        if (number > 0 && o.number > 0 && number != o.number) {
            return INFINITY;
        }

        if (std::abs(o.width - width) > width * 0.25) {
            return INFINITY;
        }

        if (usePixels(o, slot)) {
            cv::Vec3d direction = position(slot) - o.view_origin;
            if (direction.dot(cv::Vec3d(std::cos(yaw(slot)), std::sin(yaw(slot)), 0)) <= 0) {
                return INFINITY;
            }

            return pixelCost(o, slot);
        }

        Eigen::Vector4d v = innovation(o, slot);
        if (v.head<3>().norm() > config_.match_distance || std::abs(v(3)) > config_.match_yaw) {
            return INFINITY;
        }

        if (o.has_view) {
            cv::Vec3d direction = position(slot) - o.view_origin;
            if (direction.dot(cv::Vec3d(std::cos(yaw(slot)), std::sin(yaw(slot)), 0)) < 0) {
                return INFINITY;
            }
        }

        Eigen::Matrix<double, 4, 12> h = jacobian(slot);
        Covariance association_covariance = p;
        applyGeometryPrior(association_covariance, slot);
        Eigen::Matrix4d s = h * association_covariance * h.transpose() + noise(o);
        Eigen::LDLT<Eigen::Matrix4d> solve(s);

        if (solve.info() != Eigen::Success || !solve.isPositive()) {
            return INFINITY;
        }

        double value = v.dot(solve.solve(v));
        if (!(std::isfinite(value) && value < config_.mahalanobis_gate)) {
            return INFINITY;
        }

        double slot_penalty = 0;
        if (slot != last_slot) {
            slot_penalty = 0.15;
        }

        return value + slot_penalty;
    }

    Observation chooseHypothesis(const Observation& o, int slot) const {
        if (!o.has_alternative) {
            return o;
        }

        Observation alternative = o;
        alternative.position = o.alternative_position;
        alternative.yaw = o.alternative_yaw;
        alternative.has_alternative = false;

        if (primaryCost(alternative, slot) < primaryCost(o, slot)) {
            return alternative;
        }

        return o;
    }

    double cost(const Observation& o, int slot) const {
        return primaryCost(chooseHypothesis(o, slot), slot);
    }

    //用当前观测修正预测
    bool update(const Observation& raw, int slot) {
        if (slot < 0 || slot >= count || !std::isfinite(cost(raw, slot))) {
            return false;
        }

        if (usePixels(raw, slot)) {
            return updatePixels(raw, slot);
        }

        Observation o = chooseHypothesis(raw, slot);
        bool new_slot = !seen[slot];
        Covariance prior = p;//更新前的状态协方差，描述状态有多不确定
        bool observable_before = geometry_observable;

        if (new_slot) {
            applyGeometryPrior(prior, slot);
        }

        Eigen::Matrix<double, 4, 12> h = jacobian(slot);//观测模型的雅可比矩阵，描述状态变化如何影响装甲板观测
        Eigen::Matrix4d r = noise(o);//观测噪声协方差，描述观测有多不确定
        Eigen::Vector4d v = innovation(o, slot);//残差，观测到的减去预测的装甲板位置，朝向

        r(3, 3) *= std::max(1.0, std::pow(std::abs(v(3)) / 0.25, 2));
        Eigen::Matrix4d s = h * prior * h.transpose() + r;
        Eigen::LDLT<Eigen::Matrix4d> solve(s);

        if (solve.info() != Eigen::Success || !solve.isPositive()) {
            return false;
        }

        //卡尔曼增益
        Eigen::Matrix<double, 12, 4> k = solve.solve((prior * h.transpose()).transpose()).transpose();
        bool next_geometry_observable = geometry_observable;
        int seen_count = std::count(seen.begin(), seen.begin() + count, true);

        if (new_slot) {
            ++seen_count;
        }

        next_geometry_observable = next_geometry_observable || (seen_count >= 2 && count > 1);
        if (!next_geometry_observable) {
            k.bottomRows<4>().setZero();
        }

        if (count == 3 && config_.stationary_outpost) {
            for (int i = 1; i <= 5; i += 2) {
                k.row(i).setZero();
            }
        }

        bool slow_center =
            count == 3 && config_.stationary_outpost && observable_before && !new_slot && state == "TRACKING" &&
            last_seen - created_at >= config_.outpost_center_warmup_s &&
            std::max({prior(0, 0), prior(2, 2), prior(4, 4)}) <= std::pow(config_.outpost_center_max_sigma_m, 2) &&
            v.head<3>().norm() <= config_.outpost_center_max_residual_m;
        if (slow_center) {
            k.row(0) *= config_.outpost_center_gain_scale;
            k.row(2) *= config_.outpost_center_gain_scale;
            k.row(4) *= config_.outpost_center_gain_scale;
        }

        State next_x = x + k * v;//x是12行1列的向量
        next_x(6) = wrap(next_x(6));
        Covariance correction = Covariance::Identity() - k * h;
        Covariance next_p = correction * prior * correction.transpose() + k * r * k.transpose();
        next_p = (next_p + next_p.transpose()).eval() * 0.5;

        if (!next_x.allFinite() || !next_p.allFinite()) {
            return false;
        }

        x = next_x;
        p = next_p;
        seen[slot] = true;
        geometry_observable = next_geometry_observable;

        if (count > 1) {
            x(8) = std::clamp(x(8), config_.min_radius, config_.max_radius);
            x(9) = std::clamp(x(9), config_.min_radius, config_.max_radius);
        }

        x(10) = std::clamp(x(10), -config_.max_height_difference, config_.max_height_difference);
        x(11) = std::clamp(x(11), -config_.max_height_difference, config_.max_height_difference);
        if (count != 4) {
            x(9) = x(8);
        }

        if (count == 4) {
            x(11) = 0;
        }

        if (count == 1) {
            x(10) = x(11) = 0;
        }

        if (number == 0 && o.number > 0) {
            number = o.number;
        }

        if (o.number > 0) {
            tilt += 0.05 * (o.tilt - tilt);
        }

        observed[slot] = true;
        last_slot = slot;
        return true;
    }

    void finishFrame(bool matched, double time) {
        if (matched) {
            last_seen = time;
            ++confirmed;
            missed_frames = 0;
            if (confirmed >= config_.confirmationFrames(number) &&
                time - created_at >= config_.confirmationTime(number)) {
                state = "TRACKING";
            } else {
                state = "DETECTING";
            }
        } else {
            ++missed_frames;

            const double timeout = state == "DETECTING" ? std::min(.15, config_.lost_timeout) : config_.lost_timeout;

            if (time - last_seen > timeout) {
                state = "LOST";
            } else if (state != "DETECTING") {
                state = "TEMP_LOST";
            }
        }

        if (!x.allFinite() || !p.allFinite() || std::abs(x(7)) > 25) {
            state = "LOST";
        }
    }

  private:
    TrackerConfig config_;
};

struct TrackerMatch {
    double cost;
    size_t target;// 车辆在 targets 数组中的下标
    size_t observation;// 观测在 observations 数组中的下标
    int slot;
};

inline bool trackerMatchCostLess(const TrackerMatch& a, const TrackerMatch& b) {
    return a.cost < b.cost;
}

class MultiTargetTracker {
  public:
    explicit MultiTargetTracker(TrackerConfig config = {}) : config_(config) {}

    std::vector<RigidTarget> targets;

    void reset() {
        targets.clear();
        last_time_ = -1;
    }

    void update(const std::vector<Observation>& observations, double time) {
        if (last_time_ >= 0 && time <= last_time_) {
            return;
        }

        double dt;
        if (last_time_ < 0) {
            dt = 0;
        } else {
            dt = time - last_time_;
        }

        if (dt > 0.5) {
            reset();
        }

        last_time_ = time;
        for (size_t target_index = 0; target_index < targets.size(); ++target_index) {
            RigidTarget& t = targets[target_index];
            t.predict(std::min(dt, 0.5));
        }

        std::vector<TrackerMatch> matches;
        for (size_t t = 0; t < targets.size(); ++t) {
            for (int s = 0; s < targets[t].count; ++s) {
                for (size_t o = 0; o < observations.size(); ++o) {
                    if (targets[t].number == 0 && observations[o].number > 0 &&
                        config_.armorCount(observations[o].number) != targets[t].count) {
                        continue;
                    }

                    double c = targets[t].cost(observations[o], s);
                    if (std::isfinite(c)) {
                        matches.push_back({c, t, o, s});
                    }
                }
            }
        }

        std::sort(matches.begin(), matches.end(), trackerMatchCostLess);
        std::vector<bool> used(observations.size(), false), matched(targets.size(), false);
        std::vector<std::array<bool, 4>> slots(targets.size(), {false, false, false, false});

        for (size_t match_index = 0; match_index < matches.size(); ++match_index) {
            const TrackerMatch& m = matches[match_index];
            if (!used[m.observation] && !slots[m.target][m.slot]) {
                if (targets[m.target].update(observations[m.observation], m.slot)) {
                    used[m.observation] = slots[m.target][m.slot] = matched[m.target] = true;
                }//用这个观测修正对应车辆的整车状态
            }//这个观测还没有被成功分配 这辆车的这个槽位还没有被成功占用
        }

        for (size_t i = 0; i < targets.size(); ++i) {
            targets[i].finishFrame(matched[i], time);
        }

        size_t target_index = 0;
        while (target_index < targets.size()) {
            if (targets[target_index].state == "LOST") {
                targets.erase(targets.begin() + target_index);
            } else {
                ++target_index;
            }
        }

        for (size_t i = 0; i < observations.size(); ++i) {
            if (!used[i]) {
                const Observation& o = observations[i];
                if (!validArmorIdentity(o.number) || !finite(o.position) || !std::isfinite(o.yaw) ||
                    (!config_.track_unknown && o.number == 0)) {
                    continue;
                }

                bool same_body = false;
                for (size_t target_index = 0; target_index < targets.size(); ++target_index) {
                    RigidTarget& t = targets[target_index];
                    double r = t.radius(0);
                    cv::Vec3d inferred = o.position + cv::Vec3d(r * std::cos(o.yaw), r * std::sin(o.yaw), 0);
                    cv::Vec3d center(t.x(0), t.x(2), t.x(4));

                    if (cv::norm(inferred - center) > 0.7) {
                        if (o.number > 0 && t.number == o.number)
                            same_body = true;
                        continue;
                    }

                    same_body = true;
                    int best = -1;
                    double c = INFINITY;

                    for (int s = 0; s < t.count; ++s) {
                        if (!t.observed[s] && t.cost(o, s) < c) {
                            c = t.cost(o, s);
                            best = s;
                        }
                    }

                    if (best >= 0 && t.last_seen == time) {
                        t.update(o, best);
                    }

                    break;
                }

                if (!same_body && int(targets.size()) < config_.max_tracks) {
                    targets.push_back(RigidTarget(next_id_++, o, time, config_));
                    targets.back().observed[0] = true;
                }
            }
        }
    }

  private:
    TrackerConfig config_;
    double last_time_ = -1;
    uint32_t next_id_ = 1;
};
} // namespace campus