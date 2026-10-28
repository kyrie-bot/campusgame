#pragma once

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace campus {
inline double wrap(double a) {
    return std::remainder(a, 2.0 * CV_PI);
}

inline bool finite(const cv::Vec3d& p) {
    return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

// Camera optical axes: right/down/forward. Map axes: forward/left/up.
struct CameraGeometry {
    cv::Matx33d rotation;

    cv::Vec3d origin;//相机光心在世界坐标系中的位置
    CameraGeometry(const cv::Vec3d& position_scene, double yaw_deg, double pitch_deg, double scene_scale = 3.0,
                   cv::Vec3d offset = {0, 0, 0}) {
        if (scene_scale <= 0.0 || !std::isfinite(scene_scale)) {
            throw std::invalid_argument("Invalid scene scale");
        }

        double y = yaw_deg * CV_PI / 180.0;
        double p = pitch_deg * CV_PI / 180.0;
        cv::Vec3d right(-std::sin(y), -std::cos(y), 0);
        cv::Vec3d down(-std::cos(y) * std::sin(p), std::sin(y) * std::sin(p), -std::cos(p));
        cv::Vec3d forward(std::cos(y) * std::cos(p), -std::sin(y) * std::cos(p), -std::sin(p));
        rotation = {right[0], down[0], forward[0], right[1], down[1], forward[1], right[2], down[2], forward[2]};
        origin = position_scene / scene_scale + rotation * offset;
    }

    cv::Vec3d toMap(const cv::Vec3d& p) const {
        return origin + rotation * p;
    }

    cv::Vec3d toCamera(const cv::Vec3d& p) const {
        return rotation.t() * (p - origin);
    }

    bool project(const cv::Vec3d& p, const cv::Matx33d& k, const cv::Mat& d, cv::Point2f& pixel) const {
        cv::Vec3d c = toCamera(p);
        if (!finite(c) || c[2] < 0.01) {
            return false;
        }

        std::vector<cv::Point2d> projected;
        cv::projectPoints(std::vector<cv::Point3d>{{c[0], c[1], c[2]}}, cv::Vec3d(), cv::Vec3d(), k, d, projected);
        if (!std::isfinite(projected[0].x) || !std::isfinite(projected[0].y) || std::abs(projected[0].x) > 1e6 ||
            std::abs(projected[0].y) > 1e6) {
            return false;
        }

        pixel = projected[0];
        return true;
    }
};

struct ArmorPose {
    bool valid = false;
    bool has_alternative = false;
    cv::Vec3d position{0, 0, 0};
    cv::Vec3d rvec{0, 0, 0};
    cv::Vec3d alternative_position{0, 0, 0};
    double yaw = 0;//装甲板的朝向角
    double error = INFINITY;
    double yaw_variance = 0.03;

    double alternative_yaw = 0;
    double tilt = 0;
};

struct PoseOptions {
    bool refine_yaw = false;
    double tilt = 0;
    double max_fit_penalty_px = 0.75;
    double image_noise_px = 1.5;
};

inline std::vector<cv::Point3d> armorObjectPoints(double width, double height) {
    return {{-width / 2, -height / 2, 0},
            {width / 2, -height / 2, 0},
            {width / 2, height / 2, 0},
            {-width / 2, height / 2, 0}};
}

inline cv::Matx33d armorRotation(double yaw, double tilt) {
    cv::Vec3d right(std::sin(yaw), -std::cos(yaw), 0);
    cv::Vec3d down(-std::sin(tilt) * std::cos(yaw), -std::sin(tilt) * std::sin(yaw), -std::cos(tilt));
    cv::Vec3d inward = right.cross(down);
    return {right[0], down[0], inward[0], right[1], down[1], inward[1], right[2], down[2], inward[2]};
}//根据装甲板的 Yaw 和倾斜角，重建装甲板在三维空间中的朝向

inline ArmorPose evaluateArmorPose(const cv::Matx33d& rotation, const cv::Vec3d& t,
                                   const std::vector<cv::Point3d>& object, const std::vector<cv::Point2d>& pixels,
                                   const cv::Matx33d& k, const cv::Mat& d, const CameraGeometry& camera) {
    ArmorPose pose;
    if (!finite(t) || t[2] <= 0.01 || cv::norm(t) > 80) {
        return pose;
    }

    for (size_t point_index = 0; point_index < object.size(); ++point_index) {
        const cv::Point3d& p = object[point_index];
        if ((rotation * cv::Vec3d(p.x, p.y, p.z) + t)[2] <= 0.01) {
            return pose;
        }
    }

    cv::Vec3d inward = rotation * cv::Vec3d(0, 0, 1);
    if (inward.dot(t) <= 0) {
        return pose;
    }

    cv::Vec3d normal = camera.rotation * inward;
    if (std::hypot(normal[0], normal[1]) < 0.2) {
        return pose;
    }

    cv::Rodrigues(rotation, pose.rvec);

    std::vector<cv::Point2d> projected;
    cv::projectPoints(object, pose.rvec, t, k, d, projected);
    double sum = 0;

    for (int j = 0; j < 4; ++j) {
        sum += (projected[j] - pixels[j]).dot(projected[j] - pixels[j]);
    }

    pose.error = std::sqrt(sum / 4);
    pose.valid = std::isfinite(pose.error);
    pose.position = camera.toMap(t);
    pose.yaw = std::atan2(normal[1], normal[0]);
    pose.tilt = std::asin(std::clamp(-normal[2], -1.0, 1.0));

    return pose;
}

inline bool armorPoseErrorLess(const ArmorPose& a, const ArmorPose& b) {
    return a.error < b.error;
}

inline ArmorPose fitArmorYaw(double yaw, double tilt, const std::vector<cv::Point3d>& object,
                             const std::vector<cv::Point2d>& normalized, const cv::Mat& inverse,
                             const std::vector<cv::Point2d>& pixels, const cv::Matx33d& k, const cv::Mat& d,
                             const CameraGeometry& camera) {
    cv::Matx33d rotation = camera.rotation.t() * armorRotation(yaw, tilt);//装甲板旋转矩阵转换到相机坐标系
    cv::Mat rhs(8, 1, CV_64F);
    for (int i = 0; i < 4; ++i) {
        cv::Vec3d r = rotation * cv::Vec3d(object[i].x, object[i].y, 0);
        rhs.at<double>(2 * i) = normalized[i].x * r[2] - r[0];
        rhs.at<double>(2 * i + 1) = normalized[i].y * r[2] - r[1];
    }

    cv::Mat t = inverse * rhs;//平移向量
    return evaluateArmorPose(rotation, {t.at<double>(0), t.at<double>(1), t.at<double>(2)}, object, pixels, k, d,
                             camera);
}

inline ArmorPose solveArmorPose(const std::array<cv::Point2f, 4>& points, double width, double height,
                                const cv::Matx33d& k, const cv::Mat& d, const CameraGeometry& camera, double max_error,
                                PoseOptions options = {}) {
    ArmorPose best;
    if (width <= 0 || height <= 0 || k(0, 0) <= 0 || k(1, 1) <= 0) {
        return best;
    }

    std::vector<cv::Point2d> pixels(points.begin(), points.end());
    if (std::abs(cv::contourArea(std::vector<cv::Point2f>(points.begin(), points.end()))) < 8) {
        return best;
    }

    std::vector<cv::Point3d> object = armorObjectPoints(width, height);

    std::vector<cv::Mat> rotations;

    std::vector<cv::Mat> translations;
    cv::solvePnPGeneric(object, pixels, k, d, rotations, translations, false, cv::SOLVEPNP_IPPE);
    std::vector<ArmorPose> candidates;
    double fitted_yaw_variance = 0;

    for (size_t i = 0; i < rotations.size(); ++i) {
        cv::Vec3d r;
        cv::Vec3d t;
        for (int j = 0; j < 3; ++j) {
            r[j] = rotations[i].at<double>(j);
            t[j] = translations[i].at<double>(j);
        }

        if (!finite(r)) {
            continue;
        }

        cv::Matx33d rotation;
        cv::Rodrigues(r, rotation);
        ArmorPose p = evaluateArmorPose(rotation, t, object, pixels, k, d, camera);

        if (p.valid) {
            candidates.push_back(p);
        }
    }

    if (candidates.empty()) {
        return best;
    }

    std::sort(candidates.begin(), candidates.end(), armorPoseErrorLess);
    if (options.refine_yaw) {
        std::vector<cv::Point2d> normalized;
        cv::undistortPoints(pixels, normalized, k, d);
        cv::Mat a = cv::Mat::zeros(8, 3, CV_64F);
        cv::Mat inverse;

        for (int i = 0; i < 4; ++i) {
            a.at<double>(2 * i, 0) = 1;
            a.at<double>(2 * i, 2) = -normalized[i].x;
            a.at<double>(2 * i + 1, 1) = 1;
            a.at<double>(2 * i + 1, 2) = -normalized[i].y;
        }

        cv::invert(a, inverse, cv::DECOMP_SVD);
        double bearing =
            std::atan2(candidates[0].position[1] - camera.origin[1], candidates[0].position[0] - camera.origin[0]);

        const int steps = 86;
        double spacing = 2 * CV_PI / 180;
        std::array<ArmorPose, steps> grid;

        for (int i = 0; i < steps; ++i) {
            grid[i] = fitArmorYaw(bearing + (i - 42.5) * spacing, options.tilt, object, normalized, inverse, pixels, k,
                                  d, camera);
        }

        std::vector<ArmorPose> refined;
        for (int i = 1; i < steps - 1; ++i) {
            if (!grid[i].valid || grid[i].error > grid[i - 1].error || grid[i].error > grid[i + 1].error) {
                continue;
            }

            double low = bearing + (i - 43.5) * spacing;
            double high = low + 2 * spacing;
            for (int n = 0; n < 14; ++n) {
                double l = low + (high - low) / 3;
                double r = high - (high - low) / 3;

                if (fitArmorYaw(l, options.tilt, object, normalized, inverse, pixels, k, d, camera).error <
                    fitArmorYaw(r, options.tilt, object, normalized, inverse, pixels, k, d, camera).error) {
                    high = r;
                } else {
                    low = l;
                }
            }

            ArmorPose p =
                fitArmorYaw((low + high) / 2, options.tilt, object, normalized, inverse, pixels, k, d, camera);
            if (p.valid) {
                refined.push_back(p);
            }
        }

        std::sort(refined.begin(), refined.end(), armorPoseErrorLess);
        
        if (!refined.empty() && refined[0].error <= candidates[0].error + options.max_fit_penalty_px &&
            refined[0].error <= max_error) {
            candidates = refined;
            
            double step = 0.01;
            double yaw = candidates[0].yaw;
            double cost = 4 * std::pow(candidates[0].error, 2);
            ArmorPose left = fitArmorYaw(yaw - step, options.tilt, object, normalized, inverse, pixels, k, d, camera),
                      right = fitArmorYaw(yaw + step, options.tilt, object, normalized, inverse, pixels, k, d, camera);
            double information =
                (4 * std::pow(left.error, 2) + 4 * std::pow(right.error, 2) - 2 * cost) / (2 * step * step);

            if (left.valid && right.valid && information > 1e-6) {
                fitted_yaw_variance =
                    std::clamp(options.image_noise_px * options.image_noise_px / information, 0.003, 0.5);
            }
        }
    }

    best = candidates.front();
    if (best.error > max_error || !finite(best.position)) {
        best.valid = false;
        return best;
    }

    best.yaw_variance = std::max(fitted_yaw_variance, std::pow((5 + 2 * best.error) * CV_PI / 180, 2));
    for (size_t i = 1; i < candidates.size(); ++i) {
        if (candidates[i].error > best.error + 0.75 || std::abs(wrap(candidates[i].yaw - best.yaw)) < 0.025) {
            continue;
        }

        //保存第二组可能的装甲板位姿，防止只通过重投影误差来判断
        best.has_alternative = true;
        best.alternative_position = candidates[i].position;
        best.alternative_yaw = candidates[i].yaw;
        best.yaw_variance = std::max(best.yaw_variance, std::pow(wrap(best.alternative_yaw - best.yaw), 2) * 0.25);
        break;
    }

    return best;
}

inline std::array<cv::Vec3d, 4> armorCorners(const cv::Vec3d& center, double yaw, double width, double height,
                                             double tilt = 0) {
    cv::Vec3d right(std::sin(yaw), -std::cos(yaw), 0);
    cv::Vec3d down(-std::sin(tilt) * std::cos(yaw), -std::sin(tilt) * std::sin(yaw), -std::cos(tilt));
    return {center - right * (width / 2) - down * (height / 2), center + right * (width / 2) - down * (height / 2),
            center + right * (width / 2) + down * (height / 2), center - right * (width / 2) + down * (height / 2)};
}//根据三维中心生成四个角点

inline bool drawArmor(cv::Mat& image, const CameraGeometry& camera, const cv::Matx33d& k, const cv::Mat& d,
                      const cv::Vec3d& center, double yaw, double width, double height, cv::Scalar color,
                      const std::string& label, bool dashed = false, double tilt = 0) {
    std::array<cv::Vec3d, 4> corners = armorCorners(center, yaw, width, height, tilt);
    std::array<cv::Point2f, 4> pixels;
    for (int i = 0; i < 4; ++i) {
        if (!camera.project(corners[i], k, d, pixels[i])) {
            return false;
        }
    }

    for (int i = 0; i < 4; ++i) {
        cv::Point2f a = pixels[i];
        cv::Point2f b = pixels[(i + 1) % 4];
        if (!dashed) {
            cv::line(image, a, b, color, 2, cv::LINE_AA);
        } else {
            for (int s = 0; s < 10; s += 2) {
                cv::line(image, a + (b - a) * (s / 10.0f), a + (b - a) * ((s + 1) / 10.0f), color, 1, cv::LINE_AA);
            }
        }
    }

    cv::putText(image, label, pixels[0], cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv::LINE_AA);
    return true;
}

} // namespace campus