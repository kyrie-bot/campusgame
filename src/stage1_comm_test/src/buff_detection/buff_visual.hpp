#pragma once
#include <opencv2/opencv.hpp>
#include "buff_r_center.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <filesystem>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

// 纯传统视觉基础版：自动寻找 R 标识 + 当前帧实际候选轮廓的中心。
// 不绘制五个理论槽位，不推测下一组点亮的扇叶。
// 注意：检测到的颜色轮廓不一定等于完整物理扇叶，也不等于可激活目标。
/*
    cd /home/tdt/Desktop/campusgame/src/stage1_comm_test
    ../../build/stage1_comm_test/buff_detector
*/

struct Candidate {
    cv::RotatedRect rect;
    cv::Point2f center;
    double area = 0.0;
    double ratio = 0.0;
    double fill_ratio = 0.0;
    double radius = 0.0;
    double angle = 0.0;
};

// 与原 Candidate 分开保存：这里的中心来自“目标圆环内部的暗孔”。
// 小黄框的四个 Key Points 从这个圆环及符叶方向重新构造，不使用 rect.points()。
struct RingTarget {
    cv::RotatedRect rect;  // 实际轮廓的 minAreaRect（普通暗孔/十字环外轮廓）
    cv::Point2f center;    // 实际轮廓的 moments 质心
    float radius = 0.0f;   // 对应轮廓的最小包围圆半径
    cv::Point2f direction; // 目标 -> 邻近符叶的单位方向
    std::array<cv::Point2f, 4> keypoints{};
    std::array<cv::Point2f, 4> extended{};
    bool pending = false; // 当前十字 Select 点亮，不含已完成的普通圆环
    double quality = 0.0; // 方向峰值与其他方向的对比度
};

struct Settings {
    // 保留原代码的青蓝色候选轮廓阈值。
    int h_min = 80;
    int h_max = 100;
    int s_min = 94;
    int v_min = 83;
    int min_area = 30;
    int max_area = 15000;
    int max_ratio_x100 = 115;
    int min_fill_percent = 25;
    int kernel_size = 3;

    // 圆环与局部取样框参数；R 搜索由当前圆环几何提供约束。
    int ring_circle_percent = 65;      // 内孔圆形度下限：65 表示 0.65
    int target_half_percent = 135;     // 小黄框半边长 / 暗孔半径，默认 1.35
    int extend_percent = 100;          // 朝符叶延伸长度 / 暗孔直径，默认 1.0
    int direction_ratio_percent = 175; // 最佳方向 / 次优方向的最小百分比
    int show_legacy = 0;               // 是否额外显示旧版外轮廓的紫框
    int show_points = 1;               // 是否标 K0..K3

    // R 标识使用独立阈值；根据提供的录屏设定的起始参数。
    int r_h_min = 80;
    int r_h_max = 100;
    int r_s_min = 94;
    int r_v_min = 83;
};

class BuffDetector {
public:
    explicit BuffDetector(bool display = true) : display_(display) {
        // Traditional Hu-shape template from the user's small_buff1.png R
        // contour. Reject arrow fragments before they can establish a prior.
        r_tracker_.setGlyphTemplate({{249, 214}, {251, 216}, {251, 217}, {252, 218}, {252, 220}, {250, 222}, {250, 223},
                                     {249, 224}, {249, 226}, {248, 227}, {247, 230}, {251, 230}, {252, 229}, {252, 227},
                                     {254, 224}, {257, 224}, {259, 226}, {259, 228}, {258, 229}, {257, 229}, {258, 229},
                                     {259, 230}, {261, 230}, {262, 231}, {263, 231}, {263, 230}, {264, 229}, {266, 229},
                                     {264, 227}, {264, 224}, {267, 221}, {267, 220}, {268, 219}, {268, 218}, {266, 215},
                                     {265, 215}, {264, 214}});
        if (!display_)
            return;
        cv::namedWindow("buff", cv::WINDOW_NORMAL);
        cv::namedWindow("mask", cv::WINDOW_NORMAL);
        cv::namedWindow("binary", cv::WINDOW_NORMAL);
        cv::namedWindow("R mask", cv::WINDOW_NORMAL);
        cv::namedWindow("settings", cv::WINDOW_NORMAL);

        cv::resizeWindow("buff", 1000, 650);
        cv::resizeWindow("mask", 640, 400);
        cv::resizeWindow("binary", 640, 400);
        cv::resizeWindow("R mask", 640, 400);
        cv::resizeWindow("settings", 520, 750);

        cv::createTrackbar("H min", "settings", &settings_.h_min, 179);
        cv::createTrackbar("H max", "settings", &settings_.h_max, 179);
        cv::createTrackbar("S min", "settings", &settings_.s_min, 255);
        cv::createTrackbar("V min", "settings", &settings_.v_min, 255);
        cv::createTrackbar("Area min", "settings", &settings_.min_area, 10000);
        cv::createTrackbar("Area max", "settings", &settings_.max_area, 100000);
        cv::createTrackbar("Ratio x100", "settings", &settings_.max_ratio_x100, 400);
        cv::createTrackbar("Fill percent", "settings", &settings_.min_fill_percent, 100);
        cv::createTrackbar("Kernel", "settings", &settings_.kernel_size, 15);

        cv::createTrackbar("Ring round %", "settings", &settings_.ring_circle_percent, 100);
        cv::createTrackbar("Target half %", "settings", &settings_.target_half_percent, 220);
        cv::createTrackbar("Extend by dia %", "settings", &settings_.extend_percent, 250);
        cv::createTrackbar("Dir contrast %", "settings", &settings_.direction_ratio_percent, 400);
        cv::createTrackbar("Show old contours", "settings", &settings_.show_legacy, 1);
        cv::createTrackbar("Show K0 K1 K2 K3", "settings", &settings_.show_points, 1);

        cv::createTrackbar("R H min", "settings", &settings_.r_h_min, 179);
        cv::createTrackbar("R H max", "settings", &settings_.r_h_max, 179);
        cv::createTrackbar("R S min", "settings", &settings_.r_s_min, 255);
        cv::createTrackbar("R V min", "settings", &settings_.r_v_min, 255);
    }

    // 返回绘制结果；R 必须有实际像素测量，几何只限定搜索范围。
    cv::Mat process(const cv::Mat& input, double fps, double timestamp = -1,
                    const cv::Point2f* tracked_hub = nullptr) {
        if (input.empty())
            return cv::Mat();

        cv::Mat show = input.clone();
        cv::Mat raw_mask = preprocess(input);
        cv::Mat clean_mask = morphMask(raw_mask);
        std::vector<Candidate> old_candidates = findCandidates(clean_mask);

        // RETR_TREE 同时支持普通圆环和带十字的亮起圆环，避免必须等到外观变化后才画框。
        std::vector<RingTarget> rings = findTargetRings(clean_mask);
        if (rings.size() < 2)
            addMultiRingEvidence(clean_mask, rings);

        cv::Mat r_mask = preprocessR(input);
        if (r_roi_.area() > 0) {
            cv::Mat limited = cv::Mat::zeros(r_mask.size(), r_mask.type());
            cv::Rect roi = r_roi_ & cv::Rect(0, 0, r_mask.cols, r_mask.rows);
            if (roi.area() > 0)
                r_mask(roi).copyTo(limited(roi));
            r_mask = limited;
        }

        std::vector<campus_buff::RingEvidence> evidence;

        for (const auto& ring : rings)
            evidence.push_back({ring.center, ring.direction, ring.radius});
        if (timestamp < 0)
            timestamp = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        r_result_ = r_tracker_.update(r_mask, evidence, timestamp);
        // Search all crosses using the CURRENT measured hub, even when an old
        // cross remains visible. Otherwise that old cross suppresses recovery
        // of a newly lit leaf whose thin stem has poor directional contrast.
        if (r_result_.observed || tracked_hub) {
            // Optional, short-lived reprojection of a previously VISUALLY
            // solved stationary hub. It helps find real rings, never fabricates R.
            auto recovered = findTargetRings(clean_mask, r_result_.observed ? &r_result_.center : tracked_hub);
            if (std::any_of(recovered.begin(), recovered.end(), [](const auto& r){ return r.pending; })) {
                rings = std::move(recovered);
                evidence.clear();
                for (const auto& ring : rings) evidence.push_back({ring.center, ring.direction, ring.radius});
                r_result_ = r_tracker_.update(r_mask, evidence, timestamp);
            }
        }
        rings_ = rings;
        bool r_detected = r_result_.observed, r_history = r_result_.held;
        cv::Point2f r_center = r_result_.center;

        // 可选显示你最初 findCandidates() 得到的外轮廓紫框与绿色质心。
        // 这部分算法和数据没有删除，但默认关闭叠加，防止遮挡新关键点。
        if (settings_.show_legacy) {
            for (const Candidate& c : old_candidates) {
                drawRotatedRect(show, c.rect, cv::Scalar(220, 80, 220), 1);
                cv::circle(show, c.center, 3, cv::Scalar(0, 190, 0), -1);
            }
        }

        for (size_t i = 0; i < rings.size(); ++i) {
            const RingTarget& t = rings[i];
            // 紫色 + 绿色：仍由真实 contour 的 minAreaRect 和 moments 计算。
            drawRotatedRect(show, t.rect, cv::Scalar(255, 0, 255), 2);
            cv::circle(show, t.center, 4, cv::Scalar(0, 255, 0), -1);

            // 黄色：K0..K3 是“沿符叶方向”的局部矩形角点，而非 rect.points()。
            drawQuad(show, t.keypoints, cv::Scalar(0, 255, 255), 2);
            // 青色：只朝符叶一侧，把 K2 和 K3 沿 direction 延长。
            drawQuad(show, t.extended, cv::Scalar(255, 255, 0), 2);
            // 橙色：远端短边，严格垂直于当前识别的符叶方向。
            cv::line(show, t.extended[2], t.extended[3], cv::Scalar(0, 165, 255), 2);
            if (settings_.show_points) {
                for (int j = 0; j < 4; ++j) {
                    cv::circle(show, t.keypoints[j], 3, cv::Scalar(0, 255, 255), -1);
                    cv::putText(show, "K" + std::to_string(j), t.keypoints[j] + cv::Point2f(5, -5),
                                cv::FONT_HERSHEY_SIMPLEX, 0.40, cv::Scalar(0, 255, 255), 1);
                }
            }
        }

        bool has_r = r_detected || r_history;

        if (has_r) {
            cv::drawMarker(show, r_center, r_detected ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 165, 255),
                           cv::MARKER_CROSS, 20, 2);
        }

        if (r_detected || r_history) {
            cv::Scalar color = r_detected ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 165, 255);
            // cv::drawMarker(show, r_center, color, cv::MARKER_CROSS, 20, 2);
            cv::putText(show, r_detected ? "R: MEASURED" : "R: HELD (NOT MEASURED)", r_center + cv::Point2f(14, -12),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1);
        } else {
            cv::putText(show, "R: NOT FOUND", cv::Point(12, 80), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                        cv::Scalar(0, 165, 255), 2);
        }

        std::ostringstream status;
        status << "Observed rings: " << rings.size() << "  Input FPS: " << std::fixed << std::setprecision(1) << fps;
        cv::putText(show, status.str(), cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.62, cv::Scalar(255, 255, 255),
                    2);
        cv::putText(show, "Yellow: semantic target K0..K3 | Cyan: short leaf extension", cv::Point(12, 55),
                    cv::FONT_HERSHEY_SIMPLEX, 0.46, cv::Scalar(255, 255, 255), 1);

        if (display_) {
            cv::imshow("buff", show);
            cv::imshow("mask", raw_mask);
            cv::imshow("binary", clean_mask);
            cv::imshow("R mask", r_mask);
        }

        return show;
    }

    const campus_buff::RCenterResult& centerResult() const {
        return r_result_;
    }

    const std::vector<RingTarget>& ringResults() const {
        return rings_;
    }

    void reset() {
        r_tracker_.reset();
        r_result_ = {};
        rings_.clear();
    }

    void setRed(bool red) {
        red_ = red;
    }

    void setHubSearch(const cv::Rect& roi) {
        r_roi_ = roi;
    }

private:
    cv::Rect r_roi_;
    bool red_ = false;
    bool display_;
    Settings settings_;

    campus_buff::RCenterTracker r_tracker_;

    campus_buff::RCenterResult r_result_;
    std::vector<RingTarget> rings_;

    static double normalize360(double angle) {
        angle = std::fmod(angle, 360.0);
        return angle < 0.0 ? angle + 360.0 : angle;
    }

    static void drawRotatedRect(cv::Mat& image, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness) {
        cv::Point2f pts[4];
        rect.points(pts);

        for (int i = 0; i < 4; ++i)
            cv::line(image, pts[i], pts[(i + 1) % 4], color, thickness);
    }

    // 连线绘制四边形，不依赖 RotatedRect 的角度与角点编号。
    static void drawQuad(cv::Mat& image, const std::array<cv::Point2f, 4>& q, const cv::Scalar& color, int thickness) {
        for (int i = 0; i < 4; ++i)
            cv::line(image, q[i], q[(i + 1) % 4], color, thickness);
    }

    // 从圆环内孔出发，在距中心 1.4r～3r 的区域统计 HSV 亮边。
    // 关键：1.4r 内的像素全部跳过，否则圆环自己会产生多个错误方向峰。
    // 对 72 个五度角箱做平滑，只接受存在明显“唯一主方向”的圆环。
    bool estimateLeafDirection(const cv::Mat& mask, cv::Point2f center, float radius, cv::Point2f& direction,
                               double& quality) const {
        constexpr int bins = 72;
        std::array<double, bins> histogram{}; // 原始角度直方图
        std::array<double, bins> smooth{};    // 经过加权平滑后的角度直方图
        float r_min = radius * 1.40f;
        float r_max = radius * 3.0f;
        float min2 = r_min * r_min;
        float max2 = r_max * r_max;
        int x0 = std::max(0, static_cast<int>(center.x - r_max));
        int x1 = std::min(mask.cols - 1, static_cast<int>(center.x + r_max));
        int y0 = std::max(0, static_cast<int>(center.y - r_max));
        int y1 = std::min(mask.rows - 1, static_cast<int>(center.y + r_max));

        for (int y = y0; y <= y1; ++y) {
            const uchar* row = mask.ptr<uchar>(y); // 获取第y行的像素数据地址
            for (int x = x0; x <= x1; ++x) {
                if (row[x] == 0)
                    continue;
                float dx = x - center.x;
                float dy = y - center.y;
                float d2 = dx * dx + dy * dy;
                if (d2 < min2 || d2 > max2)
                    continue;
                float angle = static_cast<float>(std::atan2(dy, dx));
                if (angle < 0.0f)
                    angle += 2.0f * static_cast<float>(CV_PI);
                int index = std::min(bins - 1, static_cast<int>(angle * bins / (2.0f * static_cast<float>(CV_PI))));
                histogram[index] += 1.0;
            }
        }

        // 对角度直方图卷积 [1,2,3,4,3,2,1]；跨越 0° 时也正常连接。
        for (int i = 0; i < bins; ++i) {
            for (int offset = -3; offset <= 3; ++offset) {
                int k = (i + offset + bins) % bins;
                smooth[i] += histogram[k] * (4 - std::abs(offset));
            }
        }

        int best_index = static_cast<int>(std::max_element(smooth.begin(), smooth.end()) - smooth.begin());
        // max_element 返回的是一个迭代器，指向 smooth 数组中最大的元素，再减去smooth.begin()得到最大元素的下标
        double best = smooth[best_index];
        double second = 0.0;
        // 跳过主峰附近 ±45°：相邻采样箱不是“另一条符叶”。
        for (int i = 0; i < bins; ++i) {
            int d = std::abs(i - best_index);
            d = std::min(d, bins - d);
            if (d > 9)
                second = std::max(second, smooth[i]);
        }
        quality = best / std::max(1.0, second);
        // 同时检查强度和唯一性：排除圆环本身、R 标和孤立背景点。
        if (best < radius * 38.0f)
            return false;
        if (quality < std::max(1.1, settings_.direction_ratio_percent / 100.0))
            return false;

        // 用主峰附近像素进行圆周加权平均，避免每 5° 离散跳变。
        double sum_x = 0.0;
        double sum_y = 0.0;

        for (int offset = -3; offset <= 3; ++offset) {
            int i = (best_index + offset + bins) % bins;
            double rad = (i + 0.5) * 2.0 * CV_PI / bins;
            double w = smooth[i];
            sum_x += std::cos(rad) * w;
            sum_y += std::sin(rad) * w;
        }
        // 单位方向向量
        double length = std::hypot(sum_x, sum_y);
        if (length < 1e-6)
            return false;
        direction = cv::Point2f(static_cast<float>(sum_x / length), static_cast<float>(sum_y / length));
        return true;
    }

    // 对一个轮廓尝试生成目标：位置来自真实 contour，方向来自附近的符叶亮边。
    // 无论是普通圆环还是十字圆环，最终都共用 K0～K3 的计算方式。
    bool appendRingTarget(const cv::Mat& mask, const std::vector<cv::Point>& contour,
                          std::vector<RingTarget>& results, const cv::Point2f* measured_hub = nullptr) const {
        cv::Moments m = cv::moments(contour);
        if (std::abs(m.m00) < 1e-6)
            return false;
        cv::Point2f center(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
        cv::Point2f circle_center;
        float radius = 0.0f;
        cv::minEnclosingCircle(contour, circle_center, radius);
        if (radius < 9.0f || radius > 90.0f)
            return false;

        // 避免普通圆环内孔和十字圆环外轮廓重复生成同一个目标。
        for (const RingTarget& old : results)
            if (cv::norm(old.center - center) < std::max(old.radius, radius) * 1.15f)
                return false;

        cv::Point2f direction;
        double quality = 0.0;
        if (measured_hub) {
            direction = *measured_hub - center;
            double length = cv::norm(direction);
            if (length < 3 * radius || length > 9 * radius) return false;
            direction *= 1.0 / length;
        } else if (!estimateLeafDirection(mask, center, radius, direction, quality))
            return false;

        RingTarget t;
        t.rect = cv::minAreaRect(contour);
        t.center = center;
        t.radius = radius;
        t.direction = direction;
        t.quality = quality;
        t.keypoints = makeTargetKeypoints(center, radius, direction);
        t.extended = extendKeypoints(t.keypoints, radius, direction);
        results.push_back(t);
        return true;
    }

    // 两种目标都要识别：
    // A. 普通亮环：内部只有一个接近圆形的大暗孔（原有逻辑）。
    // B. 十字亮环：中心亮线将暗孔分成多个小孔（新增逻辑）。
    // B 不能简单降低 A 的圆形度阈值，否则会把任意小亮斑误认成目标。
    std::vector<RingTarget> findTargetRings(const cv::Mat& mask, const cv::Point2f* measured_hub = nullptr) const {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        std::vector<RingTarget> results;
        if (contours.empty() || hierarchy.empty())
            return results;

        // 第一遍：普通圆环的内孔。保留上一版的检测逻辑。
        for (size_t i = 0; i < contours.size(); ++i) {
            if (hierarchy[i][3] < 0)
                continue;
            const auto& contour = contours[i];
            double area = cv::contourArea(contour);
            if (area < 120.0 || area > 14000.0 || contour.size() < 8)
                continue;
            double perimeter = cv::arcLength(contour, true);
            if (perimeter < 1.0)
                continue;
            double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
            if (circularity < settings_.ring_circle_percent / 100.0)
                continue;
            cv::RotatedRect rect = cv::minAreaRect(contour);
            float w = rect.size.width, h = rect.size.height;
            if (std::min(w, h) < 6.0f || std::max(w, h) > 1.30f * std::min(w, h))
                continue;
            appendRingTarget(mask, contour, results);
        }

        // 第二遍：刚亮起时带十字结构的圆环。
        // 它的外轮廓近似圆形，但内孔被十字亮线分成了至少三个区域。
        for (size_t i = 0; i < contours.size(); ++i) {
            if (hierarchy[i][3] >= 0 || hierarchy[i][2] < 0)
                continue;
            const auto& contour = contours[i];
            double area = cv::contourArea(contour);
            if (area < 220.0 || area > 14000.0 || contour.size() < 8)
                continue;
            double perimeter = cv::arcLength(contour, true);
            if (perimeter < 1.0)
                continue;
            double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
            if (circularity < 0.46)
                continue;
            cv::RotatedRect rect = cv::minAreaRect(contour);
            float w = rect.size.width, h = rect.size.height;
            if (std::min(w, h) < 18.0f || std::max(w, h) > 1.45f * std::min(w, h))
                continue;

            // hierarchy[i][2] 是首个子轮廓；hierarchy[child][0] 是下一个兄弟轮廓。
            int holes = 0;
            double hole_area = 0.0;

            for (int child = hierarchy[i][2]; child >= 0; child = hierarchy[child][0]) {
                double a = cv::contourArea(contours[child]);
                // The same real cross has smaller holes after rectification
                // or at greater depth. Keep topology/area ratios, not a fixed
                // 50-pixel hole threshold that drops freshly lit small leaves.
                if (a < std::max(8.0, area * .025))
                    continue;
                ++holes;
                hole_area += a;
            }

            if (holes < 3 || hole_area < area * 0.20)
                continue;
            // Cross topology is stronger evidence than an inner circular
            // hole found in pass one. Replace all nested ordinary candidates
            // belonging to this same physical target instead of suppressing
            // the actual pending target as a duplicate.
            std::vector<RingTarget> cross;
            if (appendRingTarget(mask, contour, cross, measured_hub)) {
                auto target = cross.front();
                target.pending = true;
                results.erase(std::remove_if(results.begin(), results.end(),
                                             [&](const auto& old) {
                                                 return cv::norm(old.center - target.center) <
                                                        std::max(old.radius, target.radius) * 1.15f;
                                             }),
                              results.end());
                results.push_back(target);
            }
        }

        return results;
    }

    // Multiple real circular holes give a coarse hub even when adjacent
    // leaves spoil the single-ring directional histogram. This fit is only
    // a search prior: the actual R must still be measured in the image.
    void addMultiRingEvidence(const cv::Mat& mask, std::vector<RingTarget>& rings) const {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        std::vector<RingTarget> holes;

        for (size_t i = 0; i < contours.size(); ++i) {
            if (hierarchy[i][3] < 0)
                continue;
            double area = cv::contourArea(contours[i]), perimeter = cv::arcLength(contours[i], true);
            if (area < 120 || area > 14000 || perimeter < 1 || 4 * CV_PI * area / (perimeter * perimeter) < .65)
                continue;
            auto rect = cv::minAreaRect(contours[i]);
            double lo = std::min(rect.size.width, rect.size.height), hi = std::max(rect.size.width, rect.size.height);
            if (lo < 12 || hi > lo * 1.4)
                continue;
            cv::Moments m = cv::moments(contours[i]);
            if (std::abs(m.m00) < 1e-6)
                continue;
            RingTarget t;
            t.center = {float(m.m10 / m.m00), float(m.m01 / m.m00)};
            t.rect = rect;
            cv::Point2f dummy;
            cv::minEnclosingCircle(contours[i], dummy, t.radius);
            if (t.radius >= 9 && t.radius <= 90)
                holes.push_back(t);
        }

        if (holes.size() < 3 || holes.size() > 64)
            return;
        std::vector<int> best;
        cv::Point2f best_center;

        for (size_t seed = 0; seed < holes.size(); ++seed) {
            std::vector<int> group;

            for (size_t j = 0; j < holes.size(); ++j) {
                double distance = cv::norm(holes[j].center - holes[seed].center), scale = holes[seed].radius;
                if (holes[j].radius < scale * .65 || holes[j].radius > scale * 1.5)
                    continue;
                if (j == seed || (distance > scale * 2.5 && distance < scale * 16))
                    group.push_back(j);
            }

            if (group.size() < 3)
                continue;
            cv::Mat a(int(group.size()), 3, CV_64F), b(int(group.size()), 1, CV_64F), solution;

            for (size_t j = 0; j < group.size(); ++j) {
                auto q = holes[group[j]].center;
                a.at<double>(j, 0) = 2 * q.x;
                a.at<double>(j, 1) = 2 * q.y;
                a.at<double>(j, 2) = 1;
                b.at<double>(j) = q.dot(q);
            }

            if (!cv::solve(a, b, solution, cv::DECOMP_SVD))
                continue;
            cv::Point2f center(float(solution.at<double>(0)), float(solution.at<double>(1)));
            double radius2 = solution.at<double>(2) + center.dot(center);
            if (!std::isfinite(radius2) || radius2 <= 0)
                continue;
            double radius = std::sqrt(radius2), scale = holes[seed].radius, error = 0;
            if (radius < scale * 3 || radius > scale * 10)
                continue;
            cv::Point2f mean{};

            for (int j : group) {
                error += std::abs(cv::norm(holes[j].center - center) - radius);
                mean += holes[j].center;
            }
            mean *= 1.f / group.size();
            if (error / group.size() > radius * .13 || cv::norm(mean - center) > radius * .65)
                continue;
            if (group.size() > best.size()) {
                best = group;
                best_center = center;
            }
        }

        if (best.size() < 3)
            return;

        for (int i : best) {
            auto t = holes[i];
            bool duplicate = false;

            for (const auto& existing : rings)
                if (cv::norm(existing.center - t.center) < t.radius)
                    duplicate = true;
            if (duplicate)
                continue;
            auto delta = best_center - t.center;
            double length = cv::norm(delta);
            if (length < 1)
                continue;
            t.direction = delta / float(length);
            t.quality = 0.;
            t.keypoints = makeTargetKeypoints(t.center, t.radius, t.direction);
            t.extended = extendKeypoints(t.keypoints, t.radius, t.direction);
            rings.push_back(t);
        }
    }

    // 四点均根据“圆环质心 + 符叶轴 u + 垂直轴 v”建立，而非 minAreaRect().points()。
    // 局部定义：K0/K1=远离符叶的端部，K2/K3=靠近符叶的端部。
    std::array<cv::Point2f, 4> makeTargetKeypoints(cv::Point2f center, float radius, cv::Point2f u) const {
        cv::Point2f v(-u.y, u.x);
        float half = radius * std::max(1.0f, settings_.target_half_percent / 100.0f);
        return {{center - u * half - v * half, center - u * half + v * half, center + u * half + v * half,
                 center + u * half - v * half}};
    }

    // 用户后续可以在此基础上自己计算短边中点和 R，当前仅负责扩展矩形。
    std::array<cv::Point2f, 4> extendKeypoints(std::array<cv::Point2f, 4> small, float radius, cv::Point2f u) const {
        float extension = 2.0f * radius * (settings_.extend_percent / 100.0f);
        small[2] += u * extension;
        small[3] += u * extension;
        return small;
    }

    cv::Mat preprocess(const cv::Mat& bgr) const {
        cv::Mat hsv, mask;
        cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
        int h0 = std::min(settings_.h_min, settings_.h_max);
        int h1 = std::max(settings_.h_min, settings_.h_max);
        if (red_) {
            cv::Mat a, b;
            cv::inRange(hsv, cv::Scalar(0, 94, 83), cv::Scalar(10, 255, 255), a);
            cv::inRange(hsv, cv::Scalar(170, 94, 83), cv::Scalar(179, 255, 255), b);
            return a | b;
        }

        cv::inRange(hsv, cv::Scalar(h0, settings_.s_min, settings_.v_min), cv::Scalar(h1, 255, 255), mask);
        return mask;
    }

    cv::Mat preprocessR(const cv::Mat& bgr) const {
        cv::Mat hsv, mask;
        cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
        int h0 = std::min(settings_.r_h_min, settings_.r_h_max);
        int h1 = std::max(settings_.r_h_min, settings_.r_h_max);
        if (red_) {
            cv::Mat a, b;
            cv::inRange(hsv, cv::Scalar(0, 94, 83), cv::Scalar(10, 255, 255), a);
            cv::inRange(hsv, cv::Scalar(170, 94, 83), cv::Scalar(179, 255, 255), b);
            mask = a | b;
        } else
            cv::inRange(hsv, cv::Scalar(h0, settings_.r_s_min, settings_.r_v_min), cv::Scalar(h1, 255, 255), mask);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        return mask;
    }

    cv::Mat morphMask(const cv::Mat& mask) const {
        int size = std::max(1, settings_.kernel_size);
        if (size % 2 == 0)
            ++size;

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(size, size));
        cv::Mat clean;
        cv::morphologyEx(mask, clean, cv::MORPH_CLOSE, kernel);
        return clean;
    }

    std::vector<Candidate> findCandidates(const cv::Mat& mask) const {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<Candidate> candidates;

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < settings_.min_area || area > settings_.max_area)
                continue;
            if (contour.size() < 4)
                continue;

            cv::RotatedRect rect = cv::minAreaRect(contour);
            double w = rect.size.width;
            double h = rect.size.height;
            if (w < 2.0 || h < 2.0)
                continue;

            double ratio = std::max(w, h) / std::min(w, h);
            if (ratio > std::max(1.0, settings_.max_ratio_x100 / 100.0))
                continue;

            double fill = area / (w * h);
            if (fill < settings_.min_fill_percent / 100.0)
                continue;

            cv::Moments m = cv::moments(contour); // 图像矩
                                                  // 如果传入的是图像而不是轮廓
                                                  // 对于图像中的像素：
                                                  //- 像素值为 0：不参与。
                                                  //- 像素值不为 0：按照权重 1 参与。
            cv::Point2f contour_center = rect.center;
            if (std::abs(m.m00) > 1e-6)
                contour_center = cv::Point2f(static_cast<float>(m.m10 / m.m00),
                                             static_cast<float>(m.m01 / m.m00)); // 算质心

            Candidate c;
            c.rect = rect;
            c.center = contour_center;
            c.area = area;
            c.ratio = ratio;
            c.fill_ratio = fill;
            candidates.push_back(c);
        }

        return candidates;
    }
};