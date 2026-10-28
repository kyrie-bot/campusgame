#pragma once

#include "geometry/aim_geometry.hpp"

#include <algorithm>
#include <vector>

namespace campus {
struct DetectionConfig {
    int color = 0;
    int brightness = 120;
    int color_difference = 35;
    double min_light_length = 4;
    double min_ratio = 0.04;
    double max_ratio = 0.55;
    double max_tilt = 45;
    double min_length_ratio = 0.5;

    double max_angle_difference = 18;
    double max_center_offset = 0.5;
    double min_spacing = 0.7;
    double large_spacing = 3.3;
    double max_spacing = 5.5;
};

struct Light {
    cv::Point2f center;
    cv::Point2f top;
    cv::Point2f bottom;
    float length = 0;
    float width = 0;
    float angle = 0;
};

struct ArmorDetection {
    std::array<cv::Point2f, 4> points;
    cv::Point2f center;
    int left = 0;
    int right = 0;
    bool large = false;
    double score = 0;
};

inline bool lightCenterLess(const Light& a, const Light& b) {
    return a.center.x < b.center.x;
}

inline bool armorScoreGreater(const ArmorDetection& a, const ArmorDetection& b) {
    return a.score > b.score;
}

class ArmorDetectionEngine {
  public:
    explicit ArmorDetectionEngine(DetectionConfig config = {}) : config_(config) {}

    const cv::Mat& binary() const {
        return binary_;
    }

    const std::vector<Light>& lights() const {
        return lights_;
    }

    std::vector<ArmorDetection> detect(const cv::Mat& image) {
        lights_.clear();
        if (image.empty()) {
            binary_.release();
            return {};
        }

        CV_Assert(image.type() == CV_8UC3);
        std::vector<cv::Mat> ch;
        cv::split(image, ch);
        int own = 0;
        int other = 2;

        if (config_.color == 0) {
            own = 2;
            other = 0;
        }

        cv::Mat difference;
        cv::Mat color;
        cv::Mat bright;
        //颜色差分并二值化
        cv::subtract(ch[own], ch[other], difference);
        cv::threshold(difference, color, config_.color_difference, 255, cv::THRESH_BINARY);
        cv::dilate(color, color, cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3}));

        // Otsu 估计亮度阈值
        double otsu = cv::threshold(ch[own], bright, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::threshold(ch[own], bright, std::clamp(otsu, double(config_.brightness), 235.0), 255, cv::THRESH_BINARY);

        cv::bitwise_and(bright, color, binary_);
        cv::morphologyEx(binary_, binary_, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (size_t contour_index = 0; contour_index < contours.size(); ++contour_index) {
            const std::vector<cv::Point>& contour = contours[contour_index];
            if (contour.size() < 4 || cv::contourArea(contour) < 3) {
                continue;
            }

            Light l;
            bool coarse_valid = makeLight(cv::minAreaRect(contour), l);
            Light core;

            if (recoverBrightCore(contour, ch[own], difference, core)) {
                l = core;
            } else if (!coarse_valid) {
                continue;
            }

            cv::Rect roi = cv::boundingRect(contour) & cv::Rect(0, 0, image.cols, image.rows);
            if (cv::mean(difference(roi))[0] < config_.color_difference * 0.35) {
                continue;
            }

            lights_.push_back(l);
        }

        //灯条配对
        std::sort(lights_.begin(), lights_.end(), lightCenterLess);
        std::vector<ArmorDetection> candidates;
        for (size_t i = 0; i < lights_.size(); ++i) {
            for (size_t j = i + 1; j < lights_.size(); ++j) {
                const Light& a = lights_[i];
                const Light& b = lights_[j];
                double length = (a.length + b.length) * 0.5;

                double ratio = std::min(a.length, b.length) / std::max(a.length, b.length);
                //两个方向相加，再归一化，就得到它们的共同方向 axis。它沿着灯条的长边方向
                cv::Point2f axis = (a.bottom - a.top) / a.length + (b.bottom - b.top) / b.length;
                axis /= cv::norm(axis);
                cv::Point2f delta = b.center - a.center;
                double offset = std::abs(delta.dot(axis)) / length;
                //勾股定理算两根灯条间距
                double spacing =
                    std::sqrt(std::max(0.0, double(delta.dot(delta) - std::pow(delta.dot(axis), 2)))) / length;
                double angle = std::abs(a.angle - b.angle);

                if (ratio < config_.min_length_ratio || angle > config_.max_angle_difference ||
                    offset > config_.max_center_offset || spacing < config_.min_spacing ||
                    spacing > config_.max_spacing) {
                    continue;
                }

                std::vector<cv::Point2f> quad{a.top, b.top, b.bottom, a.bottom};
                bool contains = false;
                for (size_t m = i + 1; m < j; ++m) {
                    if (lights_[m].length > length * 0.5 && cv::pointPolygonTest(quad, lights_[m].center, false) >= 0) {
                        contains = true;
                    }//pointPolygonTest检查的是中间灯条的中心是否位于四边形内，并没有要求它整根都在里面
                }

                if (contains) {
                    continue;
                }

                ArmorDetection armor;
                armor.points = {a.top, b.top, b.bottom, a.bottom};
                armor.center = (a.center + b.center) * 0.5f;
                armor.left = i;
                armor.right = j;
                armor.large = spacing >= config_.large_spacing;
                armor.score = ratio * 2 - offset - angle / config_.max_angle_difference;
                candidates.push_back(armor);
            }
        }

        std::sort(candidates.begin(), candidates.end(), armorScoreGreater);
        std::vector<bool> used(lights_.size(), false);//used[k] 表示第 k 根灯条是否已经分配给某块装甲板
        std::vector<ArmorDetection> result;

        //贪心，按分数顺序，能选的就选
        for (size_t armor_index = 0; armor_index < candidates.size(); ++armor_index) {
            const ArmorDetection& a = candidates[armor_index];
            if (!used[a.left] && !used[a.right]) {
                result.push_back(a);
                used[a.left] = used[a.right] = true;
            }
        }

        return result;
    }

  private:
    bool makeLight(const cv::RotatedRect& rect, Light& light) const {
        cv::Point2f points[4];
        rect.points(points);
        double first = cv::norm(points[1] - points[0]);
        double second = cv::norm(points[2] - points[1]);
        light.center = rect.center;
        light.length = std::max(first, second);
        light.width = std::max(1.0, std::min(first, second));

        if (light.length < config_.min_light_length) {
            return false;
        }

        if (first < second) {
            light.top = (points[0] + points[1]) * 0.5f;
            light.bottom = (points[2] + points[3]) * 0.5f;
        } else {
            light.top = (points[1] + points[2]) * 0.5f;
            light.bottom = (points[3] + points[0]) * 0.5f;
        }

        if (light.top.y > light.bottom.y) {
            std::swap(light.top, light.bottom);
        }

        light.angle = std::atan2(light.bottom.x - light.top.x, light.bottom.y - light.top.y) * 180 / CV_PI;
        return light.width / light.length >= config_.min_ratio && light.width / light.length <= config_.max_ratio &&
               std::abs(light.angle) <= config_.max_tilt;
    }

    //筛选亮核心像素
    bool recoverBrightCore(const std::vector<cv::Point>& contour, const cv::Mat& dominant, const cv::Mat& difference,
                           Light& light) const {
        cv::Rect box = cv::boundingRect(contour) & cv::Rect(0, 0, dominant.cols, dominant.rows);
        if (box.height < config_.min_light_length || box.width > box.height * 1.8) {
            return false; 
        }

        cv::Mat support = cv::Mat::zeros(box.size(), CV_8UC1);
        std::vector<std::vector<cv::Point>> contours{contour};
        cv::drawContours(support, contours, 0, {255}, -1, cv::LINE_8, cv::noArray(), 0, -box.tl());
        cv::Mat colored;
        cv::threshold(difference(box), colored, config_.color_difference, 255, cv::THRESH_BINARY);
        cv::bitwise_and(support, colored, support);

        if (cv::countNonZero(support) < 4) {
            return false;
        }

        double peak;
        //在 dominant 图像的 box 区域内，只统计 support 掩码允许的像素，
        //然后找出这些像素中的最大值，保存到 peak 中， 找出最高亮度
        cv::minMaxLoc(dominant(box), nullptr, &peak, nullptr, nullptr, support);
        const double fractions[] = {0.72, 0.80};

        for (double fraction : fractions) {
            cv::Mat core;
            cv::threshold(dominant(box), core, std::max(double(config_.brightness), peak * fraction), 255,
                          cv::THRESH_BINARY);//既不能低于最低阈值，也要达到最高亮度的一定比例
            cv::bitwise_and(core, support, core);
            std::vector<cv::Point> pixels;
            cv::findNonZero(core, pixels);

            if (pixels.size() < 4) {
                continue;
            }

            if (fitBrightAxis(pixels, dominant(box), std::max(double(config_.brightness), peak * fraction), box.tl(),
                              light)) {
                return true;
            }
        }

        return false;
    }

    //用这些亮核心像素去估计灯条的形状
    bool fitBrightAxis(const std::vector<cv::Point>& pixels, const cv::Mat& dominant, double threshold,
                       const cv::Point& offset, Light& light) const {
        // Weighted second moments find the emitting axis without duplicating pixels or using a lambda.
        double total = 0, mean_x = 0, mean_y = 0;
        for (const cv::Point& point : pixels) {
            double weight = dominant.at<unsigned char>(point) - threshold;//权重等于“像素亮度超过阈值的部分”
            total += weight;
            mean_x += weight * point.x;
            mean_y += weight * point.y;
        }

        if (total <= 0) {
            return false;
        }

        mean_x /= total;
        mean_y /= total;
        double xx = 0, xy = 0, yy = 0;

        for (const cv::Point& point : pixels) {
            double weight = dominant.at<unsigned char>(point) - threshold;
            double dx = point.x - mean_x, dy = point.y - mean_y;
            xx += weight * dx * dx;
            xy += weight * dx * dy;
            yy += weight * dy * dy;
        }

        //协方差矩阵
        xx /= total;//像素在水平方向上分散得有多开
        xy /= total;//水平和竖直的变化是否存在关联，用来反映倾斜
        yy /= total;//像素在竖直方向上分散得有多开

        //计算二维协方差矩阵的主方向
        double angle = 0.5 * std::atan2(2 * xy, xx - yy);
        cv::Point2f axis(std::cos(angle), std::sin(angle));//axis是长度为1的向量

        if (axis.y < 0) {
            axis = -axis;
        }//统一让轴线朝下，方便后续区分上下端点

        double minimum = INFINITY, maximum = -INFINITY;
        for (const cv::Point& point : pixels) {
            //一个像素相对于加权中心，沿轴线方向走了多远
            double projection = (point.x - mean_x) * axis.x + (point.y - mean_y) * axis.y;
            minimum = std::min(minimum, projection);
            maximum = std::max(maximum, projection);
        }

        //计算的是垂直于灯条轴线方向的方差，即像素在灯条横向上分散得有多开
        double width_variance = axis.y * axis.y * xx - 2 * axis.x * axis.y * xy + axis.x * axis.x * yy;
        light.width = std::max(1.0, std::sqrt(std::max(0.0, 12 * width_variance)));//宽度为width的连续均匀分布
        light.length = maximum - minimum;//灯条长度
        cv::Point2f mean(mean_x + offset.x, mean_y + offset.y);
        light.top = mean + axis * float(minimum);
        light.bottom = mean + axis * float(maximum);
        light.center = (light.top + light.bottom) * 0.5f;
        light.angle = std::atan2(axis.x, axis.y) * 180 / CV_PI;

        //长度足够
        //宽长比处于允许范围内
        //相对竖直方向的倾斜程度不超过限制
        return light.length >= config_.min_light_length && light.width / light.length >= config_.min_ratio &&
               light.width / light.length <= config_.max_ratio && std::abs(light.angle) <= config_.max_tilt;
    }

    DetectionConfig config_;

    cv::Mat binary_;
    std::vector<Light> lights_;
};
} // namespace campus