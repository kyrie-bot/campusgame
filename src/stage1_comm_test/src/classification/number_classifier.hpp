#pragma once

#include "classification/armor_identity.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <array>
#include <cmath>
#include <stdexcept>

struct NumberResult {
    int number = 0;
    float confidence = 0;
    cv::Mat image;
};

class NumberClassifier {
  public:
    explicit NumberClassifier(const std::string& path) : net_(cv::dnn::readNetFromONNX(path)) {
        if (net_.empty()) {
            throw std::runtime_error("Cannot load number CNN: " + path);
        }

        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        predict(cv::Mat::zeros(20, 20, CV_8UC1));
    }//加载模型

    NumberResult classify(const cv::Mat& bgr, const std::array<cv::Point2f, 4>& points, bool large, float threshold) {
        NumberResult result;
        cv::Mat symbol_image = extractSymbolImage(bgr, points, large);
        if (!symbol_image.empty()) {
            std::pair<int, float> symbol = predict(symbol_image);

            if (campus::symbolArmorIdentity(symbol.first) && symbol.second >= threshold) {
                result.number = symbol.first;
                result.confidence = symbol.second;
                result.image = symbol_image;
                return result;
            }
        }

        result.image = extractNumberImage(bgr, points, large);
        if (result.image.empty()) {
            return result;
        }

        std::pair<int, float> prediction = predict(result.image);
        result.confidence = prediction.second;
        if (result.confidence >= threshold) {
            result.number = prediction.first;
        }

        return result;
    }

    std::pair<int, float> predict(const cv::Mat& gray) {
        if (gray.empty() || gray.type() != CV_8UC1) {
            throw std::invalid_argument("CNN input must be grayscale");
        }

        net_.setInput(cv::dnn::blobFromImage(gray, 1.0 / 255, {20, 20}, cv::Scalar(), false, false, CV_32F));
        cv::Mat logits = net_.forward().reshape(1, 1);//算出8个类别分数
        if (logits.total() != 8 || logits.type() != CV_32F || !cv::checkRange(logits)) {
            throw std::runtime_error("CNN must output 8 finite logits");
        }

        double maximum;
        cv::Point index;
        cv::minMaxLoc(logits, nullptr, &maximum, nullptr, &index);
        cv::Mat probabilities;
        cv::exp(logits - maximum, probabilities);

        return {index.x + 1, float(1.0 / cv::sum(probabilities)[0])};//最大分数的位置加一得到编号
    }//执行 CNN 识别

    static cv::Mat extractNumberImage(const cv::Mat& bgr, const std::array<cv::Point2f, 4>& points, bool large) {
        cv::Mat roi = warpNumberRegion(bgr, points, large);
        if (roi.empty()) {
            return cv::Mat();
        }

        cv::Mat gray;
        cv::Mat binary;
        int roi_width = roi.cols;
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Mat> channels;
        cv::split(roi, channels);
        cv::Mat smooth;
        cv::min(channels[0], channels[1], smooth);

        cv::min(smooth, channels[2], smooth);
        cv::GaussianBlur(smooth, smooth, {3, 3}, 0);
        double otsu = cv::threshold(smooth, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::threshold(smooth, binary, std::max(5.0, otsu), 255, cv::THRESH_BINARY);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::Rect box;

        for (size_t contour_index = 0; contour_index < contours.size(); ++contour_index) {
            const std::vector<cv::Point>& c = contours[contour_index];
            cv::Rect r = cv::boundingRect(c);
            if (cv::contourArea(c) < 8 || r.height < 5 || r.width > roi_width * 0.95 || r.height > 62 ||
                std::abs(r.x + r.width * 0.5 - roi_width * 0.5) > roi_width * 0.4) {
                continue;
            }

            if (box.empty()) {
                box = r;
            } else {
                box = box | r;
            }
        }

        if (box.empty() || box.height < 10) {
            return cv::Mat();
        }

        box = cv::Rect(box.x - 3, box.y - 3, box.width + 6, box.height + 6) & cv::Rect(0, 0, gray.cols, gray.rows);
        cv::Mat number;
        cv::resize(gray(box), number, {20, 20}, 0, 0, cv::INTER_LINEAR);
        cv::Scalar mean, deviation;//deviation灰度标准差
        cv::meanStdDev(number, mean, deviation);

        if (deviation[0] >= 3) {
            return number;
        }

        return cv::Mat();
    }

    static cv::Mat extractSymbolImage(const cv::Mat& bgr, const std::array<cv::Point2f, 4>& points, bool large) {
        cv::Mat region = warpNumberRegion(bgr, points, large);
        if (region.empty()) {
            return cv::Mat();
        }

        //符号形状比较分散，不按照数字的裁剪方式
        cv::Mat roi = region(cv::Rect(3, 8, region.cols - 6, 48));
        std::vector<cv::Mat> channels;
        cv::split(roi, channels);
        cv::Mat neutral;
        cv::min(channels[0], channels[1], neutral);
        cv::min(neutral, channels[2], neutral);
        cv::Scalar mean, deviation;
        cv::meanStdDev(neutral, mean, deviation);

        if (deviation[0] < 6) {
            return cv::Mat();
        }

        cv::Mat mask, smooth;
        cv::GaussianBlur(neutral, smooth, {3, 3}, 0);
        double otsu = cv::threshold(smooth, mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::threshold(smooth, mask, std::max(5.0, otsu), 255, cv::THRESH_BINARY);
        double foreground = double(cv::countNonZero(mask)) / mask.total();

        if (foreground < 0.03 || foreground > 0.65) {
            return cv::Mat();
        }

        cv::Mat gray, sample;
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        cv::resize(gray, sample, {20, 20}, 0, 0, cv::INTER_LINEAR);
        return sample;
    }

  private:
    static cv::Mat warpNumberRegion(const cv::Mat& bgr, const std::array<cv::Point2f, 4>& points, bool large) {
        if (bgr.empty() || bgr.type() != CV_8UC3) {
            return cv::Mat();
        }

        int width = 64;
        int roi_width = 54;
        if (large) {
            width = 108;
            roi_width = 64;
        }

        std::array<cv::Point2f, 4> dst{{{0, 18}, {float(width - 1), 18}, {float(width - 1), 46}, {0, 46}}};
        cv::Mat warped;
        cv::warpPerspective(bgr, warped, cv::getPerspectiveTransform(points.data(), dst.data()), {width, 64});
        return warped(cv::Rect((width - roi_width) / 2, 0, roi_width, 64));
    }//透视校正，避免装甲板倾斜带来的影响

    cv::dnn::Net net_;
};