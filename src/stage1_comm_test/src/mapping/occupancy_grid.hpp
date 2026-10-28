#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace campus {
// 格子按 [y][x] 存放，y=0 是图的下边，单位米。
class StaticOccupancyGrid {
  public:
    StaticOccupancyGrid(int width, int height, double resolution, double origin_x = 0.0, double origin_y = 0.0)
        : width_(width), height_(height), resolution_(resolution), origin_x_(origin_x), origin_y_(origin_y) {
        if (width <= 0 || height <= 0 || !std::isfinite(resolution) || resolution <= 0 || !std::isfinite(origin_x) ||
            !std::isfinite(origin_y) || static_cast<long long>(width) * height > 1000000) {
            throw std::invalid_argument("Invalid static grid dimensions");
        }

        cells_.assign(static_cast<std::size_t>(width) * height, -1);
    }

    void loadPgm(const std::string& path) {
        cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);
        if (image.empty() || image.cols != width_ || image.rows != height_) {
            throw std::runtime_error("Cannot load grid or wrong dimensions: " + path);
        }

        std::vector<int8_t> decoded(cells_.size(), -1);
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                unsigned char pixel = image.at<unsigned char>(height_ - 1 - y, x);
                int8_t value = -1;

                if (pixel == 0) {
                    value = 100;
                } else if (pixel == 254 || pixel == 255) {
                    value = 0;
                } else if (pixel != 205) {
                    throw std::runtime_error("Grid contains unsupported pixel values: " + path);
                }

                decoded[static_cast<std::size_t>(y) * width_ + x] = value;
            }
        }

        cells_ = decoded;
    }

    bool contains(int x, int y) const {
        return x >= 0 && y >= 0 && x < width_ && y < height_;
    }

    int value(int x, int y) const {
        if (!contains(x, y)) {
            return -1;
        }

        return cells_[static_cast<std::size_t>(y) * width_ + x];
    }

    bool isFree(int x, int y) const {
        return value(x, y) == 0;
    }

    bool metricToCell(double x_m, double y_m, int& x, int& y) const {
        double local_x = x_m - origin_x_;
        double local_y = y_m - origin_y_;
        if (!std::isfinite(local_x) || !std::isfinite(local_y) || local_x < 0 || local_y < 0 ||
            local_x >= width_ * resolution_ || local_y >= height_ * resolution_) {
            return false;
        }

        x = static_cast<int>(std::floor(local_x / resolution_));
        y = static_cast<int>(std::floor(local_y / resolution_));
        return contains(x, y);
    }

    bool cellCentre(int x, int y, double& x_m, double& y_m) const {
        if (!contains(x, y)) {
            return false;
        }

        x_m = origin_x_ + (x + 0.5) * resolution_;
        y_m = origin_y_ + (y + 0.5) * resolution_;
        return true;
    }

    // 后续八邻域寻路可直接用它，禁止沿两个障碍格的夹角穿过去。
    bool canStep(int from_x, int from_y, int to_x, int to_y) const {
        if (!isFree(from_x, from_y) || !isFree(to_x, to_y)) {
            return false;
        }

        int dx = to_x - from_x;
        int dy = to_y - from_y;
        if (dx < -1 || dx > 1 || dy < -1 || dy > 1 || (dx == 0 && dy == 0)) {
            return false;
        }

        if (dx != 0 && dy != 0) {
            return isFree(from_x + dx, from_y) && isFree(from_x, from_y + dy);
        }

        return true;
    }

    const std::vector<int8_t>& data() const {
        return cells_;
    }

  private:
    int width_;
    int height_;
    double resolution_;
    double origin_x_;
    double origin_y_;

    std::vector<int8_t> cells_;
};

} // namespace campus