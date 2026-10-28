#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace campus {
struct PlanPoint {
    double x = 0;
    double y = 0;
};

struct GridPlan {
    std::string status;
    std::vector<int> cells;
    double length_m = 0;
    double total_cost = 0;
    int expanded = 0;
};

class DijkstraGrid {
  public:
    int width = 0;
    int height = 0;
    double resolution = 0;
    double origin_x = 0;
    double origin_y = 0;
    std::vector<int8_t> cells;

    bool valid() const {
        return width > 0 && height > 0 && static_cast<long long>(width) * height <= 1000000 &&
               cells.size() == static_cast<std::size_t>(width) * height && std::isfinite(resolution) &&
               resolution > 0 && std::isfinite(origin_x) && std::isfinite(origin_y);
    }

    bool free(int x, int y) const {
        return x >= 0 && y >= 0 && x < width && y < height && cells[y * width + x] == 0;
    }

    int cellAt(PlanPoint point) const {
        double x = (point.x - origin_x) / resolution;
        double y = (point.y - origin_y) / resolution;
        if (!valid() || !std::isfinite(x) || !std::isfinite(y) || x < 0 || y < 0 || x >= width || y >= height) {
            return -1;
        }

        return static_cast<int>(y) * width + static_cast<int>(x);
    }

    PlanPoint centre(int index) const {
        PlanPoint point;
        point.x = origin_x + (index % width + 0.5) * resolution;
        point.y = origin_y + (index / width + 0.5) * resolution;
        return point;
    }

    bool canStep(int x, int y, int next_x, int next_y) const {
        int dx = next_x - x;
        int dy = next_y - y;
        if (!free(x, y) || !free(next_x, next_y) || std::abs(dx) > 1 || std::abs(dy) > 1 || (dx == 0 && dy == 0)) {
            return false;
        }

        // 八邻域的斜向一步，两旁的格子也必须空闲。
        return dx == 0 || dy == 0 || (free(x + dx, y) && free(x, y + dy));
    }

    void blockCircle(PlanPoint point, double radius) {
        if (!valid() || !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(radius) || radius < 0 ||
            radius > 1000) {
            return;
        }

        double coverage = radius + resolution * std::sqrt(2.0) / 2;
        // 格心加半格对角线，保守覆盖圆与格子的交集。
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                PlanPoint cell = centre(y * width + x);
                if (std::hypot(cell.x - point.x, cell.y - point.y) <= coverage) {
                    cells[y * width + x] = 100;
                }
            }
        }
    }

    GridPlan plan(PlanPoint start, PlanPoint goal, const std::vector<double>& risk = {}) const {
        GridPlan result;
        if (!valid()) {
            result.status = "INVALID_MAP";
            return result;
        }

        if (!risk.empty()) {
            if (risk.size() != cells.size()) {
                result.status = "INVALID_COSTS";
                return result;
            }

            for (double value : risk) {
                if (!std::isfinite(value) || value < 0) {
                    result.status = "INVALID_COSTS";
                    return result;
                }
            }
        }

        int first = cellAt(start);
        int last = cellAt(goal);
        if (first < 0 || last < 0) {
            result.status = "OUTSIDE_MAP";
            return result;
        }

        if (cells[first] != 0 || cells[last] != 0) {
            result.status = cells[first] != 0 ? "START_BLOCKED" : "GOAL_BLOCKED";
            return result;
        }

        std::vector<double> distance(cells.size(), std::numeric_limits<double>::infinity());
        std::vector<int> parent(cells.size(), -1);
        typedef std::pair<double, int> QueueItem;
        std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pending;
        distance[first] = 0;
        pending.push(QueueItem(0, first));
        int directions[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

        while (!pending.empty()) {
            QueueItem current = pending.top();
            pending.pop();
            if (current.first != distance[current.second]) {
                continue;
            }
            ++result.expanded;

            if (current.second == last) {
                break;
            }

            int x = current.second % width;
            int y = current.second / width;
            for (int direction = 0; direction < 8; ++direction) {
                int dx = directions[direction][0];
                int dy = directions[direction][1];

                if (!canStep(x, y, x + dx, y + dy)) {
                    continue;
                }

                int next = (y + dy) * width + x + dx;
                double step = resolution;
                if (dx != 0 && dy != 0) {
                    step *= std::sqrt(2.0);
                }

                double weight = risk.empty() ? 1 : 1 + (risk[current.second] + risk[next]) / 2;
                double candidate = current.first + step * weight;
                if (candidate < distance[next]) {
                    distance[next] = candidate;
                    parent[next] = current.second;
                    pending.push(QueueItem(candidate, next));
                }
            }
        }

        if (!std::isfinite(distance[last])) {
            result.status = "NO_PATH";
            return result;
        }

        for (int cell = last; cell != -1; cell = parent[cell]) {
            result.cells.push_back(cell);
        }

        std::reverse(result.cells.begin(), result.cells.end());
        result.total_cost = distance[last];
        for (std::size_t i = 1; i < result.cells.size(); ++i) {
            PlanPoint a = centre(result.cells[i - 1]);
            PlanPoint b = centre(result.cells[i]);
            result.length_m += std::hypot(a.x - b.x, a.y - b.y);
        }

        result.status = "OK";
        return result;
    }
};

// Unity 世界 X/Z 米 -> 当前参考地图 XY 米；这是查看投影，不是已验证地图标定。
class PlanningProjection {
  public:
    std::vector<double> matrix;

    bool valid() const {
        if (matrix.size() != 6) {
            return false;
        }

        for (std::size_t i = 0; i < matrix.size(); ++i) {
            if (!std::isfinite(matrix[i])) {
                return false;
            }
        }

        return std::abs(matrix[0] * matrix[4] - matrix[1] * matrix[3]) > 1e-9;
    }

    PlanPoint fromUnity(double x_m, double z_m) const {
        PlanPoint point;
        point.x = matrix[0] * x_m + matrix[1] * z_m + matrix[2];
        point.y = matrix[3] * x_m + matrix[4] * z_m + matrix[5];
        return point;
    }
};

// 区分选点被静态膨胀图挡住，还是被这一帧的敌情挡住。
inline std::string pointBlockReason(const DijkstraGrid& base, const DijkstraGrid& combined, PlanPoint point) {
    if (!base.valid() || !combined.valid()) {
        return "WAIT_MAP";
    }

    int cell = base.cellAt(point);
    if (cell < 0) {
        return "OUTSIDE_MAP";
    }

    if (base.cells[cell] != 0) {
        return "STATIC_OR_INFLATED";
    }

    int current = combined.cellAt(point);
    if (current < 0) {
        return "OUTSIDE_MAP";
    }

    return combined.cells[current] == 0 ? "FREE" : "ENEMY_REGION";
}
} // namespace campus