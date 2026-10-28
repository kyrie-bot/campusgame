#pragma once

#include "tracking/rigid_tracker.hpp"
#include "navigation/mission_order.hpp"

namespace campus {
class LockedTargetSelector {
  public:
    uint32_t locked_id = 0;

    const RigidTarget* select(const std::vector<RigidTarget>& targets, const CameraGeometry& camera,
                              const cv::Matx33d& k, const cv::Mat& d, int number = 0, bool acquire_detecting = false,
                              const std::string& mission_stage = {}, const std::array<int32_t, 7>* healths = nullptr,
                              bool ignore_one = false, bool field_order = false) {
        auto allowed = [&](int id) {
            return mission_stage.empty() || (healths ? missionArmorAlive(mission_stage, id, *healths, ignore_one, field_order)
                                                     : missionArmorAllowed(mission_stage, id));
        };

        for (size_t target_index = 0; target_index < targets.size(); ++target_index) {
            const RigidTarget& t = targets[target_index];
            if (allowed(t.number) && t.id == locked_id &&
                (t.state == "TRACKING" || (!acquire_detecting && t.state == "TEMP_LOST")) &&
                (!number || t.number == number)) {
                return &t;
            }
        }

        locked_id = 0;
        const RigidTarget* best = nullptr;
        double cost = INFINITY;

        for (size_t target_index = 0; target_index < targets.size(); ++target_index) {
            const RigidTarget& t = targets[target_index];
            if ((t.state != "TRACKING" && !(acquire_detecting && t.state == "DETECTING")) ||
                (number && t.number != number) || !allowed(t.number)) {
                continue;
            }

            cv::Point2f pixel;
            if (!camera.project({t.x(0), t.x(2), t.x(4)}, k, d, pixel)) {
                continue;
            }

            double distance = cv::norm(pixel - cv::Point2f(k(0, 2), k(1, 2))) + (t.state == "DETECTING" ? 1e6 : 0.);
            if (distance < cost) {
                cost = distance;
                best = &t;
            }
        }

        if (best) {
            locked_id = best->id;
        } else if (acquire_detecting) {
            for (const auto& t : targets)
                if (t.state == "TEMP_LOST" && (!number || t.number == number) && allowed(t.number)) {
                    best = &t;
                    locked_id = t.id;
                    break;
                }
        }

        return best;
    }
};

inline int drawReprojection(cv::Mat& image, const RigidTarget& target, const CameraGeometry& camera,
                            const cv::Matx33d& k, const cv::Mat& d, double horizon = 0,
                            cv::Scalar color = {0, 255, 0}) {
    if (target.state != "TRACKING" && target.state != "TEMP_LOST") {
        return 0;
    }

    int drawn = 0;
    for (int slot = 0; slot < target.count; ++slot) {
        std::array<cv::Vec3d, 4> corners = armorCorners(target.position(slot, horizon), target.yaw(slot, horizon),
                                                        target.width, target.height, target.tilt);
        std::array<cv::Point2f, 4> p;
        bool valid = true;

        for (int i = 0; i < 4; ++i) {
            if (!camera.project(corners[i], k, d, p[i])) {
                valid = false;
            }
        }

        if (!valid) {
            continue;
        }

        cv::Rect box = cv::boundingRect(std::vector<cv::Point2f>(p.begin(), p.end()));
        if (box.width > image.cols * 0.5 || box.height > image.rows * 0.5 ||
            (box & cv::Rect(0, 0, image.cols, image.rows)).empty()) {
            continue;
        }

        for (int i = 0; i < 4; ++i) {
            cv::line(image, p[i], p[(i + 1) % 4], color, 1, cv::LINE_AA);
        }
        ++drawn;
    }

    return drawn;
}

} // namespace campus