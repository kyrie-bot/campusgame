#pragma once
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

// Traditional image-space R localization. No model, theoretical five-slot
// completion or fire permission is produced by this module.
namespace campus_buff {
struct RingEvidence {
    cv::Point2f center, inward;
    float radius = 0;
};

struct RCenterResult {
    bool observed = false;
    bool held = false;
    cv::Point2f center{};
    cv::RotatedRect bounds;
    int supporting_rings = 0;
    double age_s = 0;
    double score = 0;
};

class RCenterTracker {
public:
    void setGlyphTemplate(const std::vector<cv::Point>& contour) {
        glyph_ = contour;
    }

    void reset() {
        have_ = false;
        last_time_ = -1;
        last_seen_ = -1;
        last_ = {};
    }

    RCenterResult update(const cv::Mat& mask, const std::vector<RingEvidence>& rings, double time) {
        if (mask.empty() || mask.type() != CV_8UC1 || !std::isfinite(time)) {
            reset();
            return {};
        }

        if (last_time_ >= 0 && (time < last_time_ || time - last_time_ > .5 || mask.size() != size_))
            reset();
        last_time_ = time;
        size_ = mask.size();
        cv::findContours(mask.clone(), contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        RCenterResult best;
        double cost = std::numeric_limits<double>::infinity();

        for (const auto& contour : contours_) {
            double area = cv::contourArea(contour);
            if (area < 8 || area > mask.total() * .004 || contour.size() < 5)
                continue;
            double glyph_distance = glyph_.empty() ? 0. : cv::matchShapes(contour, glyph_, cv::CONTOURS_MATCH_I1, 0);
            if (!std::isfinite(glyph_distance) || glyph_distance > 1.35)
                continue;
            auto rect = cv::minAreaRect(contour);
            double lo = std::min(rect.size.width, rect.size.height), hi = std::max(rect.size.width, rect.size.height);
            if (lo < 4 || hi / lo > 1.9)
                continue;
            std::vector<cv::Point> hull;
            cv::convexHull(contour, hull);
            double hull_area = cv::contourArea(hull);
            double fill = area / (lo * hi), solidity = area / std::max(1., hull_area);
            // A filled disk / ring outline is not the notched R glyph.
            double perimeter = cv::arcLength(contour, true);
            double roundness = perimeter > 0 ? 4 * CV_PI * area / (perimeter * perimeter) : 1.;
            if (fill < .22 || solidity > .97 || (roundness > .78 && solidity > .90))
                continue;
            cv::Point2f center = rect.center;
            int support = 0;
            double geometry = 0, scale = 0;

            for (const auto& ring : rings) {
                if (!finite(ring.center) || !finite(ring.inward) || !std::isfinite(ring.radius) || ring.radius < 5)
                    continue;
                double norm = cv::norm(ring.inward);
                if (norm < .5)
                    continue;
                cv::Point2f u = ring.inward / static_cast<float>(norm), delta = center - ring.center;
                double along = delta.dot(u), cross = std::abs(delta.x * u.y - delta.y * u.x);
                // Search a scale-dependent inward strip, not the image centre.
                // The ring and its own leaf are outside the compact-glyph test.
                if (along < 2.5 * ring.radius || along > 10. * ring.radius || cross > std::max(7., .9 * ring.radius) ||
                    hi > 2.4 * ring.radius || lo < .25 * ring.radius)
                    continue;
                ++support;
                scale += ring.radius;
                geometry += cross / ring.radius + .08 * std::abs(along / ring.radius - 5.5);
            }
            // After acquisition a completed ring's ambiguous direction must
            // not veto the same measured R. Cold acquisition still requires
            // multi-ring consensus when several rings are visible.
            if (!rings.empty() && support < std::min(2, static_cast<int>(rings.size()))) {
                double old_size = std::max(last_.bounds.size.width, last_.bounds.size.height);
                bool same_glyph = have_ && time - last_seen_ <= .15 &&
                                  cv::norm(center - last_.center) <= std::max(20., old_size * 4.8) &&
                                  hi >= old_size * .65 && hi <= old_size * 1.5;
                if (support != 1 || !same_glyph)
                    continue;
            }

            bool local_only = rings.empty() && have_ && time - last_seen_ <= .15;
            if (support == 0 && !local_only)
                continue;
            double distance = have_ ? cv::norm(center - last_.center) : 0.;
            if (local_only) {
                double old_size = std::max(last_.bounds.size.width, last_.bounds.size.height);
                if (distance > std::max(10., old_size * .65) || hi < old_size * .65 || hi > old_size * 1.5)
                    continue;
            }

            double score = .3 * glyph_distance + (support ? geometry / support : 1.) + .12 * std::abs(hi / lo - 1.2) +
                           .15 * std::abs(fill - .6);
            if (have_ && time - last_seen_ <= .15)
                score += .15 * distance / std::max(10., scale / std::max(1, support));
            score -= .35 * support;
            if (score < cost) {
                cost = score;
                best.observed = true;
                best.center = center;
                best.bounds = rect;
                best.supporting_rings = support;
                best.score = score;
            }
        }

        if (best.observed) {
            // Keep the measured rectangle centre. Averaging with a stale
            // pixel location causes lag whenever the camera moves.
            have_ = true;
            last_seen_ = time;
            last_ = best;
            return best;
        }

        if (have_ && time - last_seen_ <= .15) {
            best = last_;
            best.observed = false;
            best.held = true;
            best.age_s = time - last_seen_;
            return best;
        }
        have_ = false;
        return {};
    }

private:
    static bool finite(cv::Point2f p) {
        return std::isfinite(p.x) && std::isfinite(p.y);
    }

    bool have_ = false;
    double last_time_ = -1, last_seen_ = -1;
    cv::Size size_;
    RCenterResult last_;
    std::vector<std::vector<cv::Point>> contours_;
    std::vector<cv::Point> glyph_;
};
} // namespace campus_buff