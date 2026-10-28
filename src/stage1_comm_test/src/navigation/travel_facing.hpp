#pragma once
#include <cmath>
#include <algorithm>
namespace campus {
// Retain a tiny direction-bearing command while turning, so the independent
// gimbal owner can face the route before the chassis accelerates forwards.
inline double runeTravelSpeedLimit(double yaw, double route_yaw, double cruise) {
    if (!std::isfinite(yaw) || !std::isfinite(route_yaw)) return 0.;
    double error=std::abs(std::remainder(route_yaw-yaw,360.));
    if (error >= 60.) return std::min(cruise,.02);
    if (error <= 20.) return cruise;
    return .02+(cruise-.02)*(60.-error)/40.;
}
}
