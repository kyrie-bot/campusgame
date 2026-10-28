#pragma once
#include <algorithm>
#include <cmath>

namespace campus {
inline constexpr double kReacquireTimeout = 1.5;
inline constexpr double kReacquireHold = .6;

struct ReacquireAngles {
    double yaw, pitch;
};//重新搜索目标时的yaw,pitch

inline ReacquireAngles localReacquireAngles(double yaw, double pitch, double elapsed) {
    double t = std::max(0., elapsed - kReacquireHold);
    return {std::remainder(yaw + 4. * std::sin(t * 2 * M_PI / 2.), 360.), std::clamp(pitch, -60., 60.)};
}//目标短暂消失时进行局部重新搜索
}