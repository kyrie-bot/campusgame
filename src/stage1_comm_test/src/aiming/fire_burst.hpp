#pragma once
#include <cmath>
#include <string>

namespace campus {
inline bool burstGapReason(const std::string& reason) {
    return reason == "TEMP_LOST" || reason == "NO_FRESH_OBSERVATION" || reason == "NO_TARGET" ||
           reason == "NO_TRACKING_TARGET" || reason == "SOURCE_EXPIRED";
}//判断中断后是否允许短暂续射

inline bool burstWithinLease(double age, double observation_age, double lease) {
    return std::isfinite(age) && std::isfinite(observation_age) && lease > 0 && lease <= .15 && age >= 0 &&
           age <= lease && observation_age >= 0 && observation_age <= .35;
}//判断是否处于允许续射的短时间窗口
}