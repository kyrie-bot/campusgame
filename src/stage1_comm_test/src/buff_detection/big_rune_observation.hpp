#pragma once
#include <opencv2/core.hpp>
#include <cmath>
#include <vector>

namespace campus_buff {
inline double bigRuneObservedPhase(const std::vector<double>& phases) {
    if(phases.empty())return NAN;
    double x=0,y=0;
    for(double phase:phases) {
        if(!std::isfinite(phase))return NAN;
        x+=std::cos(5*phase);y+=std::sin(5*phase);
    }
    if(std::hypot(x,y)<.5*phases.size())return NAN;
    double mean=std::atan2(y,x)/5.;
    return phases.front()+std::remainder(mean-phases.front(),2*std::acos(-1.)/5.);
}

inline cv::Vec3d bigRuneObservedHub(const std::vector<cv::Vec3d>& hubs) {
    if(hubs.empty())return {NAN,NAN,NAN};
    cv::Vec3d mean{};
    for(auto hub:hubs)mean+=hub;
    return mean*(1./hubs.size());
}
}
