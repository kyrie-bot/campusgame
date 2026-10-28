#pragma once
#include <algorithm>
#include <cmath>

namespace campus_buff {
// A local expansion of the BALLISTIC INTERCEPT, not of the raw image centre.
// The output owner evaluates it at transmission time; transport age must not
// leave the barrel following an intercept calculated for an earlier instant.
struct RuneAimTrajectory {
    bool valid=false;
    double yaw=0, pitch=0, yaw_rate=0, pitch_rate=0, yaw_acceleration=0, pitch_acceleration=0;

    static RuneAimTrajectory fit(double y0,double p0,double ym,double pm,double yp,double pp,double step) {
        RuneAimTrajectory out;
        if(!std::isfinite(step)||step<=0||step>.02)return out;
        out.yaw=y0;out.pitch=p0;
        double dy0=std::remainder(ym-y0,360.),dy1=std::remainder(yp-y0,360.);
        out.yaw_rate=(dy1-dy0)/(2*step);out.pitch_rate=(pp-pm)/(2*step);
        out.yaw_acceleration=(dy1+dy0)/(step*step);
        out.pitch_acceleration=(pp+pm-2*p0)/(step*step);
        out.valid=std::isfinite(y0)&&std::isfinite(p0)&&out.bounded();
        return out;
    }
    static RuneAimTrajectory fitForward(double y0,double p0,double y1,double p1,double y2,double p2,double step) {
        auto out=fit(y1,p1,y0,p0,y2,p2,step);
        out.yaw=y0;out.pitch=p0;
        out.yaw_rate-=out.yaw_acceleration*step;
        out.pitch_rate-=out.pitch_acceleration*step;
        out.valid=out.valid&&out.bounded();
        return out;
    }
    bool evaluate(double age,double& y,double& p) const {
        if(!valid||!bounded()||!std::isfinite(age)||age<0||age>.08)return false;
        y=std::remainder(yaw+yaw_rate*age+.5*yaw_acceleration*age*age,360.);
        p=pitch+pitch_rate*age+.5*pitch_acceleration*age*age;
        return std::isfinite(y)&&std::isfinite(p)&&std::abs(p)<=60;
    }
private:
    bool bounded() const {
        return std::isfinite(yaw)&&std::isfinite(pitch)&&
            std::hypot(yaw_rate,pitch_rate)<=35 &&
            std::hypot(yaw_acceleration,pitch_acceleration)<=150;
    }
};
}
