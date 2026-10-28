#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>

namespace campus_buff {
// Keep image/model corrections inside the measured observer's command-path
// envelope. The ballistic intercept remains an independent firing reference.
template<class Scalar> inline void continuousRuneCommand(double previous_yaw,double previous_pitch,
                                                        Scalar& yaw,Scalar& pitch,double dt,bool acquiring=false) {
    if(!std::isfinite(dt)||dt<=0||!std::isfinite(yaw)||!std::isfinite(pitch)) {
        yaw=previous_yaw;pitch=previous_pitch;return;
    }
    double dy=std::remainder(yaw-previous_yaw,360.),dp=pitch-previous_pitch;
    // A confirmed leaf change may acquire at the existing 120 deg/s ceiling.
    // Its fire intent stays false until post-change REAL feedback arrives;
    // the measured observer still rejects the fast slew. Steady tracking
    // retains the smaller envelope for model/noise corrections.
    double length=std::hypot(dy,dp),limit=acquiring?120.*dt:std::min(120.*dt,35.*dt+.25);
    double scale=length>limit?limit/length:1.;
    yaw=std::remainder(previous_yaw+dy*scale,360.);pitch=previous_pitch+dp*scale;
}
// Collapse follows confirmed activations, never requests or ammunition use.
class BigRuneSuccesses {
public:
    void reset() {requests_.clear();}
    void observe(uint64_t request,unsigned opportunity,bool activated) {
        if(request && opportunity>=3 && opportunity<=5 && activated)requests_.insert(request);
    }
    unsigned count() const {return requests_.size();}
private:
    std::set<uint64_t> requests_;
};

inline double finalRuneAlignmentLimit(bool collapsed,double horizon,double age,double latency,double scene_speed) {
    if(!collapsed)return .70; // Preserve the first two big-rune visits.
    double speed=scene_speed/3.,flight=horizon-age-latency;
    if(!std::isfinite(speed)||speed<10||speed>35)speed=23.5;
    double range=std::isfinite(flight)&&flight>=.01&&flight<=.65
        ? speed*flight+4.905*flight*flight+.10 : 8.;
    // Reserve a 35 mm barrel budget inside the smaller centre, leaving room
    // for phase/geometry error. Never authorize a shot from range alone.
    return std::min(.70,std::atan2(.035,range)*180./std::acos(-1.));
}

// A bounded measured correction, independent of the ballistic reference.
// Requested angles cannot train it; repeated callbacks are not new feedback.
class FinalRuneTrim {
public:
    void reset() {yaw_=pitch_=0;stamp_=-INFINITY;}
    void observe(double stamp,double command_yaw,double command_pitch,double actual_yaw,double actual_pitch) {
        if(!std::isfinite(stamp)||stamp<=stamp_||!std::isfinite(command_yaw)||!std::isfinite(command_pitch)||
           !std::isfinite(actual_yaw)||!std::isfinite(actual_pitch))return;
        double dt=std::isfinite(stamp_)?std::min(.1,stamp-stamp_):.04;
        stamp_=stamp;
        // actual-command already contains the measured servo residual. Do not
        // add it to the intercept, or integrate the same correction twice.
        move(-std::remainder(actual_yaw-command_yaw,360.),command_pitch-actual_pitch,dt);
    }
    void decay(double dt) {if(std::isfinite(dt)&&dt>0)move(0,0,std::min(.1,dt));}
    double yaw() const {return yaw_;}
    double pitch() const {return pitch_;}
private:
    void move(double y,double p,double dt) {
        double length=std::hypot(y,p),scale=length>.60?.60/length:1.;
        double gain=1-std::exp(-dt/.06);
        double dy=(y*scale-yaw_)*gain,dp=(p*scale-pitch_)*gain;
        double step=std::hypot(dy,dp),rate=std::min(1.,4*dt/std::max(step,1e-9));
        yaw_+=dy*rate;pitch_+=dp*rate;
    }
    double yaw_=0,pitch_=0,stamp_=-INFINITY;
};
}
