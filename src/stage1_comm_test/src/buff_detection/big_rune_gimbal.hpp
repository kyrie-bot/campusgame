#pragma once
#include "aiming/rune_gimbal_feedback.hpp"

namespace campus_buff {
// A measured tracking observer for stationary rune aiming only.
// Compare delayed actual angles with the command that was in effect at THEIR
// timestamp. Only an established, slow, low-residual track can carry that
// measured residual forward along the commanded path. A requested angle alone
// can never establish tracking (a stalled barrel or a slew fails the checks).
class BigRuneGimbal {
    struct Sample {double t,y,p;};
    std::deque<Sample> commands_,measurements_;
public:
    void reset() {commands_.clear();measurements_.clear();rejection_=estimate_source_=0;}
    void command(double applied,double y,double p) {append(commands_,applied,y,p);}
    void measured(double stamp,double y,double p) {append(measurements_,stamp,y,p);}
    int rejection() const {return rejection_;}
    int estimateSource() const {return estimate_source_;}
    double measurementStamp() const {return measurements_.empty()?NAN:measurements_.back().t;}
    campus::RuneGimbalEstimate predict(double now,double delay,double y,double p,bool big_rune=false) const {
        rejection_=0;
        if(measurements_.size()<3||commands_.size()<2||delay<0||delay>.1)return reject(1);
        auto last=measurements_.back();
        if(now-last.t<0||now-last.t>.28)return reject(2);
        double sy=0,sp=0,first=INFINITY,first_y=0,first_p=0,last_y=0,last_p=0;int n=0;
        // Three consistent real measurements establish the local residual.
        // Validate the entire later command path too; old pre-slew samples
        // must not keep a now-stable barrel blocked for another frame.
        size_t begin=measurements_.size()-3;
        while(begin>0 && last.t-measurements_[begin].t<.055)--begin;
        for(size_t i=begin;i<measurements_.size();++i) {
            auto m=measurements_[i];
            Sample c;if(!at(m.t,c))return reject(3);
            double ey=std::remainder(m.y-c.y,360.),ep=m.p-c.p;
            // A stable measured servo offset is corrected below, not ignored.
            // For big runes the old .40 deg acquisition cap was stricter than
            // the final .70 deg barrel-alignment gate, causing phase-dependent
            // stalls with a few extra milliseconds of actuator lag. Keep the
            // offset bounded, its drift checked, and the final fire gate intact.
            if(std::hypot(ey,ep)>(big_rune?.85:.40))return reject(4);
            if(!n) {first=m.t;first_y=ey;first_p=ep;}
            sy+=ey;sp+=ep;last_y=ey;last_p=ep;++n;
        }
        // About 0.1 deg quantisation on two axes can span 0.28 deg between
        // endpoints. Remove that bounded noise before testing physical drift;
        // retain the original small-rune observer and the 3 deg/s drift bound.
        double noise=big_rune?.28:.16;
        if(n<3||last.t-first<.055||last.t-first>.45||std::max(0.,std::hypot(last_y-first_y,last_p-first_p)-noise)/(last.t-first)>3.)return reject(5);
        // Validate the whole unmeasured command path, not just its endpoints.
        Sample previous;if(!at(last.t,previous))return reject(6);
        for(auto c:commands_)if(c.t>last.t) {
            if(!continuous(previous,c))return reject(7);
            previous=c;
        }
        Sample future{now+delay,y,p};
        if(future.t>previous.t+.001) {if(!continuous(previous,future))return reject(8);}
        else if(std::hypot(std::remainder(y-previous.y,360.),p-previous.p)>.035)return reject(8);
        double ey=sy/n,ep=sp/n;
        if(big_rune) {
            // Extra servo delay produces a CHANGING command/actual residual
            // as the rune accelerates. Carrying its old mean 180 ms forward
            // falsely declares alignment. Fit only timestamp-paired REAL
            // feedback, then forecast that residual to command application.
            std::deque<Sample> residuals;
            for(auto m:measurements_)if(last.t-m.t<=.30) {
                Sample c;if(!at(m.t,c)){residuals.clear();continue;}
                double dy=std::remainder(m.y-c.y,360.),dp=m.p-c.p;
                if(std::hypot(dy,dp)>.85){residuals.clear();continue;}
                // A past acquisition slew is not part of the CURRENT stable
                // tracking segment. Apply the same residual/drift envelope as
                // the three-measurement acquisition check above, then fit only
                // the contiguous measured tail. Old slew data cannot veto an
                // already established fresh track for another 300 ms.
                if(!residuals.empty() && std::hypot(dy-residuals.back().y,dp-residuals.back().p)>
                    .28+3*(m.t-residuals.back().t))residuals.clear();
                residuals.push_back({m.t,dy,dp});
            }
            double st=0,stt=0,sy=0,sp=0,sty=0,stp=0;int count=residuals.size();
            for(auto r:residuals) {
                double t=r.t-last.t,dy=r.y,dp=r.p;
                st+=t;stt+=t*t;sy+=dy;sp+=dp;sty+=t*dy;stp+=t*dp;
            }
            double denominator=count*stt-st*st;
            if(count>=5 && last.t-residuals.front().t>=.16 && denominator>1e-8) {
                double vy=(count*sty-st*sy)/denominator,vp=(count*stp-st*sp)/denominator;
                double by=(sy-vy*st)/count,bp=(sp-vp*st)/count,squared=0;
                for(auto r:residuals) {
                    double t=r.t-last.t;
                    squared+=std::pow(r.y-by-vy*t,2)+std::pow(r.p-bp-vp*t,2);
                }
                double lead=now+delay-last.t;
                if(std::hypot(vy,vp)>3. || std::sqrt(squared/count)>.12 || lead>.32)return reject(9);
                ey=by+vy*lead;ep=bp+vp*lead;
                if(std::hypot(ey,ep)>.85)return reject(9);
            }
        }
        return {true,std::remainder(y+ey,360.),p+ep};
    }
// A fresh feedback stream need not wait for a long command-history fit.
    // This short extrapolation is never used for 180 ms delayed feedback:
    // three actual measurements must agree in rate and curvature first.
    campus::RuneGimbalEstimate fireEstimate(double now,double delay,double y,double p,bool big_rune=false) const {
        estimate_source_=0;
        if(!std::isfinite(now)||!std::isfinite(delay)||!std::isfinite(y)||!std::isfinite(p))return {};
        auto tracked=predict(now,delay,y,p,big_rune);
        if(tracked.valid){estimate_source_=1;return tracked;}
        if(measurements_.size()<3 || !std::isfinite(delay) || delay<0 || delay>.1)return {};
        auto a=measurements_[measurements_.size()-3],b=measurements_[measurements_.size()-2],c=measurements_.back();
        double age=now-c.t,dt1=b.t-a.t,dt2=c.t-b.t,span=c.t-a.t;
        if(age<0 || age>.08 || dt1<=0 || dt2<=0 || span<.03 || span>.28)return {};
        double vy1=std::remainder(b.y-a.y,360.)/dt1,vp1=(b.p-a.p)/dt1;
        double vy2=std::remainder(c.y-b.y,360.)/dt2,vp2=(c.p-b.p)/dt2;
        if(std::max(std::hypot(vy1,vp1),std::hypot(vy2,vp2))>35. ||
           std::hypot(vy2-vy1,vp2-vp1)>80.*span*.5+.8)return {};
        double vy=std::remainder(c.y-a.y,360.)/span,vp=(c.p-a.p)/span;
        if(std::hypot(std::remainder(b.y-a.y-vy*dt1,360.),b.p-a.p-vp*dt1)>.12)return {};
        campus::RuneGimbalEstimate out{true,std::remainder(c.y+vy*(age+delay),360.),c.p+vp*(age+delay)};
        if(std::hypot(std::remainder(y-out.yaw,360.),p-out.pitch)>1.)return {};
        rejection_=0;estimate_source_=2;return out;
    }
private:
    mutable int rejection_=0,estimate_source_=0;
    campus::RuneGimbalEstimate reject(int reason) const {rejection_=reason;return {};}
    static void append(std::deque<Sample>& q,double t,double y,double p) {
        if(!std::isfinite(t)||!std::isfinite(y)||!std::isfinite(p))return;
        if(!q.empty()&&t<=q.back().t)return;
        q.push_back({t,y,p});while(q.size()>200||t-q.front().t>1.)q.pop_front();
    }
    static bool continuous(Sample a,Sample b) {
        double dt=b.t-a.t;
        return dt>0&&dt<.12&&std::hypot(std::remainder(b.y-a.y,360.),b.p-a.p)<=35.*dt+.35;
    }
    bool at(double t,Sample& out) const {
        for(size_t i=1;i<commands_.size();++i)if(commands_[i-1].t<=t&&commands_[i].t>=t) {
            auto a=commands_[i-1],b=commands_[i];if(!continuous(a,b))return false;
            double f=(t-a.t)/(b.t-a.t);
            out={t,a.y+std::remainder(b.y-a.y,360.)*f,a.p+(b.p-a.p)*f};return true;
        }
        return false;
    }
};
}
