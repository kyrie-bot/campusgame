#pragma once
#include "geometry/aim_geometry.hpp"
#include <deque>

namespace campus {
class ShooterMotion {
    struct Sample { double t; cv::Vec3d v; };
    std::deque<Sample> samples_;
public:
    void reset() { samples_.clear(); }//清除历史运动信息
    void update(double time, cv::Vec3d velocity) {
        if (!std::isfinite(time) || !finite(velocity)) { 
            reset(); return; 
        }
        if (!samples_.empty() && time <= samples_.back().t) return;
        if (!samples_.empty() && time-samples_.back().t > .28) reset();

        samples_.push_back({time,velocity});

        while (samples_.size()>12 || time-samples_.front().t>.18) samples_.pop_front();
    }//己方机器人最新速度信息

    cv::Vec3d releaseCorrection(double delay) const {
        if (samples_.size()<4 || !std::isfinite(delay) || delay<0 || delay>.32 ||
            samples_.back().t-samples_.front().t<.06) return {};

        if(samples_.size()>=7 && samples_.back().t-samples_.front().t>=.12) {
            auto fit_curve=[&](size_t count,cv::Mat& coefficients) {
                cv::Mat design(int(count),3,CV_64F),values(int(count),3,CV_64F);

                for(size_t i=0;i<count;++i) {
                    double t=samples_[i].t-samples_.back().t;
                    design.at<double>(i,0)=1;design.at<double>(i,1)=t;design.at<double>(i,2)=.5*t*t;
                    for(int axis=0;axis<3;++axis) values.at<double>(i,axis)=samples_[i].v[axis];
                }

                return cv::solve(design,values,coefficients,cv::DECOMP_SVD);
            };

            cv::Mat coefficients,withheld;

            if(fit_curve(samples_.size(),coefficients) && fit_curve(samples_.size()-2,withheld)) {
                auto row=[](const cv::Mat& m,int i){return cv::Vec3d(m.at<double>(i,0),m.at<double>(i,1),m.at<double>(i,2));};
                auto v=row(coefficients,0),a=row(coefficients,1),jerk=row(coefficients,2);
                double residual=0,heldout=0;

                for(size_t i=0;i<samples_.size();++i) {
                    double t=samples_[i].t-samples_.back().t;
                    auto e=samples_[i].v-v-a*t-jerk*(.5*t*t);
                    residual+=e.dot(e);
                    if(i>=samples_.size()-2) {
                        auto h=samples_[i].v-row(withheld,0)-row(withheld,1)*t-row(withheld,2)*(.5*t*t);
                        heldout+=h.dot(h);
                    }
                }

                if(std::sqrt(residual/samples_.size())<.05 && std::sqrt(heldout/2)<.08 &&
                   cv::norm(a)<=9. && cv::norm(jerk)<=40. && cv::norm(a+jerk*delay)<=9.)
                    return a*(.5*delay*delay)+jerk*(delay*delay*delay/6.);
            }
        }

        auto fit=[&](size_t n, cv::Vec3d& intercept,cv::Vec3d& acceleration) {
            double st=0,stt=0;cv::Vec3d sv{},stv{};

            for(size_t i=0;i<n;++i) {
                double t=samples_[i].t-samples_.back().t;
                st+=t;stt+=t*t;sv+=samples_[i].v;stv+=samples_[i].v*t;
            }

            double den=n*stt-st*st;

            if(den<1e-9) return false;

            acceleration=(stv*double(n)-sv*st)/den;
            intercept=(sv-acceleration*st)/double(n);return true;
        };

        cv::Vec3d v,a,hv,ha;

        if(!fit(samples_.size(),v,a) || !fit(samples_.size()-1,hv,ha) || cv::norm(a)>9.) return {};

        double residual=0;

        for(auto s:samples_) residual+=cv::norm(s.v-v-a*(s.t-samples_.back().t),cv::NORM_L2SQR);

        if(std::sqrt(residual/samples_.size())>.12 || cv::norm(samples_.back().v-hv)>.2) return {};
        return a*(.5*delay*delay);
    }//估计运动状态带来的发射位置补偿
};
} // namespace campus
