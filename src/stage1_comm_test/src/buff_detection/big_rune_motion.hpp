#pragma once
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <vector>

namespace campus_buff {

class BigRuneMotion {
    struct Sample { double t, a; };
    struct Fit {
        double base=0, start=0, offset=0, amplitude=.9, omega=1.94, phase=0, sign=1;
        double rms=INFINITY, validation=INFINITY, prediction_error=INFINITY;
        double value(double t) const {
            double x=t-base;
            return offset+sign*((2.09-amplitude)*x-amplitude/omega*std::cos(omega*x+phase));
        }
        double velocity(double t) const {
            return sign*(2.09-amplitude+amplitude*std::sin(omega*(t-base)+phase));
        }
    };

public:
    void reset() { *this=BigRuneMotion(); }

    bool update(double angle,double time) {
        if(!std::isfinite(angle)||!std::isfinite(time)) {
            reset();return false;
        }

        if(!samples_.empty()&&time<=samples_.back().t) return false;

        const double slot=2*CV_PI/5;
        double unwrapped=angle;

        if(!samples_.empty()) {
            double dt=time-samples_.back().t;

            if(dt>.65 || (dt>.35&&!fitted_)) reset();

            else {
                double expected=samples_.back().a+(fitted_ ? fit_.value(time)-fit_.value(samples_.back().t) : velocity_*dt);
                unwrapped=expected+std::remainder(angle-expected,slot);
                if(std::abs(unwrapped-expected)>.06+2.1*dt) {
                    if(++outliers_>=4) reset();
                    return false;
                }
            }
        }

        outliers_=0;samples_.push_back({time,unwrapped});
        while(time-samples_.front().t>4.)samples_.pop_front();
        acquired_=false;
        double n=0,st=0,sa=0,stt=0,sta=0;
        
        for(auto s:samples_) if(time-s.t<=.3) {
            double x=s.t-time ; ++n ; st+=x ; sa+=s.a ; stt+=x*x ; sta+=x*s.a;
        }
        double den=n*stt-st*st;
        if(n >= 4 && den > 1e-8 && time-samples_.front().t >= .07) {
            velocity_=(n*sta-st*sa)/den;
            acquired_=std::abs(velocity_)<=2.3;
        }
        if(time-samples_.front().t>=.80&&time-last_fit_>=.075) {
            last_fit_=time;estimate();
        }

        if(fitted_&&time-fit_.base<=.35)acquired_=true;
        if(readyToFire())established_=true;
        return true;
    }

    double stamp() const {return samples_.empty()?NAN:samples_.back().t;}
    bool ready() const {return acquired_&&!samples_.empty();}
    bool modelReady() const {return ready()&&fitted_&&samples_.back().t-fit_.base<=.35 &&
        std::abs(std::remainder(fit_.value(samples_.back().t)-samples_.back().a,2*CV_PI/5))<.10;}
    double fittedSpan() const {return fitted_?fit_.base-fit_.start:0.;}
    double observedSpan() const {return samples_.empty()?0.:samples_.back().t-samples_.front().t;}
    double fitError() const {return fit_.rms;}
    double validationError() const {return fit_.validation;}
    double predictionError() const {return fit_.prediction_error;}
    
    bool readyToFire() const {
        return modelReady() && fittedSpan()>=1.05 && (fittedSpan()>=1.20 ||
            (fit_.rms<.012 && fit_.validation<.020 && fit_.prediction_error<.025));
    }
    double error() const {return readyToFire()?std::max(fit_.rms,fit_.validation):INFINITY;}
    double filteredPhase(double measured) const {
        return modelReady()?measured+std::clamp(std::remainder(fit_.value(samples_.back().t)-measured,2*CV_PI/5),-.06,.06):measured;
    }
    double advance(double horizon) const {
        if(!ready()||!std::isfinite(horizon)||horizon<0||horizon>.65)return NAN;
        double t=samples_.back().t;
        return modelReady()?fit_.value(t+horizon)-fit_.value(t):velocity_*horizon;
    }

private:
    static double robustError(std::vector<double> residuals) {
        if(residuals.empty())return INFINITY;
        for(auto& r:residuals)r*=r;
        std::sort(residuals.begin(),residuals.end());

        size_t count=residuals.size()-residuals.size()/5;
        double sum=0;for(size_t i=0;i<count;++i)sum+=residuals[i];
        return std::sqrt(sum/count);
    }

    Fit solve(double omega,double phase,double sign,size_t count) const {
        Fit out;out.base=samples_.back().t;out.start=samples_.front().t;out.omega=omega;out.phase=phase;out.sign=sign;
        out.offset=samples_[count-1].a+out.amplitude/omega*std::cos(phase);

        for(int iteration=0;iteration<9;++iteration) {
            cv::Matx33d normal=cv::Matx33d::zeros();cv::Vec3d rhs{},delta{};

            for(size_t i=0;i<count;++i) {
                auto s=samples_[i];double x=s.t-out.base,z=omega*x+out.phase;
                double residual=s.a-out.value(s.t);
                double robust=iteration<3 ? std::min(1.,.025/std::max(.025,std::abs(residual))) :
                    1./(1.+std::pow(residual/.015,2));
                double weight=std::exp2(x/3.)*robust;
                cv::Vec3d j{1.,sign*(-x-std::cos(z)/omega),sign*out.amplitude/omega*std::sin(z)};
                for(int a=0;a<3;++a) {
                    rhs[a]+=weight*j[a]*residual;
                    for(int b=0;b<3;++b)normal(a,b)+=weight*j[a]*j[b];
                }
            }

            if(!cv::solve(normal,rhs,delta,cv::DECOMP_SVD))return out;
            out.offset+=std::clamp(delta[0],-.6,.6);
            out.amplitude=std::clamp(out.amplitude+std::clamp(delta[1],-.15,.15),.780,1.045);
            out.phase=std::remainder(out.phase+std::clamp(delta[2],-.5,.5),2*CV_PI);
        }

        std::vector<double> residuals;
        for(size_t i=0;i<count;++i)residuals.push_back(samples_[i].a-out.value(samples_[i].t));
        out.rms=robustError(residuals);return out;
    }
    void estimate(bool allow_recent=true) {
        size_t training=samples_.size();double end=samples_.back().t;
        
        while(training && (end-samples_[training-1].t<.16 || samples_.size()-training<2))--training;
        if(training<8) return;
        double sign=samples_.back().a>=samples_.front().a?1.:-1.;
        Fit best;double best_score=INFINITY;

        for(double w=1.884;w<=2.00001;w+=.004) {
            for(int p=0;p<8;++p) {
                auto trained=solve(w,p*2*CV_PI/8,sign,training);

                if(trained.rms>.030)continue;

                std::vector<double> residuals;

                for(size_t i=training;i<samples_.size();++i)residuals.push_back(samples_[i].a-trained.value(samples_[i].t));
                
                double held=robustError(residuals);

                if(held>.040)continue;

                auto full=solve(w,trained.phase,sign,samples_.size());

                full.validation=held;
                full.prediction_error=std::abs((trained.value(end+.6)-trained.value(end))-(full.value(end+.6)-full.value(end)));
               
                if(full.rms>=.025 || full.prediction_error>=.040)continue;
                
                double score=full.rms+.1*held+.1*full.prediction_error;
                if(score<best_score) {best_score=score;best=full;}
            }
        }

        if(best.rms<.025&&best.validation<.040&&best.prediction_error<.040) {
            fit_=best;fitted_=true;
        }

        else {
           
            if(allow_recent && !established_ && end-samples_.front().t>1.5) {
                auto recent=*this;
                while(end-recent.samples_.front().t>1.4)recent.samples_.pop_front();
                recent.fitted_=false;
                recent.estimate(false);
                
                if(recent.fitted_) {
                    fit_=recent.fit_;fitted_=true;return;
                }
            }

            if(!fitted_||end-fit_.base>.35)fitted_=false;
        }
    }

    std::deque<Sample> samples_;
    Fit fit_;
    bool acquired_=false,fitted_=false,established_=false;
    int outliers_=0;
    double velocity_=0,last_fit_=-INFINITY;
};
} // namespace campus_buff
