#pragma once
#include "rune_leaf_cycle.hpp"
#include <string>

namespace campus_buff {
// Campus 6.7 / RM2026 5.5.2: five ROUNDS, two illuminated leaves per round.
// A consumed bullet is not a hit. First-hit visual confirmation starts the
// optional second-hit deadline; one good first hit is enough for the round.
class BigRuneCycle {
public:
    void reset() {*this=BigRuneCycle();}
    void begin(double stamp) {reset();activation_=stamp;}
    void predictRotation(double from,double to,double delta) {
        predicted_stamp_=to;
        predicted_delta_=std::abs(from-stamp_)<.001&&std::isfinite(delta)?delta:NAN;
    }
    void observeDark(double stamp) {
        // A lost contour during acquisition/slew is not a new round. Require
        // sustained darkness at a time when the game can actually change pairs.
        int shots=0;for(bool sent:spent_)shots+=sent;
        double eligible=first_slot_<0 ? round_start_+2.3 :
            (shots>=2 ? last_impact_+.02 : first_impact_+.85);
        if(!std::isfinite(stamp)||stamp<=stamp_||!pair_||stamp<eligible)return;
        if(stamp-dark_last_>.09)dark_start_=stamp;
        dark_last_=stamp;
        // With a confirmed first hit, one eligible dark image is useful
        // evidence: a returning pair still needs two separate observations.
        // Requiring two dark frames loses a 160 ms transition at low FPS.
        if(first_hit_ || stamp-dark_start_>=.025)dark_=stamp;
    }
    void observe(const std::vector<RuneLeafObservation>& leaves,double stamp) {
        if(leaves.empty()||!std::isfinite(stamp)||stamp<=stamp_)return;
        dark_last_=-INFINITY;
        double next=leaves.front().phase;
        if(std::isfinite(stamp_)) {
            double dt=stamp-stamp_;
            double expected=basis_+(std::abs(stamp-predicted_stamp_)<.001&&std::isfinite(predicted_delta_)?
                                    predicted_delta_:velocity_*dt);
            next=expected+std::remainder(next-expected,step());
            if(dt<.3)velocity_=std::clamp(.6*velocity_+.4*(next-basis_)/dt,-2.2,2.2);
        }
        basis_=next;stamp_=stamp;
        unsigned pending=0,ordinary=0;
        for(auto leaf:leaves) (leaf.pending?pending:ordinary)|=1u<<slot(leaf.phase);
        int count=0;for(int i=0;i<5;++i)count+=(pending>>i)&1u;
        if(count==2) {
            // A measured missed pair may light again after its FIRST-hit
            // window, with the dark interval falling between camera frames.
            // Do not invent a second-hit deadline for two actual misses;
            // fresh re-illumination still requires two separate observations.
            bool expired_unconfirmed=first_slot_>=0 && !first_hit_ && miss_confirmed_ &&
                stamp>round_start_+2.56 && stamp>last_impact_+.06;
            // A fresh different pair is direct evidence of a new round even
            // if the completed first plate was outside the camera crop.
            bool changed_after_impact=first_slot_>=0 && consumed_[first_slot_] &&
                pending!=pair_ && stamp>last_impact_+.06;
            bool transition=pair_ && ((first_slot_<0&&stamp>round_start_+2.5&&(pending!=pair_||dark_>round_start_+2.3)) ||
                expired_unconfirmed ||
                changed_after_impact ||
                (extra_&&stamp>last_impact_+.08) ||
                (first_hit_&&(pending!=pair_||dark_>last_impact_-.12 ||
                    // At low FPS the entire dark gap can fall between images.
                    // A previously confirmed hit relighting after the round
                    // deadline is also evidence; two relit images are required.
                    ((pending&(1u<<first_slot_)) && stamp>first_impact_+1.))) ||
                (first_slot_>=0&&stamp>round_start_+2.5&&(pending!=pair_||dark_>round_start_+2.3)));
            if(!pair_||transition) {
                if(candidate_!=pending||stamp-candidate_last_>.28) {candidate_=pending;candidate_frames_=0;candidate_start_=stamp;}
                candidate_last_=stamp;
                if(++candidate_frames_>=2) {
                    if(pair_)++generation_;
                    bool initial=!pair_;
                    pair_=pending;round_start_=initial&&std::isfinite(activation_)?activation_:candidate_start_;
                    spent_.fill(false);consumed_.fill(false);confirmed_.fill(false);hit_frames_.fill(0);
                    retry_ready_.fill(false);attempts_.fill(0);
                    first_slot_=-1;first_hit_=false;first_impact_=last_impact_=-INFINITY;dark_=-INFINITY;
                    candidate_frames_=0;extra_=false;rescue_ready_=false;miss_confirmed_=false;miss_frames_=0;miss_last_=-INFINITY;
                    followup_ready_=false;follow_frames_=0;follow_last_=-INFINITY;
                }
            } else candidate_frames_=0;
        } else candidate_frames_=0;
        // A rescue can become the first actual hit. Observe both consumed
        // shots; the original missed leaf must not pin the whole round.
        for(int i=0;i<5;++i)if(spent_[i]&&consumed_[i]&&pastImpact(i,stamp)) {
            unsigned bit=1u<<i;
            // Seeing the other pending plate does NOT prove this one hit:
            // the first plate may simply have left the crop during the slew.
            bool hit=!(pending&bit)&&(ordinary&bit);
            hit_frames_[i]=hit?hit_frames_[i]+1:0;
            if(hit_frames_[i]>=2) {
                confirmed_[i]=true;
                if(!first_hit_ || impact_lower_[i]<first_impact_) {
                    first_hit_=true;first_slot_=i;first_impact_=impact_lower_[i];
                }
            }
        }
        // Fresh post-impact images first authorize the other unshot plate.
        // Two consumed but still visibly pending plates can each receive
        // one measured retry. Confirmed hits remain locked; neither a
        // retry nor a delayed image extends the original first-hit window.
        bool consumed=true,unshot=false;
        for(int i=0;i<5;++i)if(pair_&(1u<<i)) {
            unshot=unshot||!spent_[i];
            if(spent_[i] && (!consumed_[i]||!pastImpact(i,stamp)))consumed=false;
        }
        bool unchanged=first_slot_>=0 && !first_hit_ && !extra_ && consumed &&
            pending==pair_ && !(ordinary&pair_);
        if(unchanged) {
            if(stamp-miss_last_>.28)miss_frames_=0;
            if(!miss_frames_)miss_start_=stamp;
            miss_last_=stamp;++miss_frames_;
            rescue_ready_=miss_frames_>=2 && stamp-miss_start_>=.025;
            if(rescue_ready_)miss_confirmed_=true;
            if(rescue_ready_ && !unshot) {
                // Both consumed shots still visibly pending after flight:
                // permit one measured retry per leaf, never a held burst.
                for(int i=0;i<5;++i)retry_ready_[i]=(pair_&(1u<<i))&&spent_[i]&&
                    consumed_[i]&&!confirmed_[i]&&attempts_[i]<2;
            }
        } else {
            rescue_ready_=false;miss_frames_=0;miss_last_=-INFINITY;
            retry_ready_.fill(false);
        }
        bool visible_unshot=false;
        for(int i=0;i<5;++i)visible_unshot=visible_unshot||((pending&(1u<<i))&&!spent_[i]);
        bool follow=first_slot_>=0 && !first_hit_ && !extra_ && consumed_[first_slot_] &&
            pastImpact(first_slot_,stamp) && visible_unshot && !(pending&~pair_);
        if(follow) {
            if(stamp-follow_last_>.28)follow_frames_=0;
            if(!follow_frames_)follow_start_=stamp;
            follow_last_=stamp;++follow_frames_;
            followup_ready_=follow_frames_>=2 && stamp-follow_start_>=.025;
        } else {followup_ready_=false;follow_frames_=0;follow_last_=-INFINITY;}
    }
    int slot(double phase) const {
        int n=int(std::llround(std::remainder(phase-basis_,2*std::acos(-1.))/step()));return(n%5+5)%5;
    }
    bool available(int i) const {return i>=0&&i<5&&(pair_&(1u<<i))&&(!spent_[i]||retry_ready_[i])&&!confirmed_[i];}
    bool retryReady(int i) const {return i>=0&&i<5&&retry_ready_[i];}
    bool completed(int i) const {return i>=0&&i<5&&confirmed_[i];}
    int selectPending(const std::vector<int>& slots,int current) const {
        std::array<bool,5> seen{};int first=-1,count=0;
        for(int i:slots)if(available(i)&&!seen[i]) {seen[i]=true;++count;if(first<0)first=i;}
        if(count>2)return -1;
        return current>=0&&current<5&&seen[current]?current:first;
    }
    void sent(int i,double completion_after) {
        if(!available(i))return;
        ++attempts_[i];retry_ready_.fill(false);
        spent_[i]=true;consumed_[i]=false;completion_[i]=completion_after;
        impact_lower_[i]=completion_after-.10;hit_frames_[i]=0;
        rescue_ready_=false;miss_confirmed_=false;miss_frames_=0;miss_last_=-INFINITY;
        followup_ready_=false;follow_frames_=0;follow_last_=-INFINITY;
        last_impact_=completion_after-.10;
        if(first_slot_<0) {first_slot_=i;first_impact_=last_impact_;}
    }
    void confirmConsumed(int i,double impact_upper=NAN,double impact_lower=NAN) {
        if(i<0||i>=5||!spent_[i]||consumed_[i])return;
        consumed_[i]=true;
        // Ammunition changed between two real state captures. Never inspect
        // an apparent miss before the LATEST possible projectile arrival,
        // and do not extend the second-hit deadline to that upper bound.
        if(std::isfinite(impact_upper))completion_[i]=std::max(completion_[i],impact_upper+.10);
        if(std::isfinite(impact_lower))impact_lower_[i]=std::max(impact_lower_[i],
            std::min(impact_lower,completion_[i]-.10));
        last_impact_=std::max(last_impact_,completion_[i]-.10);
        if(first_slot_==i&&!first_hit_)first_impact_=impact_lower_[i];
    }
    void notFired(int i) {
        if(i<0||i>=5||consumed_[i])return;
        spent_[i]=false;
        if(attempts_[i])--attempts_[i];
        if(first_slot_==i&&!first_hit_)first_slot_=-1;
    }
    void extraConsumed(int) {extra_=true;}
    bool needsRestartView() const {return false;} // Never pull the aim back to R during a round.
    uint64_t generation() const {return generation_;}
    bool rescueReady() const {return rescue_ready_;}
    bool impactInRound(double impact) const {
        if(!pair_||!std::isfinite(impact))return false;
        if(first_slot_<0)return impact<round_start_+2.42;
        if(extra_)return false;
        if(first_hit_)return impact<first_impact_+.90;
        // The one-second SECOND-hit window starts on a VALID hit, not on a
        // shot that fresh post-impact images have shown to miss.
        if(rescue_ready_)return impact<round_start_+2.42;
        // The other unshot plate can be safely hit under BOTH possibilities:
        // first hit or first miss. This does not pretend the cropped first
        // plate is completed; keep both deadlines until a miss is measured.
        return followup_ready_ && impact<round_start_+2.42 && impact<first_impact_+.90;
    }
    std::string blockedReason() const {
        if(!pair_)return "WAIT_BIG_PAIR_CONFIRM";
        if(rescue_ready_)return "WAIT_BIG_RESCUE_DEADLINE";
        return first_slot_>=0&&!first_hit_?"WAIT_BIG_FIRST_HIT_CONFIRM":"WAIT_BIG_NEXT_ROUND";
    }
private:
    bool pastImpact(int i,double stamp) const {return stamp>completion_[i]-.10+.06;}
    static double step() {return 2*std::acos(-1.)/5;}
    double basis_=0,velocity_=0,stamp_=-INFINITY,round_start_=INFINITY,activation_=NAN;
    double first_impact_=-INFINITY,last_impact_=-INFINITY,dark_=-INFINITY;
    double candidate_start_=0,candidate_last_=-INFINITY;
    double dark_start_=-INFINITY,dark_last_=-INFINITY;
    double predicted_stamp_=NAN,predicted_delta_=NAN;
    unsigned pair_=0,candidate_=0;int candidate_frames_=0,first_slot_=-1;
    bool first_hit_=false,extra_=false;
    bool rescue_ready_=false,miss_confirmed_=false;
    bool followup_ready_=false;
    int follow_frames_=0;double follow_start_=0,follow_last_=-INFINITY;
    int miss_frames_=0;double miss_start_=0,miss_last_=-INFINITY;
    uint64_t generation_=0;
    std::array<bool,5> spent_{},consumed_{},confirmed_{};
    std::array<bool,5> retry_ready_{};
    std::array<unsigned,5> attempts_{};
    std::array<int,5> hit_frames_{};
    std::array<double,5> completion_{};
    std::array<double,5> impact_lower_{};
};

// Small state machine is preserved; only explicit big observations dispatch
// into the new round controller. Both share the already tested shot handshake.
class RuneTargetCycle {
public:
    void reset() {small_.reset();big_.reset();big_mode_=false;}
    void beginBig(double stamp) {big_mode_=true;big_.begin(stamp);}
    void predictRotation(double from,double to,double delta,bool big) {
        if(big)big_.predictRotation(from,to,delta);else small_.predictRotation(from,to,delta);
    }
    void observe(const std::vector<RuneLeafObservation>& v,double t,bool big=false,bool full=false) {
        big_mode_=big;if(big)big_.observe(v,t);else small_.observe(v,t,false,full);
    }
    void observeDark(double t) {if(big_mode_)big_.observeDark(t);else small_.observeDark(t);}
    int slot(double p) const {return big_mode_?big_.slot(p):small_.slot(p);}
    bool available(int i) const {return big_mode_?big_.available(i):small_.available(i);}
    bool completed(int i) const {return big_mode_?big_.completed(i):small_.completed(i);}
    uint64_t generation() const {return big_mode_?big_.generation():small_.generation();}
    bool rescueReady() const {return big_mode_&&big_.rescueReady();}
    bool retryReady(int i) const {return big_mode_&&big_.retryReady(i);}
    bool needsRestartView() const {return big_mode_?big_.needsRestartView():small_.needsRestartView();}
    int selectPending(const std::vector<int>& v,int i,bool big) const {return big?big_.selectPending(v,i):small_.selectPending(v,i,false);}
    void sent(int i,double t) {if(big_mode_)big_.sent(i,t);else small_.sent(i,t);}
    void confirmConsumed(int i,double upper=NAN,double lower=NAN) {if(big_mode_)big_.confirmConsumed(i,upper,lower);else small_.confirmConsumed(i);}
    void notFired(int i) {if(big_mode_)big_.notFired(i);else small_.notFired(i);}
    void extraConsumed(int i) {if(big_mode_)big_.extraConsumed(i);else small_.extraConsumed(i);}
    bool impactInRound(double t,bool big) const {return big?big_.impactInRound(t):small_.impactInRound(t,false);}
    std::string blockedReason() const {return big_.blockedReason();}
private:
    RuneLeafCycle small_;BigRuneCycle big_;bool big_mode_=false;
};
} // namespace campus_buff
