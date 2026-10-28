#pragma once

#include "navigation/path_following.hpp"
#include "aiming/reacquisition.hpp"

#include <optional>
#include <array>
#include <cstdint>

namespace campus {
inline constexpr double kFourthLostViewStep = 1.4;
inline constexpr double kFourthNoFireStep = .65;
inline constexpr double kFourthNoFireWait = 2.0;
// Restore the earlier close-position approach for the chassis only. This must
// never be part of anchorReady(): the gimbal may still track during travel.
inline bool fourthRouteApproachPending(int mission_target, double target_range, double goal_distance) {
    return mission_target == 4 && std::isfinite(target_range) && std::isfinite(goal_distance) &&
           target_range > 2.8 && goal_distance > .30;
}
// A convex quadrilateral alone does not establish visibility: two clipped
// light bars still produce four corners and a confident digit classification.
// These checks qualify a chassis stop only; they never filter aiming inputs.
template<class Points> bool completeArmorView(const Points& points, int width, int height, bool maintaining=false) {
    if (width < 32 || height < 32) return false;
    double area=0, sign=0;
    for(int i=0;i<4;++i) {
        const auto& a=points[i];const auto& b=points[(i+1)%4];const auto& c=points[(i+2)%4];
        if(!std::isfinite(a.x)||!std::isfinite(a.y)||a.x<6||a.y<6||a.x>width-6||a.y>height-6) return false;
        double cross=(b.x-a.x)*(c.y-b.y)-(b.y-a.y)*(c.x-b.x);
        if(std::abs(cross)<1e-6 || (sign && cross*sign<0)) return false;
        sign=cross;area+=a.x*b.y-a.y*b.x;
    }
    if(std::abs(area)<32.)return false;
    const auto length=[](const auto& a,const auto& b){return std::hypot(a.x-b.x,a.y-b.y);};
    double left=length(points[0],points[3]),right=length(points[1],points[2]);
    double span=(length(points[0],points[1])+length(points[3],points[2]))*.5;
    // Installed mesh: 125 x 58.5 mm, tilted 15 degrees. Full, frontal
    // bars give ~2.2; allow perspective/pixel error, but reject short stubs.
    // Hysteresis applies only after a complete view has established a station.
    // Short, distant bars quantize by several pixels as the turret settles;
    // that is not evidence that the chassis should leave the station again.
    return maintaining ? std::min(left,right)>=.65*std::max(left,right) && span<=2.8*(left+right)*.5
                       : std::min(left,right)>=.8*std::max(left,right) && span<=2.65*std::min(left,right);
}
class FourthArmorView {
public:
    void reset() { stamp_=-INFINITY;good_=-INFINITY;started_=0;frames_=0; }
    void observe(double stamp,double received,bool complete,bool maintained=false) {
        if(!std::isfinite(stamp)||!std::isfinite(received)||stamp<=stamp_) return;
        const bool existing=ready(received);
        if(!complete || stamp-stamp_>.25)frames_=0;
        if(complete && frames_++==0)started_=stamp;
        stamp_=stamp;
        // Confirm measured geometry across exposures, without requiring a
        // high frame rate. A usable 5--10 Hz stream must still stop the car.
        if(complete && frames_>=2 && stamp-started_>=.04-1e-6) good_=received;
        if(existing && maintained)good_=received;
    }
    bool ready(double now) const { return now>=good_ && now-good_<=.35; }
private:
    double stamp_=-INFINITY,good_=-INFINITY,started_=0;int frames_=0;
};
// A measured complete view owns the stop, including during a forward retry.
// Damage feedback and distance to the planned endpoint cannot revoke it.
// Only loss of the complete view can release the normal checked approach.
class FourthArmorHoldProgress {
public:
    void reset() { initialized_=advancing_=false; }
    void settled(double,PlanPoint position) {
        advancing_=false;anchor_=position;
    }
    bool update(double time,PlanPoint position,int health,bool complete) {
        if(!std::isfinite(time)||!std::isfinite(position.x)||!std::isfinite(position.y)||health<=0) {
            reset();return false;
        }
        if(initialized_ && time<last_)reset();
        last_=time;
        if(complete) {
            initialized_=true;advancing_=false;anchor_=position;
            return true;
        }
        if(!initialized_)return false;
        if(advancing_) {
            if(pointDistance(anchor_,position)<kFourthLostViewStep && time-advance_started_<5.)return false;
            advancing_=false;anchor_=position;
            return false;
        }
        // The upstream visual lease already tolerates brief gaps. Once it
        // expires, start a bounded braking segment immediately; do not first
        // accelerate along the whole route while a retry timer runs.
        advancing_=true;anchor_=position;advance_started_=time;
        return false;
    }
    bool advancing() const { return advancing_; }
private:
    bool initialized_=false,advancing_=false;
    double last_=0,advance_started_=0;
    PlanPoint anchor_;
};

class FourthNoFireAdvance {
public:
    void reset() { initialized_=attempted_=active_=holding_=false; }
    void finish() { active_=false; }
    bool active() const { return active_; }
    bool update(double time, PlanPoint position, int ammo, int health, bool allowed, bool holding) {
        if (!std::isfinite(time) || !std::isfinite(position.x) || !std::isfinite(position.y) || ammo < 0 || health <= 0) {
            reset(); return false;
        }
        if (initialized_ && (time < last_ || health > health_ || pointDistance(anchor_,position) > .8)) reset();
        if (!initialized_) {
            initialized_=true;anchor_=position;ammo_=ammo;health_=health;progress_=time;
        }
        last_=time;
        if (holding && !holding_ && !active_) progress_=time;
        holding_=holding;
        const bool fired=ammo<ammo_ || health<health_;
        if (ammo!=ammo_ || fired) progress_=time;
        ammo_=ammo;health_=health;
        if (!allowed || ammo<=0) { active_=false;progress_=time;return false; }
        if (fired && active_) { active_=false;return false; }
        if (active_) {
            if (time-started_>=3.) active_=false;
            return active_;
        }
        if (!holding) { progress_=time;if(!attempted_)anchor_=position; }
        if (holding && !attempted_ && time-progress_>=kFourthNoFireWait) {
            active_=attempted_=true;started_=time;
        }
        return active_;
    }
private:
    bool initialized_=false,attempted_=false,active_=false,holding_=false;
    double last_=0,progress_=0,started_=0;
    int ammo_=0,health_=0;
    PlanPoint anchor_;
};

// Freeze one short section of the existing route, preserving its bends.
// Giving the follower this endpoint lets its normal braking stop the retry;
// a moving distance threshold alone can overshoot a useful firing view.
inline std::vector<PlanPoint> forwardRouteSegment(const std::vector<PlanPoint>& path,PlanPoint position,double distance) {
    if(path.size()<2 || !std::isfinite(position.x)||!std::isfinite(position.y)||!std::isfinite(distance)||distance<=0)return {};
    size_t next=0;PlanPoint start;double best=INFINITY;
    for(size_t i=1;i<path.size();++i) {
        auto a=path[i-1],b=path[i];double dx=b.x-a.x,dy=b.y-a.y,length2=dx*dx+dy*dy;
        if(!std::isfinite(length2)||length2<1e-12)continue;
        double f=std::clamp(((position.x-a.x)*dx+(position.y-a.y)*dy)/length2,0.,1.);
        PlanPoint p{a.x+f*dx,a.y+f*dy};double error=pointDistance(position,p);
        if(error<best) {best=error;start=p;next=i;}
    }
    if(!std::isfinite(best))return {};
    std::vector<PlanPoint> result{start};
    for(size_t i=next;i<path.size();++i) {
        auto a=result.back(),b=path[i];double length=pointDistance(a,b);
        if(!std::isfinite(length))return {};
        if(length<1e-9)continue;
        if(length>=distance) {
            result.push_back({a.x+(b.x-a.x)*distance/length,a.y+(b.y-a.y)*distance/length});break;
        }
        result.push_back(b);distance-=length;
    }
    return result.size()>=2?result:std::vector<PlanPoint>{};
}
// A health decrease is evidence of damage, not evidence of who fired.
// Legacy mode requires a freshly observed, identified mobile opponent;
// assault mode separately admits enemy-colored tracked armor and structures.
inline bool combatRobot(int number, const std::string& state, double observation_age) {
    return ((number >= 1 && number <= 5) || number == 7) && state == "TRACKING" && std::isfinite(observation_age) &&
           observation_age >= 0 && observation_age <= 0.2;
}

inline bool combatObservation(int number, const std::string& state, double age, bool structures, bool unknown) {
    return ((number >= 1 && number <= 5) || number == 7 || (structures && (number == 6 || number == 8)) ||
            (unknown && number == 0)) &&
           (state == "TRACKING" || state == "DETECTING") && std::isfinite(age) && age >= 0 && age <= .2;
}

inline bool routeThreat(PlanPoint own, PlanPoint enemy, const std::vector<PlanPoint>& path) {
    double distance = pointDistance(own, enemy);
    if (distance <= 4.)
        return true;
    if (distance > 8.)
        return false;

    for (const auto& point : path)
        if (pointDistance(point, enemy) <= 3.)
            return true;
    return false;
}

// A stop belongs to a physical observation position, not a volatile track ID.
// Intermittent detections and alternating robot identities cannot keep renewing
// it. After an unproductive stop, give the normal checked path follower enough
// time to move or report a blocked route to the planner.
class CombatStopProgress {
  public:
    void reset() {
        initialized_ = advancing_ = false;
    }

    bool update(double time, PlanPoint position, const std::array<int32_t, 7>& healths, bool mobile_contact,
                double advance_distance = .6, bool complete_view = false) {
        if (!std::isfinite(time) || !std::isfinite(position.x) || !std::isfinite(position.y)) {
            reset();
            return false;
        }

        if (!initialized_ || time < last_) {
            if (!mobile_contact)
                return false;
            initialized_ = true;
            anchor_ = position;
            progress_ = time;
            healths_ = healths;
        }

        last_ = time;
        bool damage = false;
        for (size_t i = 0; i < 5; ++i)
            damage = damage || (healths[i] >= 0 && healths[i] < healths_[i]);
        healths_ = healths;

        // A measured whole fourth plate is an explicit stop, including during
        // a retry. No damage may mean the gun has not fired yet; it must not
        // force the chassis past the available line of sight. Only a sustained
        // loss of this view permits the normal no-progress recovery again.
        if (complete_view) {
            advancing_ = false;
            anchor_ = position;
            progress_ = time;
            return false;
        }

        if (advancing_) {
            if (pointDistance(anchor_, position) < advance_distance && time - advance_started_ < 7.)
                return true;
            advancing_ = false;
            anchor_ = position;
            progress_ = time;
        }

        if (damage || pointDistance(anchor_, position) >= .3) {
            anchor_ = position;
            progress_ = time;
        }

        if (mobile_contact && time - progress_ >= 3.) {
            advancing_ = true;
            anchor_ = position;
            advance_started_ = time;
        }

        return advancing_;
    }

  private:
    bool initialized_ = false, advancing_ = false;
    double last_ = 0, progress_ = 0, advance_started_ = 0;
    PlanPoint anchor_;
    std::array<int32_t, 7> healths_{};
};

// Track real damage, not commands sent or unstable tracker IDs. Occasional
// shots and short occlusions must not indefinitely pin the chassis in place.
class CombatApproachProgress {
  public:
    void reset() {
        number_ = 0;
        attempts_ = 0;
        seen_ = progress_ = started_ = -INFINITY;
    }

    void observe(int number, int health, double time) {
        if (number <= 0 || health <= 0 || !std::isfinite(time))
            return;
        if (number != number_ || time < seen_) {
            number_ = number;
            health_ = health;
            progress_ = started_ = time;
            attempts_ = 0;
        }

        if (health < health_) {
            progress_ = time;
            attempts_ = 0;
        }

        if (time - seen_ > 1.)
            progress_ = started_ = time;
        health_ = health;
        seen_ = time;
    }

    bool needsAdjustment(double time, double range = 0., double distant_range=3.8) const {
        bool distant = std::isfinite(range) && range > distant_range && time - started_ >= 3.;
        return number_ > 0 && time >= seen_ && time - seen_ <= .35 && (time - progress_ >= 3. || distant) &&
               attempts_ < 3;
    }

    void adjusted(double time) {
        progress_ = started_ = time;
        ++attempts_;
    }

    void settled(double time) {
        progress_ = started_ = time;
    }

  private:
    int number_ = 0, health_ = 0, attempts_ = 0;
    double seen_ = -INFINITY, progress_ = -INFINITY, started_ = -INFINITY;
};

// Keep normal opponents unchanged; vehicle four gets a longer forward
// approach, shortened only to preserve the existing 2.4 m stand-off.
inline double combatApproachStep(int number, double range) {
    return number == 4 ? std::clamp(range - 2.4, 0., 2.4) : .6;
}

inline std::vector<PlanPoint> combatApproachWaypoints(const DijkstraGrid& grid, PlanPoint position, PlanPoint enemy, double step=.6) {
    std::vector<PlanPoint> result;
    double range = pointDistance(position, enemy);
    if (!grid.valid() || !std::isfinite(range) || range < 2.9)
        return result;
    double bearing = std::atan2(enemy.y - position.y, enemy.x - position.x);

    for (double distance : {step, step*.75, step*.5})
        for (double turn : {0., M_PI / 6, -M_PI / 6, M_PI / 3, -M_PI / 3}) {
            PlanPoint goal{position.x + distance * std::cos(bearing + turn),
                           position.y + distance * std::sin(bearing + turn)};
            if (pointDistance(goal, enemy) >= 2.4 && segmentFree(grid, position, goal))
                result.push_back(goal);
        }

    return result;
}

inline std::optional<PlanPoint> combatApproachWaypoint(const DijkstraGrid& grid, PlanPoint position, PlanPoint enemy) {
    auto candidates = combatApproachWaypoints(grid, position, enemy);
    if (candidates.empty())
        return {};
    return candidates.front();
}

class CombatPolicy {
  public:
    void reset() {
        seen_until_ = evade_until_ = reacquire_until_ = -INFINITY;
        target_id = 0;
    }

    // Transport faults still stop/cancel the motion plan. Retain only a
    // bounded visual contact for search after valid feedback/plan recover.
    void pause() {
        seen_until_ = evade_until_ = -INFINITY;
    }

    void see(uint32_t id, PlanPoint point, double time, bool confirmed = true, bool mobile = true) {
        if (!id || !std::isfinite(point.x) || !std::isfinite(point.y))
            return;
        target_id = id;
        enemy = point;
        seen_until_ = time + 0.35;
        confirmed_ = confirmed;
        mobile_enemy = mobile;
        reacquire_until_ = time + kReacquireTimeout;
    }

    void damaged(double time) {
        evade_until_ = time + 1.5;
        // Alternate with time, not every feedback packet.
        if (time - last_damage_ > 0.5)
            side_ = -side_;
        last_damage_ = time;
    }

    void damaged(double time, PlanPoint position, PlanPoint velocity) {
        if (time > seen_until_) {
            double speed = std::hypot(velocity.x, velocity.y);
            enemy = {position.x + (speed > .05 ? velocity.x / speed : 1.) * 4,
                     position.y + (speed > .05 ? velocity.y / speed : 0.) * 4};
        }

        damaged(time);
    }

    bool freshEnemy(double time) const {
        return time <= seen_until_;
    }

    std::string phase(double time) const {
        if (time < evade_until_)
            return "EVADE";
        if (time > seen_until_)
            return time <= reacquire_until_ ? "REACQUIRE" : "NAVIGATE";
        return confirmed_ ? "ENGAGE" : "ACQUIRE";
    }

    // Only choose a waypoint whose whole segment is free in the inflated map.
    // The caller must additionally check dynamic obstacles and full braking.
    PlanPoint dodge(const DijkstraGrid& grid, PlanPoint position, double time) const {
        double dx = enemy.x - position.x, dy = enemy.y - position.y;
        double length = std::hypot(dx, dy);
        if (length < 1e-6 || !grid.valid())
            return position;
        int side = (static_cast<int>(std::floor(time / 0.65)) % 2 ? -1 : 1) * side_;
        PlanPoint perpendicular{-dy / length, dx / length};

        for (double distance : {0.7, 0.45, 0.25}) {
            for (int direction : {side, -side}) {
                PlanPoint goal{position.x + direction * distance * perpendicular.x,
                               position.y + direction * distance * perpendicular.y};
                if (segmentFree(grid, position, goal))
                    return goal;
            }
        }

        PlanPoint retreat{position.x - 0.4 * dx / length, position.y - 0.4 * dy / length};
        return segmentFree(grid, position, retreat) ? retreat : position;
    }

    uint32_t target_id = 0;
    bool mobile_enemy = true;
    PlanPoint enemy;

  private:
    double seen_until_ = -INFINITY, evade_until_ = -INFINITY, reacquire_until_ = -INFINITY, last_damage_ = -INFINITY;
    int side_ = 1;
    bool confirmed_ = true;
};
} // namespace campus
