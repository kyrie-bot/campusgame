#pragma once

#include "path_following.hpp"

#include <array>

namespace campus {
// Per-match memories use real health transitions and measured own position.
class OpeningAndAnchor {
  public:
    void reset() {
        opening_done = false;
        sentry_advanced = false;
        anchor_known = false;
        anchor_from_four = false;
        health_known = false;
        last_four = 0;
        four_view_known=false;
        rejected_sentry.clear();
    }

    void observe(int four_health, PlanPoint own, bool capture = true) {
        if (health_known && last_four <= 0 && four_health > 0) {
            anchor_known = false;
            anchor_from_four = false;
            sentry_advanced = false;
            four_view_known=false;
            rejected_sentry.clear();
        }

        if (capture && health_known && last_four > 0 && four_health <= 0 && std::isfinite(own.x) &&
            std::isfinite(own.y)) {
            anchor = four_view_known ? four_view : own;
            four_origin = anchor;
            anchor_known = true;
            anchor_from_four = true;
            sentry_advanced = false;
        }

        last_four = four_health;
        health_known = true;
    }

    void fallback(PlanPoint own) {
        if (!anchor_known) {
            anchor = own;
            four_origin = own;
            anchor_known = true;
            anchor_from_four = false;
        }
    }

    bool sentry_advanced=false;
    void rememberFourView(PlanPoint p) {
        if(std::isfinite(p.x) && std::isfinite(p.y)) {four_view=p;four_view_known=true;}
    }
    void rejectSentryStation(PlanPoint p) {
        if(!std::isfinite(p.x)||!std::isfinite(p.y))return;
        if(sentryStationAllowed(p))rejected_sentry.push_back(p);
    }
    bool sentryStationAllowed(PlanPoint p) const {
        for(auto q:rejected_sentry)if(pointDistance(p,q)<.55)return false;
        return true;
    }
    std::vector<PlanPoint> rejected_sentry;
    bool opening_done = false, anchor_known = false, anchor_from_four = false;
    PlanPoint anchor, four_origin;

  private:
    bool health_known = false;
    bool four_view_known=false;
    PlanPoint four_view;
    int last_four = 0;
};

inline bool fieldRobotsCleared(const std::array<int32_t, 7>& hp) {
    return hp[1] <= 0 && hp[2] <= 0 && hp[3] <= 0;
}

// An anchor circle is not a terrain classification: it can cross a ramp lip.
// These cells are the independently supplied, inset highland firing surface.
inline bool onCombatHighland(const DijkstraGrid& grid, const std::vector<int64_t>& cells, PlanPoint p) {
    int cell = grid.cellAt(p);
    return cell >= 0 && std::binary_search(cells.begin(), cells.end(), cell);
}

// A goal on the last highland cell can be reached within the follower's
// 15 cm stop tolerance while the actual robot is still outside that cell.
// Require the whole arrival disk to belong to the firing surface.
inline bool highlandArrivalSafe(const DijkstraGrid& grid, const std::vector<int64_t>& cells,
                                PlanPoint p, double radius=.25) {
    if (!onCombatHighland(grid,cells,p)) return false;
    const double half=grid.resolution*.5;
    const int extent=static_cast<int>(std::ceil(radius/grid.resolution))+1;
    const int centre=grid.cellAt(p), cx=centre%grid.width, cy=centre/grid.width;
    for(int dy=-extent;dy<=extent;++dy) for(int dx=-extent;dx<=extent;++dx) {
        const int x=cx+dx,y=cy+dy;
        PlanPoint q{grid.origin_x+(x+.5)*grid.resolution,grid.origin_y+(y+.5)*grid.resolution};
        const double ex=std::max(0.,std::abs(q.x-p.x)-half);
        const double ey=std::max(0.,std::abs(q.y-p.y)-half);
        if(std::hypot(ex,ey)>radius) continue;
        if(x<0 || y<0 || x>=grid.width || y>=grid.height ||
           !std::binary_search(cells.begin(),cells.end(),int64_t(y*grid.width+x))) return false;
    }
    return true;
}

// Remember the measured descent, rather than routing from the rune to a new
// enemy goal. The return remains a Dijkstra plan inside the same passage.
class RuneReturnRoute {
  public:
    void reset() { trail.clear(); }
    bool available() const { return trail.size() > 1; }
    PlanPoint destination() const { return trail.front(); }
    void record(PlanPoint p, bool on_highland) {
        if (on_highland) { trail.assign(1, p); return; }
        if (!trail.empty() && pointDistance(p, trail.back()) >= .35)
            trail.push_back(p);
    }
    bool reached(PlanPoint p, bool on_highland) const {
        return available() && on_highland && pointDistance(p, destination()) <= .4;
    }
    bool prepareArrival(const DijkstraGrid& grid, const std::vector<int64_t>& highland,
                        const std::vector<double>& costs) {
        if (!available() || costs.size()!=grid.cells.size()) return false;
        auto original=destination();
        double best=INFINITY;
        PlanPoint goal;
        // Keep the return in the same ramp entrance, but stop far enough onto
        // the highland to agree with the follower's physical arrival tolerance.
        for(auto cell:highland) {
            if(cell<0 || size_t(cell)>=costs.size() || grid.cells[cell]!=0 || !std::isfinite(costs[cell])) continue;
            auto p=grid.centre(static_cast<int>(cell));
            double distance=pointDistance(p,original);
            if(distance>.8 || !highlandArrivalSafe(grid,highland,p)) continue;
            double score=distance+.001*costs[cell];
            if(score<best) {best=score;goal=p;}
        }
        if(!std::isfinite(best)) return false;
        if(pointDistance(goal,original)>.001) trail.insert(trail.begin(),goal);
        return true;
    }
    void restrictGrid(DijkstraGrid& grid) const {
        if (!available()) return;
        for (size_t cell=0;cell<grid.cells.size();++cell) {
            if (grid.cells[cell] != 0) continue;
            auto p=grid.centre(static_cast<int>(cell));
            bool near=false;
            for(size_t i=1;i<trail.size();++i) {
                auto a=trail[i-1], b=trail[i];
                double dx=b.x-a.x, dy=b.y-a.y, length=dx*dx+dy*dy;
                double t=length>0 ? std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/length,0.,1.) : 0.;
                if(std::hypot(p.x-a.x-t*dx,p.y-a.y-t*dy)<=1.1) { near=true;break; }
            }
            if(!near) grid.cells[cell]=100;
        }
    }
  private:
    std::vector<PlanPoint> trail;
};

inline void restrictCombatHighland(DijkstraGrid& grid, const std::vector<int64_t>& cells) {
    std::vector<bool> allowed(grid.cells.size(),false);
    for (auto cell : cells)
        if (cell >= 0 && size_t(cell) < allowed.size()) allowed[cell]=true;
    for (size_t i=0;i<grid.cells.size();++i)
        if (!allowed[i]) grid.cells[i]=100;
}

inline bool anchorContains(PlanPoint p, PlanPoint centre, double radius, double inset = .35) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(centre.x) && std::isfinite(centre.y) &&
           std::isfinite(radius) && radius >= .8 && pointDistance(p, centre) <= radius - inset;
}

inline void restrictAnchorGrid(DijkstraGrid& grid, PlanPoint centre, double radius) {
    for (int cell = 0; cell < static_cast<int>(grid.cells.size()); ++cell)
        if (!anchorContains(grid.centre(cell), centre, radius, .2))
            grid.cells[cell] = 100;
}

inline bool chooseOpeningRamp(const DijkstraGrid& grid, const std::vector<double>& costs,
                              const std::vector<int64_t>& cells, int player, PlanPoint& goal) {
    PlanPoint desired = player == 1 ? PlanPoint{13.75, 1.75} : PlanPoint{14.25, 13.25};
    double best = INFINITY;
    for (auto cell : cells) {
        if (cell < 0 || cell >= static_cast<int64_t>(grid.cells.size()) || grid.cells[cell] != 0 ||
            !std::isfinite(costs[cell]))
            continue;
        auto p = grid.centre(cell);

        if (pointDistance(p, desired) > .6 || !segmentWithReserve(grid, p, p, .03))
            continue;
        double score = 10 * pointDistance(p, desired) + .01 * costs[cell];
        if (score < best) {
            best = score;
            goal = p;
        }
    }

    return std::isfinite(best);
}

// Opening ascent and rune trips share one centred ramp entrance. Real occupied
// cells remain occupied; a blocked entrance never authorizes another ramp.
inline bool restrictRuneRampTransit(DijkstraGrid& grid, const std::vector<int64_t>& ramp,
                                    int player, PlanPoint start, std::vector<double>* risk=nullptr,
                                    const std::vector<int64_t>* all_ramps=nullptr) {
    if(!grid.valid() || ramp.empty()) return false;
    PlanPoint centre=player==1?PlanPoint{13.75,1.75}:PlanPoint{14.25,13.25};
    double low=INFINITY,high=-INFINITY;
    for(auto cell:ramp) if(cell>=0 && size_t(cell)<grid.cells.size()) {
        auto p=grid.centre(static_cast<int>(cell));
        if(std::abs(p.y-centre.y)<2.) {low=std::min(low,p.x);high=std::max(high,p.x);}
    }
    if(!std::isfinite(low)) return false;
    bool needs_entry=start.x>=low-.1 && start.x<=high+.1 && std::abs(start.y-centre.y)>.30;
    const auto& surfaces=all_ramps && !all_ramps->empty()?*all_ramps:ramp;
    for(auto cell:surfaces) {
        if(cell<0 || size_t(cell)>=grid.cells.size() || grid.cells[cell]!=0) continue;
        auto p=grid.centre(static_cast<int>(cell));
        bool selected=p.x>=low-.01 && p.x<=high+.01 && std::abs(p.y-centre.y)<2.;
        bool join=needs_entry && std::abs(p.x-start.x)<=.45 &&
            p.y>=std::min(start.y,centre.y)-.15 && p.y<=std::max(start.y,centre.y)+.15;
        if(!selected || (std::abs(p.y-centre.y)>.30 && !join)) grid.cells[cell]=100;
    }
    // Centre the approach with a cost, not a map-wide virtual wall. A rune
    // return interrupted from the outpost may start elsewhere on the deck.
    for(size_t i=0;i<grid.cells.size();++i) {
        if(grid.cells[i]!=0) continue;
        auto p=grid.centre(static_cast<int>(i));
        if(p.x<low-1. || p.x>high+1.) continue;
        double offset=std::abs(p.y-centre.y);
        if(offset<1. && risk && risk->size()==grid.cells.size()) (*risk)[i]+=4.*std::pow(offset/.30,2);
    }
    return true;
}

inline GridPlan planRuneRampTransit(const DijkstraGrid& grid, PlanPoint start, PlanPoint goal,
                                    int player, const std::vector<double>& risk) {
    PlanPoint centre=player==1?PlanPoint{13.75,1.75}:PlanPoint{14.25,13.25};
    if((start.x-centre.x)*(goal.x-centre.x)>=0) return grid.plan(start,goal,risk);
    auto first=grid.plan(start,centre,risk);
    if(first.status!="OK") {
        if(first.status=="GOAL_BLOCKED")first.status="NO_PATH";
        return first;
    }
    auto second=grid.plan(centre,goal,risk);
    if(second.status!="OK")return second;
    first.cells.insert(first.cells.end(),second.cells.begin()+1,second.cells.end());
    first.length_m+=second.length_m;first.total_cost+=second.total_cost;first.expanded+=second.expanded;
    return first;
}

// Brief FOLLOWING samples between obstacle stops do not constitute progress.
// Reset the blocked interval only after measured displacement or a new task.
class ApproachProgress {
  public:
    void reset() {
        initialized_ = blocked_ = false;
        last_time_ = -1;
    }

    void update(const std::string& status, PlanPoint position, double time) {
        bool approach = status == "FOLLOWING" || status == "LOCAL_BLOCKED" || status == "WAIT_OBSTACLE";
        if (!approach || !std::isfinite(time) || !std::isfinite(position.x) || !std::isfinite(position.y)) {
            reset();
            return;
        }

        if (!initialized_ || time < last_time_ || time - last_time_ > .5 || pointDistance(position, origin_) >= .1) {
            origin_ = position;
            since_ = time;
            blocked_ = false;
            initialized_ = true;
        }

        last_time_ = time;
        if (status == "LOCAL_BLOCKED" || status == "WAIT_OBSTACLE") {
            blocked_ = true;
            last_block_ = time;
        }
    }

    bool needsReplan(double time) const {
        return initialized_ && blocked_ && time >= last_time_ && time - last_time_ <= .5 && time - last_block_ <= .5 &&
               time - since_ >= 1.2;
    }

  private:
    PlanPoint origin_;
    bool initialized_ = false, blocked_ = false;
    double since_ = 0, last_time_ = -1, last_block_ = -INFINITY;
};

inline std::string assaultObjective(const std::array<int32_t, 7>& healths, int ammo, bool weakened) {
    if (healths[6] <= 0)
        return "COMPLETE";
    if (ammo <= 0 || weakened)
        return "REFILL";
    return healths[5] > 0 ? "OUTPOST" : "BASE";
}

inline std::string orderedAssaultObjective(const std::array<int32_t, 7>& healths, int ammo, bool weakened,
                                           bool ignore_one = false) {
    if (healths[6] <= 0)
        return "COMPLETE";
    if (ammo <= 0 || weakened)
        return "REFILL";
    for (int i = ignore_one ? 1 : 0; i < 5; ++i)
        if (healths[i] > 0)
            return "CLEAR_ROBOTS";
    return healths[5] > 0 ? "OUTPOST" : "BASE";
}

class SpawnRefill {
  public:
    void reset() {
        captured_ = active_ = false;
        stopped_since_ = last_time_ = -1;
    }

    void capture(PlanPoint position) {
        if (!captured_ && std::isfinite(position.x) && std::isfinite(position.y)) {
            point_ = position;
            captured_ = true;
        }
    }

    void begin() {
        if (!active_) {
            active_ = true;
            stopped_since_ = -1;
        }
    }

    bool update(PlanPoint position, PlanPoint velocity, int ammo, int capacity, bool weakened, double time) {
        if (!active_ || !captured_ || !std::isfinite(time))
            return false;
        double speed = std::hypot(velocity.x, velocity.y);
        if (!std::isfinite(speed) || !std::isfinite(position.x) || !std::isfinite(position.y) ||
            pointDistance(position, point_) > .15 || speed > .03 || time < last_time_ || time - last_time_ > .3)
            stopped_since_ = -1;
        last_time_ = time;

        if (!std::isfinite(position.x) || !std::isfinite(position.y) || pointDistance(position, point_) > .15 ||
            !std::isfinite(speed) || speed > .03)
            return false;
        if (stopped_since_ < 0)
            stopped_since_ = time;
        if (time - stopped_since_ >= 3. && capacity > 0 && ammo >= capacity && !weakened) {
            active_ = false;
            stopped_since_ = -1;
            return true;
        }

        return false;
    }

    bool captured() const {
        return captured_;
    }

    bool active() const {
        return active_;
    }

    PlanPoint point() const {
        return point_;
    }

  private:
    PlanPoint point_;
    bool captured_ = false, active_ = false;
    double stopped_since_ = -1, last_time_ = -1;
};

// One weighted Dijkstra expansion ranks all candidate attack positions. Do not
// run a separate full search for every point on the ring at each control tick.
inline std::vector<double> routeCosts(const DijkstraGrid& g, PlanPoint start, const std::vector<double>& risk) {
    std::vector<double> cost(g.cells.size(), INFINITY);
    int first = g.cellAt(start);
    if (!g.valid() || first < 0 || g.cells[first] != 0 || risk.size() != g.cells.size())
        return cost;
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> q;
    cost[first] = 0;
    q.emplace(0, first);

    while (!q.empty()) {
        auto [c, i] = q.top();
        q.pop();
        if (c != cost[i])
            continue;

        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) {
                int x = i % g.width, y = i / g.width, nx = x + dx, ny = y + dy;
                if (!g.canStep(x, y, nx, ny))
                    continue;
                int next = ny * g.width + nx;
                double step = g.resolution * (dx && dy ? std::sqrt(2.) : 1.) * (1 + (risk[i] + risk[next]) * .5);

                if (c + step < cost[next]) {
                    cost[next] = c + step;
                    q.emplace(cost[next], next);
                }
            }
    }

    return cost;
}

inline bool attackRayFree(const DijkstraGrid& raw, PlanPoint point, PlanPoint enemy, double body_radius) {
    double d = pointDistance(point, enemy);
    if (d <= body_radius + .2)
        return false;
    double f = (d - body_radius - .2) / d;
    return segmentFree(raw, point, {point.x + (enemy.x - point.x) * f, point.y + (enemy.y - point.y) * f});
}

inline bool chooseAnchorPosition(const DijkstraGrid& grid, const DijkstraGrid& raw, const std::vector<double>& costs,
                                 PlanPoint centre, double radius, PlanPoint enemy, double body, PlanPoint& goal,
                                 PlanPoint excluded = {INFINITY, INFINITY}) {
    double best = INFINITY;
    for (int cell = 0; cell < static_cast<int>(grid.cells.size()); ++cell) {
        if (grid.cells[cell] != 0 || !std::isfinite(costs[cell]))
            continue;
        auto p = grid.centre(cell);

        if (!anchorContains(p, centre, radius, .4) || pointDistance(p, excluded) < .6 ||
            !segmentWithReserve(grid, p, p, .03) || !attackRayFree(raw, p, enemy, body))
            continue;
        double score = costs[cell] + .5 * pointDistance(p, centre);
        if (score < best) {
            best = score;
            goal = p;
        }
    }

    return std::isfinite(best);
}

// Returning from a rune/refill requires a reachable anchor, not immediate
// line of fire at a moving sentry. Firing remains guarded by actual vision.
inline bool chooseAnchorReturn(const DijkstraGrid& grid, const std::vector<double>& costs,
                               PlanPoint centre, double radius, PlanPoint& goal) {
    double best=INFINITY;
    for(int cell=0;cell<int(grid.cells.size());++cell) {
        if(grid.cells[cell]!=0 || cell>=int(costs.size()) || !std::isfinite(costs[cell])) continue;
        auto p=grid.centre(cell);
        if(!anchorContains(p,centre,radius,.4) || !segmentWithReserve(grid,p,p,.03)) continue;
        double score=costs[cell]+2*pointDistance(p,centre);
        if(score<best) { best=score;goal=p; }
    }
    return std::isfinite(best);
}

inline bool stoppedAttackPosition(const DijkstraGrid& g, const DijkstraGrid& raw, PlanPoint position, PlanPoint enemy,
                                  double radius, PlanPoint& goal) {
    int cell = g.cellAt(position);
    if (!g.valid() || cell < 0 || g.cells[cell] != 0)
        return false;
    auto point = g.centre(cell);
    double range = pointDistance(point, enemy);

    if (range < 2.4 || range > 6. || !attackRayFree(raw, point, enemy, radius))
        return false;
    goal = point;
    return true;
}

struct StandoffBand {
    double minimum = 2.8, preferred = 3.5, maximum = 4.2;
    bool valid() const {
        return std::isfinite(minimum) && std::isfinite(preferred) && std::isfinite(maximum) && minimum >= 1.8 &&
               minimum < preferred && preferred < maximum && maximum - minimum >= .6 && maximum <= 8.;
    }

    bool contains(double distance) const {
        return valid() && std::isfinite(distance) && distance >= minimum && distance <= maximum;
    }
};

// Retreat and approach use the same reachable annulus. Snapped cells must
// remain inside it, and every shooting ray must pass the raw map check.
inline bool chooseStandoffPosition(const DijkstraGrid& grid, const DijkstraGrid& raw, const std::vector<double>& costs,
                                   PlanPoint enemy, double body_radius, const StandoffBand& band, PlanPoint& goal,
                                   PlanPoint excluded = {INFINITY, INFINITY}) {
    if (!band.valid() || costs.size() != grid.cells.size())
        return false;
    double best = INFINITY;
    for (double radius : {band.preferred, (band.minimum + band.preferred) * .5, (band.preferred + band.maximum) * .5,
                          band.minimum + .3, band.maximum - .3})
        for (int sector = 0; sector < 64; ++sector) {
            double angle = sector * 2 * M_PI / 64;
            int cell = grid.cellAt({enemy.x + radius * std::cos(angle), enemy.y + radius * std::sin(angle)});

            if (cell < 0 || grid.cells[cell] != 0 || !std::isfinite(costs[cell]))
                continue;
            auto point = grid.centre(cell);
            double distance = pointDistance(point, enemy);

            // Leave room for the follower's 0.15 m arrival tolerance: a settled
            // robot must still be inside the shooting band, not stuck at its edge.
            if (!band.contains(distance) || distance < band.minimum + .2 || distance > band.maximum - .2 ||
                pointDistance(point, excluded) < .6 || !attackRayFree(raw, point, enemy, body_radius))
                continue;
            double score = costs[cell] + 2. * std::abs(distance - band.preferred);
            if (score < best) {
                best = score;
                goal = point;
            }
        }

    return std::isfinite(best);
}

// Stay in the original fourth-vehicle neighbourhood and on the same side
// of the outpost. This applies to the route and local recovery, not just goals.
inline bool sentryLocalContains(PlanPoint p, PlanPoint origin, PlanPoint outpost, bool known) {
    if(pointDistance(p,origin)>1.7) return false;
    if(!known) return true;
    double dx=origin.x-outpost.x,dy=origin.y-outpost.y,length=std::hypot(dx,dy);
    return length<.05 || ((p.x-outpost.x)*dx+(p.y-outpost.y)*dy)/length>=.05;
}
inline void restrictSentryLocal(DijkstraGrid& grid, PlanPoint origin, PlanPoint outpost, bool known) {
    for(size_t i=0;i<grid.cells.size();++i)
        if(!sentryLocalContains(grid.centre(static_cast<int>(i)),origin,outpost,known)) grid.cells[i]=100;
}
// A short, directly reachable step from the fourth vehicle's original pose.
// Never replace a blocked local step with a distant highland frontier.
inline bool sentrySightClear(const DijkstraGrid& raw, PlanPoint p, PlanPoint enemy, double body,
                              PlanPoint outpost, bool known, double outpost_radius=.6) {
    if(!attackRayFree(raw,p,enemy,body)) return false;
    if(!known) return true;
    const double dx=enemy.x-p.x,dy=enemy.y-p.y,length=dx*dx+dy*dy;
    if(length<1e-6) return false;
    double t=((outpost.x-p.x)*dx+(outpost.y-p.y)*dy)/length;
    return t<=0 || t>=1 || std::hypot(outpost.x-p.x-t*dx,outpost.y-p.y-t*dy)>outpost_radius+.1;
}

inline bool chooseSentryFrontier(const DijkstraGrid& grid, const DijkstraGrid& raw,
        const std::vector<int64_t>& highland, const std::vector<double>& costs,
        PlanPoint anchor, PlanPoint enemy, double body, PlanPoint& goal,
        PlanPoint outpost={NAN,NAN}, bool outpost_known=false, double outpost_radius=.6,
        const std::vector<PlanPoint>& rejected={}) {
    if(costs.size()!=grid.cells.size()) return false;
    const double range=pointDistance(anchor,enemy);
    if(!std::isfinite(range) || range<.1) return false;
    PlanPoint direction{(enemy.x-anchor.x)/range,(enemy.y-anchor.y)/range};
    const bool sidestep=outpost_known &&
        !sentrySightClear(raw,anchor,enemy,body,outpost,true,outpost_radius);
    // The tower and fourth-vehicle pose define the side of the highland.
    // A patrolling sentry must not rotate this station-selection frame and
    // turn the intended left step into a long move along the platform edge.
    if(outpost_known && pointDistance(anchor,outpost)>.1) {
        double tower_range=pointDistance(anchor,outpost);
        direction={(outpost.x-anchor.x)/tower_range,(outpost.y-anchor.y)/tower_range};
    }
    double best=INFINITY;
    // If already inside the tower's clearance, allow a sideways/back escape
    // rather than requiring forward motion through the tower itself.
    const double minimum_forward=outpost_known && pointDistance(anchor,outpost)<outpost_radius+.4
        ? -.15 : (outpost_known?.3:.1);
    for(auto cell:highland) {
        if(cell<0 || size_t(cell)>=costs.size() || grid.cells[cell]!=0 || !std::isfinite(costs[cell])) continue;
        auto p=grid.centre(static_cast<int>(cell));
        bool refused=false;
        for(auto q:rejected)if(pointDistance(p,q)<.55){refused=true;break;}
        if(refused)continue;
        double dx=p.x-anchor.x,dy=p.y-anchor.y,advance=dx*direction.x+dy*direction.y;
        double left=direction.x*dy-direction.y*dx,lateral=std::abs(left);
        if((sidestep && left<.2) || (outpost_known && left<-.05) || advance<minimum_forward || advance>1.35 || pointDistance(p,anchor)>1.65 || lateral>1.2 ||
           pointDistance(p,enemy)<2.2 || !sentryLocalContains(p,anchor,outpost,outpost_known) ||
           !highlandArrivalSafe(grid,highland,p,.2) || !segmentWithReserve(grid,anchor,p,.03) ||
           !sentrySightClear(raw,p,enemy,body,outpost,outpost_known,outpost_radius)) continue;
        // Prefer forward progress from the measured four-vehicle station.
        // Only actual tower occlusion requires a substantial left sidestep.
        double score=2*std::abs(advance-(sidestep?.65:1.2))+
            (sidestep?1.5*std::abs(left-.85):2*lateral)+.0001*costs[cell];
        if(score<best) {best=score;goal=p;}
    }
    if(std::isfinite(best)) return true;
    int cell=grid.cellAt(anchor);
    bool refused=false;for(auto q:rejected)if(pointDistance(anchor,q)<.55){refused=true;break;}
    if(!refused && cell>=0 && grid.cells[cell]==0 && std::isfinite(costs[cell]) && onCombatHighland(grid,highland,anchor) &&
       sentrySightClear(raw,anchor,enemy,body,outpost,outpost_known,outpost_radius)) {
        goal=anchor;return true;
    }
    return false;
}

// Prefer the nearest feasible highland firing band, then choose one whole
// route to that band. Never trade terrain/line of sight for a shorter range.
inline bool chooseCloseCombatPosition(const DijkstraGrid& grid, const DijkstraGrid& raw,
        const std::vector<double>& costs, PlanPoint enemy, double body, PlanPoint& goal,
        PlanPoint excluded={INFINITY,INFINITY}, PlanPoint anchor={INFINITY,INFINITY}, double radius=0.) {
    if(costs.size()!=grid.cells.size()) return false;
    std::vector<int> candidates; double nearest=INFINITY;
    for(size_t i=0;i<costs.size();++i) {
        if(grid.cells[i]!=0 || !std::isfinite(costs[i])) continue;
        auto p=grid.centre(static_cast<int>(i));double distance=pointDistance(p,enemy);
        if(distance<2.2 || pointDistance(p,excluded)<.6 ||
           (radius>0 && !anchorContains(p,anchor,radius,.4)) ||
           !segmentWithReserve(grid,p,p,.03) || !attackRayFree(raw,p,enemy,body)) continue;
        nearest=std::min(nearest,distance);candidates.push_back(i);
    }
    double best=INFINITY;
    for(auto i:candidates) {
        auto p=grid.centre(i);double distance=pointDistance(p,enemy);
        if(distance>std::max(2.65,nearest+.25)) continue;
        double score=costs[i]+3*std::abs(distance-2.3);
        if(score<best) {best=score;goal=p;}
    }
    return std::isfinite(best);
}

inline bool chooseAttackPosition(const DijkstraGrid& g, const DijkstraGrid& raw, const std::vector<double>& costs,
                                 PlanPoint enemy, double body_radius, PlanPoint& goal,
                                 PlanPoint excluded = {INFINITY, INFINITY}, double maximum_range = INFINITY) {
    double best = INFINITY;
    for (double radius : {3., 3.6, 2.4, 4.2})
        for (int sector = 0; sector < 32; ++sector) {
            if (radius > maximum_range)
                continue;
            double angle = sector * 2 * 3.141592653589793 / 32;
            PlanPoint p{enemy.x + radius * std::cos(angle), enemy.y + radius * std::sin(angle)};
            int i = g.cellAt(p);

            if (i < 0 || !std::isfinite(costs[i]))
                continue;
            p = g.centre(i);
            if (pointDistance(p, excluded) < .6)
                continue;

            if (!attackRayFree(raw, p, enemy, body_radius))
                continue;
            double score = costs[i] + std::abs(radius - 3.) * .3;
            if (score < best) {
                best = score;
                goal = p;
            }
        }

    return std::isfinite(best);
}

inline bool chooseAttackPosition(const DijkstraGrid& g, const DijkstraGrid& raw, PlanPoint start, PlanPoint enemy,
                                 double body_radius, const std::vector<double>& risk, PlanPoint& goal) {
    return chooseAttackPosition(g, raw, routeCosts(g, start, risk), enemy, body_radius, goal);
}

struct TacticalGains {
    double base = 100, unlock = 30, robot = 24, ammo = 45, health = 45, travel = 1, damage = .002, switch_margin = 6;
    bool valid() const {
        for (double v : {base, unlock, robot, ammo, health, travel, damage, switch_margin})
            if (!std::isfinite(v) || v < 0 || v > 1000)
                return false;
        return base > 0 && travel > 0;
    }
};

struct TacticalCandidate {
    std::string stage;
    int target_id = 0;
    PlanPoint objective, goal;
    double gain = 0, route_cost = INFINITY, score = -INFINITY;
};

// Benefits rank destinations. All Dijkstra edge costs remain nonnegative;
// subtracting rewards from edges would create invalid negative-cost cycles.
inline TacticalCandidate scoreTacticalCandidate(std::string stage, int id, PlanPoint objective, PlanPoint goal,
                                                double gain, const DijkstraGrid& g, const std::vector<double>& costs,
                                                const TacticalGains& weights, int health = 0) {
    TacticalCandidate c{stage, id, objective, goal, gain};
    int cell = g.cellAt(goal);
    if (cell < 0 || costs.size() != g.cells.size() || !std::isfinite(costs[cell]) || !std::isfinite(gain))
        return c;
    c.route_cost = costs[cell];
    c.score = gain - weights.travel * c.route_cost - weights.damage * std::max(0, health);

    return c;
}

inline bool betterTacticalCandidate(const TacticalCandidate& candidate, const TacticalCandidate& current,
                                    bool keep_current, const TacticalGains& weights) {
    return std::isfinite(candidate.score) &&
           (!std::isfinite(current.score) ||
            candidate.score > current.score + (keep_current ? weights.switch_margin : 0));
}
} // namespace campus
