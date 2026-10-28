#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace campus_buff {
struct RuneLeafObservation {
    double phase;
    bool pending;
};
// Keep physical leaf identities in the rotating coordinate system. Screen
// proximity to yesterday's target is not an identity when the next leaf lights.
class RuneLeafCycle {
public:
    void reset() {
        *this = RuneLeafCycle();
    }

    void predictRotation(double from,double to,double delta) {
        predicted_stamp_=to;
        predicted_delta_=std::abs(from-stamp_)<.001&&std::isfinite(delta)?delta:NAN;
    }

    // Called only for a fresh image with an actually measured/held R and no
    // visible leaf contours, never for missing transport/corrupt frames.
    void observeDark(double stamp) {
        if(round_shot_ && std::isfinite(stamp) && stamp>stamp_ &&
           stamp>last_impact_-.15 && stamp-last_impact_<1.5) dark_stamp_=stamp;
    }

    void observe(const std::vector<RuneLeafObservation>& leaves, double stamp, bool big = false, bool full_view = false) {
        if (leaves.empty() || !std::isfinite(stamp) || stamp <= stamp_)
            return;
        const double step = 2 * std::acos(-1.) / 5;
        double next = leaves.front().phase;
        if (std::isfinite(stamp_)) {
            double dt = stamp - stamp_;
            double expected=basis_+(std::abs(stamp-predicted_stamp_)<.001&&std::isfinite(predicted_delta_)?
                                    predicted_delta_:velocity_*dt);
            next = expected + std::remainder(next - expected, step);
            if (dt < .3)
                velocity_ = std::clamp((next - basis_) / dt, -2.3, 2.3);
        }
        basis_ = next;
        stamp_ = stamp;
        std::array<bool, 5> pending{}, completed{};

        for (auto leaf : leaves)
            (leaf.pending ? pending : completed)[slot(leaf.phase)] = true;
        unsigned pair = 0, count = 0;

        for (int i = 0; i < 5; ++i)
            if (pending[i]) {
                pair |= 1u << i;
                ++count;
            }

        bool new_pair = false;
        if (big && count == 2) {
            if (!pair_)
                pair_ = pair;
            else if ((pair != pair_ || (dark_stamp_ > last_impact_-.15 && stamp-dark_stamp_ < .35)) &&
                     round_shot_ && stamp > last_impact_) {
                pair_frames_ = pair == pair_candidate_ ? pair_frames_ + 1 : 1;
                pair_candidate_ = pair;
                new_pair = pair_frames_ >= 2;
            } else
                pair_frames_ = 0;
        } else
            pair_frames_ = 0;
        // A completed leaf lighting again is direct evidence of a new game
        // sequence. Require two images; one contour glitch must not unlock it.
        bool restarted = false;

        for (int i = 0; i < 5; ++i) {
            relit_[i] = stamp > completion_after_[i] && pending[i] && completed_[i] ? relit_[i] + 1 : 0;
            restarted = restarted || relit_[i] >= 2;
        }
        // A duplicate hit can clear the completed rings before their normal
        // completion->relight transition is captured. Require a full-board
        // view, a spent/consumed pending leaf, and sustained visual rollback.
        // Partial views and an unchanged old cross alone never rearm a shot.
        bool owned_pending=false,known_completed=false,visible_completed=false;
        for(int i=0;i<5;++i) {
            owned_pending=owned_pending || (pending[i] && spent_[i] && consumed_[i]);
            known_completed=known_completed || completed_[i];
            visible_completed=visible_completed || completed[i];
        }
        bool rollback=!big && full_view && count==1 && owned_pending && !visible_completed &&
                      (known_completed || extra_consumed_) && stamp>last_impact_+.06;
        if(rollback) {
            if(stamp-rollback_last_>.15) rollback_frames_=0;
            rollback_last_=stamp;
            if(!rollback_frames_) rollback_since_=stamp;
            ++rollback_frames_;
        } else rollback_frames_=0;
        bool board_restart=rollback_frames_>=3 && stamp-rollback_since_>=.08;
        // Big rune rounds may reuse the just-hit second leaf immediately, so
        // its completed ring need not have appeared in an intervening image.
        if ((!big && restarted) || board_restart || new_pair) {
            ++generation_;
            spent_.fill(false);
            consumed_.fill(false);
            completed_.fill(false);
            relit_.fill(0);
            ordinary_.fill(0);
            absent_.fill(0);
            round_shot_ = false;
            round_hit_ = false;
            dark_stamp_ = -INFINITY;
            pair_frames_ = 0;
            rollback_frames_=0;
            extra_consumed_=false;
            if (count == 2)
                pair_ = pair;
        }

        for (int i = 0; i < 5; ++i) {
            // A briefly broken cross can look like an ordinary hole. It does
            // not prove completion of an unfired leaf. Nor can a pre-impact
            // delayed image complete a leaf and unlock its following cross.
            ordinary_[i] =
                spent_[i] && consumed_[i] && stamp > completion_after_[i] && completed[i] && !pending[i] ? ordinary_[i] + 1 : 0;
            // Big-hit animation need not expose an ordinary ring. Two fresh
            // views with only the OTHER cross remaining still establish the
            // first hit's timing; waiting for an ordinary contour can otherwise
            // allow a late second bullet into the following round.
            absent_[i] = big && count == 1 && spent_[i] && consumed_[i] &&
                         stamp > completion_after_[i] && !pending[i] ? absent_[i]+1 : 0;
            if (!round_hit_ && absent_[i] >= 2) {
                round_hit_ = true;
                first_impact_ = completion_after_[i];
            }
            if (ordinary_[i] >= 2) {
                completed_[i] = true;
                if (!round_hit_) {
                    round_hit_ = true;
                    first_impact_ = completion_after_[i];
                }
            }
            // A still-lit cross is NOT proof of a miss: delayed rendering and
            // hit effects can retain it. One consumed round per illumination.
            // Only an observed completed -> relit transition/new big round
            // can unlock it; never a timeout or consecutive pending frames.
        }
    }

    int slot(double phase) const {
        const double step = 2 * std::acos(-1.) / 5;
        int i = static_cast<int>(std::llround(std::remainder(phase - basis_, 2 * std::acos(-1.)) / step));
        return (i % 5 + 5) % 5;
    }

    bool available(int slot) const {
        return slot >= 0 && slot < 5 && !spent_[slot] && !completed_[slot];
    }

    uint64_t generation() const { return generation_; }
    bool needsRestartView() const { return extra_consumed_; }

    bool impactInRound(double impact_stamp, bool big) const {
        // The optional second big-rune hit must arrive within 1 s of the
        // first. Keep a 100 ms margin for command/impact uncertainty. Do not
        // send a late shot into an unlit leaf in the following round.
        return !big || !round_hit_ || impact_stamp < first_impact_ + .8;
    }

    // Count only unshot illuminations. An old cross can remain visible during
    // the hit animation and must neither steal tracking nor veto a new leaf.
    int selectPending(const std::vector<int>& slots, int current, bool big) const {
        std::array<bool, 5> eligible{};
        int count = 0, first = -1;
        for (int slot : slots) {
            if (!available(slot) || eligible[slot]) continue;
            eligible[slot] = true;
            ++count;
            if (first < 0) first = slot;
        }
        if (count > (big ? 2 : 1)) return -1;
        return current >= 0 && current < 5 && eligible[current] ? current : first;
    }

    bool completed(int slot) const {
        return slot >= 0 && slot < 5 && completed_[slot];
    }

    void sent(int slot, double completion_after = -INFINITY) {
        if (slot >= 0 && slot < 5) {
            spent_[slot] = true;
            consumed_[slot] = false;
            completion_after_[slot] = completion_after;
            ordinary_[slot] = 0;
            round_shot_ = true;
            last_impact_ = completion_after;
        }
    }

    void notFired(int slot) {
        if (slot >= 0 && slot < 5) {
            spent_[slot] = false;
            consumed_[slot] = false;
        }
    }

    void confirmConsumed(int slot) {
        if (slot >= 0 && slot < 5 && spent_[slot]) consumed_[slot] = true;
    }

    void extraConsumed(int slot) {
        if(slot>=0 && slot<5 && spent_[slot] && consumed_[slot]) extra_consumed_=true;
    }

private:
    double basis_ = 0, stamp_ = -INFINITY, velocity_ = 0;
    double predicted_stamp_=NAN,predicted_delta_=NAN;
    std::array<bool, 5> spent_{}, completed_{}, consumed_{};
    std::array<int, 5> relit_{};
    std::array<int, 5> ordinary_{};
    std::array<int, 5> absent_{};
    std::array<double, 5> completion_after_{};
    unsigned pair_ = 0, pair_candidate_ = 0;
    int pair_frames_ = 0;
    bool round_shot_ = false, round_hit_ = false;
    uint64_t generation_ = 0;
    double first_impact_ = -INFINITY;
    double last_impact_ = -INFINITY, dark_stamp_ = -INFINITY;
    bool extra_consumed_=false;
    int rollback_frames_=0;
    double rollback_since_=-INFINITY,rollback_last_=-INFINITY;
};
} // namespace campus_buff
