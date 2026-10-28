#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include "buff_detection/final_rune_accuracy.hpp"

namespace campus_buff {
// One bounded opportunity per visit. The game, not detection, confirms success.
// Accepted opportunities alone advance small/small/big/big/big. An ambiguous
// unacknowledged request stops automatic retries with a NEW id.
class RuneSession {
public:
    void setLimit(unsigned limit) { limit_=std::clamp(limit,1u,5u); }
    void reset() {
        auto limit=limit_; *this = RuneSession(); limit_=limit;
    }

    void begin(uint64_t goal, double time) {
        goal_ = goal;
        began_ = time;
        request_ = 0;
        accepted_ = false;
        finished_ = false;
        activated_ = false;
        reason_ = "APPROACH";
        arrived_ = false;
    }

    void abort(const std::string& reason) {
        if (!finished_ && request_ && !accepted_ && reason != "REQUEST_REJECTED" && reason != "BUFF_ALREADY_ACTIVE")
            uncertain_ = true;
        finished_ = true;
        reason_ = reason;
    }

    void arrived(double time) {
        if (!arrived_) {
            began_ = time;
            arrived_ = true;
        }
    }

    void tick(double time) {
        if (finished_)
            return;
        if (arrived_ && time - began_ >= 45)
            abort("VISIT_TIMEOUT");
        if (request_ && !accepted_ && time - sent_ > 2.) {
            uncertain_ = true;
            abort("REQUEST_ACK_TIMEOUT");
        }

        if (accepted_ && time - sent_ >= 20.)
            abort("ACTIVATION_TIMEOUT");
    }

    bool canRequest() const {
        return !finished_ && !request_ && count_ < limit_ && !uncertain_;
    }

    void requested(uint64_t id, double time) {
        arrived(time);
        request_ = id;
        sent_ = time;
        reason_ = "WAIT_REQUEST_ACK";
    }

    void result(uint64_t id, bool accepted, bool activated, double time) {
        if (finished_ || !request_ || id != request_ || time < sent_ || time - sent_ > 21)
            return;
        if (accepted && !accepted_) {
            accepted_ = true;
            ++count_;
            reason_ = "TRACK_PENDING";
        }

        if (accepted_ && activated) {
            successes_.observe(request_,count_,true);
            activated_ = true;
            abort("ACTIVATED");
        } else if (!accepted)
            abort(activated ? "BUFF_ALREADY_ACTIVE" : "REQUEST_REJECTED");
    }

    bool accepted() const {
        return accepted_ && !finished_;
    }

    bool big() const {
        return accepted_ && count_ >= 3;
    }

    bool finished() const {
        return finished_;
    }

    bool activated() const {
        return activated_;
    }

    bool uncertain() const {
        return uncertain_;
    }

    unsigned count() const {
        return count_;
    }
    unsigned successfulBigActivations() const {return successes_.count();}

    uint64_t request() const {
        return request_;
    }

    uint64_t goal() const {
        return goal_;
    }

    double sent() const {
        return sent_;
    }

    std::string reason() const {
        return reason_;
    }

private:
    uint64_t goal_ = 0, request_ = 0;
    unsigned count_ = 0, limit_ = 5;
    BigRuneSuccesses successes_;
    double began_ = 0, sent_ = 0;
    bool arrived_ = false, accepted_ = false, finished_ = true, activated_ = false, uncertain_ = false;
    std::string reason_ = "IDLE";
};
} // namespace campus_buff
