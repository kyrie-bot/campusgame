#pragma once
#include <cstdint>
#include <cmath>

namespace campus {
class RuneSingleShot {
public:
    void reset() { *this=RuneSingleShot(); }
    bool consume(uint64_t id, double now = 0.) {
        if (id == 0 || id <= last_ || !std::isfinite(now) || now < next_ || pending_)
            return false;

        last_ = id;
        until_ = -INFINITY;
        pending_ = true;
        emitted_ = false;
        return true;
    }//接受一次新的射击请求并避免重复使用相同射击编号

    uint64_t last() const {
        return last_;
    }//最近一次射击编号

    bool active(double now) const {
        return std::isfinite(now) && (pending_ || now < until_);
    }//判断当前射击许可时间窗口是否有效

    bool emit(double now) {
        if (!std::isfinite(now) || !pending_ || emitted_) return false;

        until_=now+.020;
        next_=now+.10;
        pending_=false;
        emitted_=true;
        return true;
    }//发送射击指令的时机

    void cancel() {
        until_ = -INFINITY;
        pending_ = false;
    }//取消射击

private:
    uint64_t last_ = 0;
    double until_ = -INFINITY;
    double next_ = -INFINITY;
    bool emitted_ = false;
    bool pending_ = false;
};
} // namespace campus
