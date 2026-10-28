#pragma once

#include <tdt_interface/msg/auto_aim_state.hpp>
#include <sstream>
#include <string>

namespace campus {
inline std::string navigationStateSession(const tdt_interface::msg::AutoAimState& state) {
    std::ostringstream session;
    session << state.header.frame_id;
#ifdef CAMPUS_STATE_HAS_SESSION_ID
    session << '/' << state.session_id;
#endif
    return session.str();
}
} // namespace campus