#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <sstream>

namespace campus {
inline constexpr const char* kRobotMissionOrder = "ROBOT_OUTPOST_BASE";
inline constexpr const char* kBuffMissionOrder = "BUFF_OPENING_ROBOTS_BUFF_ON_EXPIRY_OUTPOST_BASE";

// The status protocol has two supported mission schedules. Parse the field,
// not a substring of the old schedule: rune missions still hand off to armor.
inline std::string trackerMissionStage(const std::string& status) {
    std::istringstream input(status);
    std::string stage, token;
    if (!(input >> stage)) return "WAIT_MISSION";
    while (input >> token) {
        if (token.rfind("mission_order=", 0) != 0) continue;
        const auto order = token.substr(14);
        return order == kRobotMissionOrder || order == kBuffMissionOrder ? stage : "WAIT_MISSION";
    }
    return "WAIT_MISSION";
}

inline std::string armorSelectionStage(const std::string& stage, bool refill_threats = false) {
    if (stage == "WAIT_CLEAR_ROBOTS_POSITION" || (stage == "REFILL" && refill_threats))
        return "CLEAR_ROBOTS";
    return stage;
}

inline bool missionArmorAllowed(const std::string& stage, int number) {
    if (stage == "OPENING_RAMP")
        return number >= 2 && number <= 4;
    if (stage == "FIELD_ROBOTS")
        return number >= 1 && number <= 4;
    if (stage == "CLEAR_ROBOTS")
        return (number >= 1 && number <= 5) || number == 7 || number == 0;

    if (stage == "OUTPOST")
        return number == 6;
    if (stage == "BASE")
        return number == 8;
    return false;
}

// Visual sentry armor is 7, base armor is 8; public unit IDs are 5 and 7.
inline int armorHealthIndex(int number) {
    if (number >= 1 && number <= 6)
        return number - 1;
    if (number == 7)
        return 4;
    if (number == 8)
        return 6;
    return -1;
}

inline bool robotCleanupComplete(const std::array<int32_t, 7>& healths, bool ignore_one = false) {
    for (int i = ignore_one ? 1 : 0; i < 5; ++i)
        if (healths[i] > 0)
            return false;
    return true;
}

// The match schedule is 3 -> 2 -> 4; health feedback, not visual loss,
// advances the order. Refill/evade policy and structure stages stay separate.
inline int nextFieldRobot(const std::array<int32_t, 7>& healths) {
    for (int number : {3, 2, 4})
        if (healths[number - 1] > 0) return number;
    return 0;
}

inline bool fieldRobotOrderAllowed(const std::string& stage, int number,
                                   const std::array<int32_t, 7>& healths, bool enabled) {
    if (!enabled || (stage != "OPENING_RAMP" && stage != "FIELD_ROBOTS" && stage != "CLEAR_ROBOTS"))
        return true;
    const int next = nextFieldRobot(healths);
    return !next || number == next;
}

inline bool missionArmorAlive(const std::string& stage, int number, const std::array<int32_t, 7>& healths,
                              bool ignore_one = false, bool field_order = false) {
    if (!fieldRobotOrderAllowed(stage, number, healths, field_order)) return false;
    if (ignore_one && number == 1)
        return false;
    if ((stage == "OUTPOST" || stage == "BASE") && !robotCleanupComplete(healths, ignore_one))
        return false;
    if (stage == "BASE" && healths[5] > 0)
        return false;
    int i = armorHealthIndex(number);
    return missionArmorAllowed(stage, number) && (i < 0 || healths[i] > 0);
}
} // namespace campus