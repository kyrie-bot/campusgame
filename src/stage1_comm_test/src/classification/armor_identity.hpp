#pragma once

namespace campus {
inline bool knownArmorIdentity(int number) {
    return number >= 1 && number <= 8;
}

inline bool validArmorIdentity(int number) {
    return number == 0 || knownArmorIdentity(number);
}

inline bool symbolArmorIdentity(int number) {
    return number >= 6 && number <= 8;
}

inline const char* armorIdentityName(int number) {
    if (number >= 1 && number <= 5) {
        return "ROBOT";
    }

    if (number == 6) {
        return "OUTPOST";
    }

    if (number == 7) {
        return "SENTRY";
    }

    if (number == 8) {
        return "BASE";
    }

    return "UNKNOWN";
}
} // namespace campus