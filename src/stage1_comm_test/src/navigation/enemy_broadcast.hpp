#pragma once

#include <cmath>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace campus {
struct BroadcastEnemy {
    int id = 0;
    double unity_x_m = 0;
    double unity_z_m = 0;
    double health = 1;
    double attack_range_raw = 0;
};

struct BroadcastFields {
    std::string array_key;
    std::string id_key = "number";
    std::string x_key = "x";
    std::string z_key = "z";
    std::string health_key;
    std::string attack_range_key = "attack_range";
    double scene_units_per_m = 3;
};

struct EnemyPacket {
    uint64_t stamp_ns = 0;
    int player_id = 0;
    std::string session_id;
    std::string frame_id;
    std::vector<BroadcastEnemy> enemies;
};

inline std::vector<BroadcastEnemy> parseEnemyBroadcast(const std::string& text, const BroadcastFields& fields) {
    if (text.size() > 65536 || !std::isfinite(fields.scene_units_per_m) || fields.scene_units_per_m <= 0) {
        throw std::invalid_argument("Invalid enemy packet size or unit scale");
    }

    nlohmann::json root = nlohmann::json::parse(text);
    nlohmann::json array = root;
    if (!fields.array_key.empty()) {
        array = root.at(fields.array_key);
    }

    if (!array.is_array() || array.size() > 6) {
        throw std::invalid_argument("Expected enemy array with at most six units (IDs 2..7)");
    }

    std::set<int> ids;
    std::vector<BroadcastEnemy> enemies;
    for (std::size_t i = 0; i < array.size(); ++i) {
        const nlohmann::json& entry = array.at(i);

        if (!entry.at(fields.id_key).is_number_integer() || !entry.at(fields.x_key).is_number() ||
            !entry.at(fields.z_key).is_number()) {
            throw std::invalid_argument("Enemy ID/X/Z must be numeric");
        }

        BroadcastEnemy enemy;
        if (entry.at(fields.id_key) < 2 || entry.at(fields.id_key) > 7) {
            throw std::invalid_argument("Enemy number must be 2..7");
        }

        enemy.id = entry.at(fields.id_key).get<int>();
        enemy.unity_x_m = entry.at(fields.x_key).get<double>() / fields.scene_units_per_m;
        enemy.unity_z_m = entry.at(fields.z_key).get<double>() / fields.scene_units_per_m;

        if (enemy.id < 2 || enemy.id > 7 || !ids.insert(enemy.id).second || !std::isfinite(enemy.unity_x_m) ||
            !std::isfinite(enemy.unity_z_m) || std::abs(enemy.unity_x_m) > 10000 || std::abs(enemy.unity_z_m) > 10000) {
            throw std::invalid_argument("Invalid, duplicate or nonfinite enemy record");
        }

        if (!fields.health_key.empty()) {
            if (!entry.at(fields.health_key).is_number()) {
                throw std::invalid_argument("Enemy health must be numeric");
            }

            enemy.health = entry.at(fields.health_key).get<double>();
            if (!std::isfinite(enemy.health) || enemy.health < 0) {
                throw std::invalid_argument("Invalid enemy health");
            }

            if (enemy.health == 0 && enemy.id <= 5) {
                continue;
            }
        }

        if (!fields.attack_range_key.empty()) {
            if (!entry.at(fields.attack_range_key).is_number()) {
                throw std::invalid_argument("Enemy attack range must be numeric");
            }

            enemy.attack_range_raw = entry.at(fields.attack_range_key).get<double>();
            if (!std::isfinite(enemy.attack_range_raw) || enemy.attack_range_raw < 0) {
                throw std::invalid_argument("Invalid enemy attack range");
            }
        }

        enemies.push_back(enemy);
    }

    return enemies;
}

inline EnemyPacket parseEnemyPacket(const std::string& text, const BroadcastFields& fields) {
    if (text.size() > 65536) {
        throw std::invalid_argument("Enemy packet too large");
    }

    nlohmann::json root = nlohmann::json::parse(text);
    if (!root.is_object() || root.at("schema_version") != 1 || !root.at("stamp_ns").is_number_unsigned() ||
        !root.at("player_id").is_number_integer() || !root.at("session_id").is_string() ||
        !root.at("frame_id").is_string()) {
        throw std::invalid_argument("Invalid enemy packet header/schema");
    }

    if (root.at("player_id") < 1 || root.at("player_id") > 2) {
        throw std::invalid_argument("Invalid player number");
    }

    EnemyPacket packet;
    packet.stamp_ns = root.at("stamp_ns").get<uint64_t>();
    packet.player_id = root.at("player_id").get<int>();
    packet.session_id = root.at("session_id").get<std::string>();
    packet.frame_id = root.at("frame_id").get<std::string>();

    if (packet.stamp_ns == 0 || packet.stamp_ns > static_cast<uint64_t>(INT64_MAX) || packet.session_id.empty() ||
        packet.session_id.size() > 128 || (packet.player_id != 1 && packet.player_id != 2)) {
        throw std::invalid_argument("Invalid enemy time/session/player");
    }

    packet.enemies = parseEnemyBroadcast(text, fields);
    return packet;
}
} // namespace campus