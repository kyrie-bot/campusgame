#include "navigation/dijkstra.hpp"
#include "navigation/clearance_cost.hpp"
#include "navigation/assault_mission.hpp"
#include "navigation/mission_order.hpp"
#include "navigation/rune_mission.hpp"

#include <campus_aim_interfaces/msg/buff_status.hpp>

#include "navigation/enemy_broadcast.hpp"
#include "navigation/path_stability.hpp"

#include <chrono>
#include <array>
#include <functional>
#include <memory>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <campus_aim_interfaces/msg/navigation_plan.hpp>
#include <campus_aim_interfaces/msg/track_frame.hpp>
#include <tdt_interface/msg/auto_aim_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class DijkstraPlanner : public rclcpp::Node {
  public:
    DijkstraPlanner() : Node("dijkstra_planner") {
        frame_ = declare_parameter<std::string>("map_frame", "arena_reference");
        demo_ = declare_parameter<bool>("demo_mode", false);
        display_only_ = declare_parameter<bool>("display_only", true);
        allow_alignment_ = declare_parameter<bool>("allow_live_alignment", false);
        risk_mode_ = declare_parameter<bool>("uncertainty_as_cost", false);
        risk_weight_ = declare_parameter<double>("uncertainty_cost_weight", 8.0);
        wall_distance_ = declare_parameter<double>("wall_preference_distance_m", 0.6);
        wall_weight_ = declare_parameter<double>("wall_preference_weight", 6.0);

        player_ = declare_parameter<int>("player_id", 1);

        autonomous_ = declare_parameter<bool>("mission.enabled", false);
        buff_enabled_ = declare_parameter<bool>("mission.buff_enabled", false);
        buff_limit_=declare_parameter<int>("mission.buff_max_opportunities",5);
        if(buff_limit_<1 || buff_limit_>5) throw std::invalid_argument("mission.buff_max_opportunities must be 1..5");
        tactical_ = declare_parameter<bool>("mission.tactical_selection", true);
        strict_order_ = declare_parameter<bool>("mission.strict_order", false);
        field_order_ = declare_parameter<bool>("mission.field_robot_order", false);
        four_station_enabled_ = declare_parameter<bool>("mission.four_station_enabled", false);
        auto station = declare_parameter<std::vector<double>>("mission.four_station_xy_m", {11.15, 1.45});
        if (station.size()!=2 || !std::isfinite(station[0]) || !std::isfinite(station[1]) ||
            station[0]<=0 || station[0]>=28 || station[1]<=0 || station[1]>=15)
            throw std::invalid_argument("Invalid mission.four_station_xy_m");
        four_station_ = player_==1 ? campus::PlanPoint{station[0],station[1]}
                                 : campus::PlanPoint{28.-station[0],15.-station[1]};
        ignore_one_ = declare_parameter<bool>("mission.ignore_enemy_one", false);
        opening_enabled_ = declare_parameter<bool>("mission.opening_ramp_enabled", false);
        buff_low_ledges_ =
            declare_parameter<std::vector<int64_t>>("mission.buff_low_ledge_cells", std::vector<int64_t>{});
        buff_ground_excluded_ =
            declare_parameter<std::vector<int64_t>>("mission.buff_ground_excluded_cells", std::vector<int64_t>{});
        opening_cells_ = declare_parameter<std::vector<int64_t>>("mission.opening_ramp_cells", std::vector<int64_t>{});
        rune_transit_cells_ = declare_parameter<std::vector<int64_t>>("mission.rune_transit_ramp_cells", std::vector<int64_t>{});

        if (opening_enabled_ && opening_cells_.empty())
            throw std::invalid_argument("Missing opening ramp cells");

        sentry_last_ = declare_parameter<bool>("mission.sentry_last", false);
        anchor_enabled_ = declare_parameter<bool>("mission.sentry_anchor_enabled", false);
        highland_cells_=declare_parameter<std::vector<int64_t>>("mission.robot_highland_cells",std::vector<int64_t>{});
        std::sort(highland_cells_.begin(),highland_cells_.end());
        anchor_radius_ = declare_parameter<double>("mission.sentry_anchor_radius_m", 2.);

        if (!std::isfinite(anchor_radius_) || anchor_radius_ < .8 || anchor_radius_ > 3.)
            throw std::invalid_argument("Invalid sentry anchor radius");

        outpost_band_.minimum = declare_parameter<double>("mission.outpost_min_range_m", 2.8);
        outpost_band_.preferred = declare_parameter<double>("mission.outpost_preferred_range_m", 3.5);
        outpost_band_.maximum = declare_parameter<double>("mission.outpost_max_range_m", 4.2);

        if (!outpost_band_.valid() || outpost_band_.minimum < 2.5)
            throw std::invalid_argument("Invalid outpost standoff band");

        base_band_.minimum = declare_parameter<double>("mission.base_min_range_m", 1.8);
        base_band_.preferred = declare_parameter<double>("mission.base_preferred_range_m", 2.5);
        base_band_.maximum = declare_parameter<double>("mission.base_max_range_m", 3.2);

        if (!base_band_.valid() || base_band_.maximum > 4.)
            throw std::invalid_argument("Invalid close-range base band");

        gains_.base = declare_parameter<double>("mission.gain.base", 100);
        gains_.unlock = declare_parameter<double>("mission.gain.unlock_base", 30);
        gains_.robot = declare_parameter<double>("mission.gain.robot", 24);
        gains_.ammo = declare_parameter<double>("mission.gain.ammo", 45);
        gains_.health = declare_parameter<double>("mission.gain.health", 45);
        gains_.travel = declare_parameter<double>("mission.cost.route", 1);
        gains_.damage = declare_parameter<double>("mission.cost.target_health", .002);
        gains_.switch_margin = declare_parameter<double>("mission.switch_margin", 6);

        if (!gains_.valid())
            throw std::invalid_argument("Invalid mission gains/costs");

        gain_points_ = declare_parameter<std::vector<double>>("mission.gain_points_m", std::vector<double>{});
        gain_values_ = declare_parameter<std::vector<double>>("mission.gain_values", std::vector<double>{});
        if (gain_points_.size() != gain_values_.size() * 2 || gain_values_.size() > 32 ||
            !std::all_of(gain_points_.begin(), gain_points_.end(), [](double v) { return std::isfinite(v); }) ||
            !std::all_of(gain_values_.begin(), gain_values_.end(),
                         [](double v) { return std::isfinite(v) && v > 0 && v <= 1000; }))
            throw std::invalid_argument("Gain points need finite x/y pairs and positive rewards");
        gain_used_until_.assign(gain_values_.size(), 0);

        // Approximate public tactical-map candidates, not measured trigger
        // boundaries. Feedback, rather than arrival, confirms replenishment.
        refill_points_ = declare_parameter<std::vector<double>>(
            "mission.refill_points_m",
            player_ == 1 ? std::vector<double>{26.4, 13.3, 26.4, 12.5, 21.42, 7.5, 24., 7.5, 25.62, 9.2, 25.62, 5.8}
                         : std::vector<double>{1.6, 1.7, 1.6, 2.5, 6.58, 7.5, 4., 7.5, 2.38, 5.8, 2.38, 9.2});
        if (refill_points_.empty() || refill_points_.size() % 2 || refill_points_.size() > 64 ||
            !std::all_of(refill_points_.begin(), refill_points_.end(), [](double v) { return std::isfinite(v); }))
            throw std::invalid_argument("mission.refill_points_m must contain finite x,y pairs");

        projection_.matrix = declare_parameter<std::vector<double>>("unity_to_reference", std::vector<double>());
        initial_projection_ = projection_.matrix;

        fields_.scene_units_per_m = declare_parameter<double>("scene_units_per_m", 3.0);
        fields_.array_key = declare_parameter<std::string>("json.array_key", "enemies");
        fields_.id_key = declare_parameter<std::string>("json.id_key", "number");
        fields_.x_key = declare_parameter<std::string>("json.x_key", "x");
        fields_.z_key = declare_parameter<std::string>("json.z_key", "z");
        fields_.health_key = declare_parameter<std::string>("json.health_key", "health");
        fields_.attack_range_key = declare_parameter<std::string>("json.attack_range_key", "attack_range");
        enemy_frame_ = declare_parameter<std::string>("json.frame_id", "unity_world");

        enemy_timeout_ = declare_parameter<double>("enemy_timeout_s", 2.5);
        state_timeout_ = declare_parameter<double>("state_timeout_s", 0.5);
        replan_hz_ = declare_parameter<double>("replan_hz", 5.0);
        enemy_radius_ = declare_parameter<double>("enemy_radius_m", 0.35);
        own_radius_ = declare_parameter<double>("own_radius_m", 0.30);
        margin_ = declare_parameter<double>("dynamic_margin_m", 0.15);
        uncertainty_ = declare_parameter<double>("broadcast_uncertainty_m", 0.20);
        speed_bound_ = declare_parameter<double>("enemy_speed_bound_mps", 3.0);

        forecast_time_ = declare_parameter<double>("enemy_forecast_s", 0.0);
        outpost_radius_ = declare_parameter<double>("outpost_radius_m", 0.6);
        base_radius_ = declare_parameter<double>("base_radius_m", 0.9);
        use_healths_ = declare_parameter<bool>("use_enemy_healths", true);
        bool supported_frame = frame_ == "arena_reference" || (frame_ == "arena_campus" && display_only_);

        if (!supported_frame || !projection_.valid() || (player_ != 1 && player_ != 2) || fields_.array_key.empty() ||
            fields_.id_key.empty() || fields_.x_key.empty() || fields_.z_key.empty()) {
            throw std::invalid_argument("Invalid planner frame, projection or broadcast fields");
        }

        if (!std::isfinite(wall_distance_) || wall_distance_ <= 0 || wall_distance_ > 3 ||
            !std::isfinite(wall_weight_) || wall_weight_ < 0 || wall_weight_ > 100 || !std::isfinite(risk_weight_) ||
            risk_weight_ < 0 || risk_weight_ > 100 || (allow_alignment_ && frame_ != "arena_reference")) {
            throw std::invalid_argument("Invalid reference alignment or risk cost settings");
        }

        double values[] = {fields_.scene_units_per_m,
                           enemy_timeout_,
                           state_timeout_,
                           replan_hz_,
                           enemy_radius_,
                           own_radius_,
                           margin_,
                           uncertainty_,
                           speed_bound_,
                           outpost_radius_,
                           base_radius_};
        for (double value : values) {
            if (!std::isfinite(value) || value <= 0) {
                throw std::invalid_argument("Planner distances/rates/timeouts must be positive");
            }
        }

        if (enemy_timeout_ > 10 || state_timeout_ > 2 || replan_hz_ > 30 || speed_bound_ > 20 || enemy_radius_ > 10 ||
            own_radius_ > 10 || outpost_radius_ > 10 || base_radius_ > 10 || margin_ > 5 || uncertainty_ > 5) {
            throw std::invalid_argument("Planner timeout/rate/speed exceeds supported limits");
        }

        // Full stopping-time forecast at the supported 3 m/s cruise speed
        // exceeds 5 seconds. Preserve that forecast instead of clipping it.
        if (!std::isfinite(forecast_time_) || forecast_time_ < 0 || forecast_time_ > 10) {
            throw std::invalid_argument("enemy_forecast_s must be finite and in [0, 10]; got " +
                                        std::to_string(forecast_time_));
        }

        rclcpp::QoS cached(1);
        cached.reliable().transient_local();
        rclcpp::QoS latest(1);
        latest.reliable();

        path_pub_ = create_publisher<nav_msgs::msg::Path>("/planning/path", cached);
        history_pub_ = create_publisher<nav_msgs::msg::Path>("/planning/task_path", cached);
        grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/planning/dynamic_grid", cached);
        enemy_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/planning/enemy_markers", cached);
        point_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/planning/points", cached);
        status_pub_ = create_publisher<std_msgs::msg::String>("/planning/status", cached);
        mission_pub_ = create_publisher<std_msgs::msg::String>("/mission/status", cached);

        buff_status_sub_ = create_subscription<campus_aim_interfaces::msg::BuffStatus>(
            "/buff/status", latest, [this](campus_aim_interfaces::msg::BuffStatus::ConstSharedPtr m) {
                double age = now().seconds() - stampSeconds(m->header.stamp);
                if (age < -.1 || age > .2 || m->header.frame_id != state_frame_ || m->state_session != state_session_ ||
                    m->frame_sequence > last_sequence_ || m->frame_sequence < buff_min_sequence_)
                    return;
                bool expired = buff_effect_active_ && !m->buff_active;
                buff_effect_active_ = m->buff_active;
                buff_count_ = m->opportunities;

                if (expired && buff_count_ < unsigned(buff_limit_) && !buff_exhausted_)
                    buff_done_ = false;

                if (m->request_uncertain)
                    buff_exhausted_ = true;
                if (!buff_started_ && campus::runeRequestStartsVisit(
                        mission_stage_ == "BUFF" && m->goal_id == goal_id_, m->request_id, m->finished, buff_done_)) {
                    buff_started_ = true;
                    buff_since_ = SteadyClock::now();
                }

                if (campus::runeVisitInterrupted(m->reason))
                    return;
                if (!buff_done_ && mission_stage_ == "BUFF" && m->goal_id == goal_id_ && m->finished) {
                    buff_done_ = !campus::retryBigRuneAtStation(m->opportunities, m->buff_active,
                                                               m->request_uncertain, m->reason);
                    buff_started_ = false;
                    buff_finished_ = SteadyClock::now();
                    have_goal_ = false;

                    if (m->opportunities >= buff_limit_ || m->request_uncertain)
                        buff_exhausted_ = true;
                    if (m->reason == "REQUEST_REJECTED" || m->reason == "REQUEST_ACK_TIMEOUT") {
                        if (++buff_rejections_ >= 3)
                            buff_exhausted_ = true;
                    }

                    RCLCPP_INFO(get_logger(), "Rune visit finished: %s; opportunities=%u effect_active=%d",
                                m->reason.c_str(), m->opportunities, m->buff_active);
                }
            });
        buff_start_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/buff/start", latest, [this](std_msgs::msg::Bool::ConstSharedPtr m) {
                if (m->data && buff_enabled_ && mission_stage_ != "BUFF") {
                    buff_done_ = false;
                    buff_started_ = false;
                    buff_exhausted_ = false;
                    have_goal_ = false;
                }
            });
        resume_sub_ = create_subscription<std_msgs::msg::Bool>("/mission/resume", latest,
                                                               [this](std_msgs::msg::Bool::ConstSharedPtr m) {
                                                                   if (m->data) {
                                                                       mission_paused_ = false;
                                                                       manual_goal_ = false;
                                                                       have_goal_ = false;
                                                                   }
                                                               });
        follow_sub_ = create_subscription<std_msgs::msg::String>(
            "/navigation/follow_status", cached, [this](std_msgs::msg::String::ConstSharedPtr m) {
                auto marker = m->data.find(" goal=");
                uint64_t reported_goal = 0;

                if (marker != std::string::npos) {
                    try {
                        reported_goal = std::stoull(m->data.substr(marker + 6));
                    } catch (const std::exception&) {
                        return;
                    }
                }

                if (reported_goal != goal_id_)
                    return;
                if(mission_target_==4 && enemy_healths_[3]>0 &&
                   m->data.find(" four_hold=1 ")!=std::string::npos) {
                    try {
                        auto speed=m->data.find(" measured_speed_mps=");
                        auto pose=m->data.find(" position_m=");
                        if(speed!=std::string::npos && pose!=std::string::npos &&
                           std::stod(m->data.substr(speed+20))<=.1) {
                            auto xy=m->data.substr(pose+12);auto comma=xy.find(',');
                            campus::PlanPoint p{std::stod(xy),std::stod(xy.substr(comma+1))};
                            if(campus::onCombatHighland(base_,highland_cells_,p))memory_.rememberFourView(p);
                        }
                    } catch(const std::exception&) {}
                }
                if (mission_stage_ == "OPENING_RAMP" && m->data.rfind("ARRIVED ", 0) == 0)
                    memory_.opening_done = true;
                combat_holding_ = m->data.rfind("COMBAT_", 0) == 0;
                combat_received_ = SteadyClock::now();
                approach_progress_.update(m->data.substr(0, m->data.find(' ')), start_,
                                          std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count());
                locally_blocked_ = approach_progress_.needsReplan(
                    std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count());
                bool reached =
                    m->data.rfind("ARRIVED ", 0) == 0 ||
                    (m->data.rfind("COMBAT_", 0) == 0 && m->data.find("combat_ready=true") != std::string::npos);

                if (reached && !at_attack_pose_) {
                    at_attack_pose_ = true;
                    attack_progress_ = SteadyClock::now();
                }

                if (m->data.rfind("MULTIPLE_SPEED_PUBLISHERS", 0) == 0) {
                    mission_paused_ = true;
                    have_goal_ = false;
                } else if (m->data.rfind("STUCK_SELECT_NEW_GOAL", 0) == 0) {
                    if (autonomous_ && !manual_goal_ && !mission_paused_) {
                        // A blocked approach revokes this motion goal, not the
                        // whole autonomous mission. Fresh feedback must produce
                        // a different reachable observation pose before restart.
                        avoid_goal_ = goal_;
                        recovery_replan_ = true;
                        locally_blocked_ = true;
                        blocked_replans_ = 0;
                        have_goal_ = false;
                    } else {
                        mission_paused_ = true;
                        have_goal_ = false;
                    }
                } else if (autonomous_ && !mission_paused_ && !manual_goal_ &&
                           m->data.rfind("CANCELLED_SELECT_NEW_GOAL", 0) == 0) {
                    // A watchdog revokes the old goal. Only a fresh automatic
                    // plan, after all feedback guards pass, may restart it.
                    have_goal_ = false;
                }
            });

        plan_pub_ = create_publisher<campus_aim_interfaces::msg::NavigationPlan>("/planning/snapshot", cached);
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/reference/blocked_map", cached, std::bind(&DijkstraPlanner::mapCallback, this, std::placeholders::_1));
        raw_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/reference/map", cached, std::bind(&DijkstraPlanner::rawMapCallback, this, std::placeholders::_1));
        enemy_sub_ = create_subscription<std_msgs::msg::String>(
            "/navigation/enemies", latest, std::bind(&DijkstraPlanner::enemyCallback, this, std::placeholders::_1));
        state_sub_ = create_subscription<tdt_interface::msg::AutoAimState>(
            "/autoaim/state", latest, std::bind(&DijkstraPlanner::stateCallback, this, std::placeholders::_1));

        if (strict_order_)
            tracks_sub_ = create_subscription<campus_aim_interfaces::msg::TrackFrame>(
                "/tracker/targets", latest, [this](campus_aim_interfaces::msg::TrackFrame::ConstSharedPtr m) {
                    double stamp = stampSeconds(m->frame.header.stamp), a = now().seconds() - stamp;
                    if (m->reset || !have_state_ || !own_alive_ || !std::isfinite(a) || a < -.1 || a > .2 ||
                        m->frame.state.header.frame_id != state_frame_ || stamp * 1e9 < enemy_min_stamp_)
                        return;
                    for (const auto& t : m->targets)
                        if (campus::armorHealthIndex(t.number) >= 0 && campus::armorHealthIndex(t.number) < 5 &&
                            t.state == "TRACKING" && std::isfinite(t.last_observation_age) &&
                            t.last_observation_age >= 0 && t.last_observation_age <= .2 &&
                            finitePoint({t.center.x, t.center.y}) && std::isfinite(t.position_variance) &&
                            t.position_variance >= 0 && t.position_variance <= .1) {
                            int index = campus::armorHealthIndex(t.number), id = index + 1;

                            if (ignore_one_ && id == 1)
                                continue;
                            double observation_stamp = stamp - t.last_observation_age;
                            double observation_age = now().seconds() - observation_stamp;

                            if (enemy_healths_[index] <= 0 || observation_stamp <= visual_enemy_stamps_[index] ||
                                observation_age < -.1 || observation_age > .2)
                                continue;
                            double x = -t.center.y, z = t.center.x;
                            auto known = std::find_if(enemies_.begin(), enemies_.end(),
                                                      [id](const auto& e) { return e.id == id; });
                            // A known numbered body must agree with its public
                            // position plus the motion and observation error bounds.
                            double public_age = std::max(0., now().seconds() - last_enemy_stamp_ * 1e-9);
                            double match_bound = uncertainty_ + bodyRadius(id) + 3 * std::sqrt(t.position_variance) +
                                                 speed_bound_ * (public_age + std::max(0., observation_age));

                            if (known != enemies_.end() &&
                                std::hypot(x - known->unity_x_m, z - known->unity_z_m) > match_bound)
                                continue;
                            visual_enemies_[index] = {id, x, z, static_cast<double>(enemy_healths_[index]),
                                                      known == enemies_.end() ? 0 : known->attack_range_raw};
                            visual_enemy_stamps_[index] = observation_stamp;
                        }
                });

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/planning/goal", latest, std::bind(&DijkstraPlanner::goalCallback, this, std::placeholders::_1));
        start_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/planning/start", latest, std::bind(&DijkstraPlanner::startCallback, this, std::placeholders::_1));
        cancel_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/planning/cancel", latest, std::bind(&DijkstraPlanner::cancelCallback, this, std::placeholders::_1));
        double initial_start_x = declare_parameter<double>("demo_start_x_m", 19.6);
        double initial_start_y = declare_parameter<double>("demo_start_y_m", 9.0);
        double initial_goal_x = declare_parameter<double>("demo_goal_x_m", 26.6);
        double initial_goal_y = declare_parameter<double>("demo_goal_y_m", 9.0);

        if (demo_) {
            start_ = {initial_start_x, initial_start_y};
            goal_ = {initial_goal_x, initial_goal_y};
            have_start_ = finitePoint(start_);

            have_goal_ = finitePoint(goal_);
            goal_id_ = static_cast<uint64_t>(now().nanoseconds());
        }

        int period_ms = static_cast<int>(1000 / replan_hz_);
        timer_ = create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&DijkstraPlanner::tick, this));
        RCLCPP_INFO(get_logger(),
                    "Dijkstra: frame=%s, demo=%d, enemy input=1 Hz typical; planner has no speed publishers",
                    frame_.c_str(), demo_);
    }

  private:
    typedef std::chrono::steady_clock SteadyClock;

    std::vector<campus::BroadcastEnemy> planningEnemies() const {
        auto result = enemies_;
        for (int i = 0; strict_order_ && i < 5; ++i) {
            double stamp = visual_enemy_stamps_[i], a = planning_time_ - stamp;
            if (stamp <= 0 || stamp * 1e9 < enemy_min_stamp_ || a < -.1 || a > (i == 0 ? enemy_timeout_ : .2) ||
                enemy_healths_[i] <= 0)
                continue;
            auto known = std::find_if(result.begin(), result.end(), [i](const auto& e) { return e.id == i + 1; });

            if (known == result.end())
                result.push_back(visual_enemies_[i]);
            else
                *known = visual_enemies_[i];
        }

        return result;
    }

    bool finitePoint(campus::PlanPoint point) const {
        return std::isfinite(point.x) && std::isfinite(point.y);
    }

    double silence(SteadyClock::time_point received) const {
        return std::chrono::duration<double>(SteadyClock::now() - received).count();
    }

    double stampSeconds(const builtin_interfaces::msg::Time& stamp) const {
        return static_cast<double>(stamp.sec) + stamp.nanosec * 1e-9;
    }

    bool decodeMap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr message, campus::DijkstraGrid& grid) {
        if (message->info.width > 1000000 || message->info.height > 1000000) {
            return false;
        }

        grid.width = static_cast<int>(message->info.width);
        grid.height = static_cast<int>(message->info.height);
        grid.resolution = message->info.resolution;
        grid.origin_x = message->info.origin.position.x;
        grid.origin_y = message->info.origin.position.y;
        grid.cells = message->data;
        const geometry_msgs::msg::Quaternion& rotation = message->info.origin.orientation;

        if (message->header.frame_id != frame_ || !grid.valid() || !std::isfinite(rotation.w) ||
            std::abs(std::abs(rotation.w) - 1) > 1e-6 || rotation.x != 0 || rotation.y != 0 || rotation.z != 0) {
            RCLCPP_WARN(get_logger(), "Reject map: frame, dimensions or origin orientation mismatch");
            return false;
        }

        for (int8_t value : grid.cells) {
            if (value < -1 || value > 100) {
                return false;
            }
        }

        return true;
    }

    void mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
        campus::DijkstraGrid incoming;
        have_map_ = decodeMap(message, incoming);
        if (have_map_) {
            if (incoming.cells != base_.cells || incoming.width != base_.width || incoming.height != base_.height ||
                incoming.resolution != base_.resolution) {
                wall_cost_ = campus::wallPreference(incoming, wall_distance_, wall_weight_);
            }

            base_ = std::move(incoming);
            map_message_ = *message;
        }
    }

    void rawMapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
        have_raw_map_ = decodeMap(message, raw_);
    }

    std::string noPathReason() const {
        // 诊断才比较其他图层。执行用的路径始终来自静态膨胀图加当前敌情。
        if (base_.plan(start_, goal_).status == "OK") {
            return "DYNAMIC_DISCONNECTED";
        }

        if (!have_raw_map_ || raw_.width != base_.width || raw_.height != base_.height ||
            raw_.resolution != base_.resolution || raw_.origin_x != base_.origin_x || raw_.origin_y != base_.origin_y) {
            return "STATIC_OR_INFLATION_DISCONNECTED";
        }

        if (raw_.plan(start_, goal_).status == "OK") {
            return "INFLATION_DISCONNECTED";
        }

        return "STATIC_DISCONNECTED";
    }

    void enemyCallback(std_msgs::msg::String::ConstSharedPtr message) {
        try {
            campus::EnemyPacket packet = campus::parseEnemyPacket(message->data, fields_);
            double age = now().seconds() - packet.stamp_ns * 1e-9;
            if (packet.player_id != player_ || packet.frame_id != enemy_frame_) {
                have_enemies_ = false;
                RCLCPP_WARN(get_logger(), "Reject enemy packet: wrong player/frame");
                return;
            }

            if (!std::isfinite(age) || age < -0.1 || age > enemy_timeout_) {
                return;
            }

            if (packet.stamp_ns < enemy_min_stamp_) {
                return;
            }

            if (packet.stamp_ns <= last_enemy_stamp_) {
                return;
            }

            if (!enemy_session_.empty() && enemy_session_ != packet.session_id) {
                resetAlignment();
                memory_.reset();
            rune_return_.reset();
                history_.poses.clear();
                recovery_replan_ = false;
                have_goal_ = false;
                goal_event_ = "CLEARED_ENEMY_SESSION";
                have_start_ = demo_;
                have_state_ = false;

                have_enemies_ = false;
                last_enemy_stamp_ = 0;
                RCLCPP_WARN(get_logger(), "New enemy session: clear previous goal; select a new goal in RViz");
            }

            enemy_session_ = packet.session_id;
            last_enemy_stamp_ = packet.stamp_ns;
            enemies_ = packet.enemies;
            enemy_received_ = SteadyClock::now();
            have_enemies_ = true;
        } catch (const std::exception& error) {
            // 坏包不刷新有效期，也不把未知敌情当作空列表。
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Reject enemy JSON: %s", error.what());
        }
    }

    void stateCallback(tdt_interface::msg::AutoAimState::ConstSharedPtr state) {
        if (demo_) {
            return;
        }

        double stamp = stampSeconds(state->header.stamp);
        double age = now().seconds() - stamp;
        if (stamp <= 0 || !std::isfinite(age) || age < -0.1 || age > state_timeout_ ||
            !std::isfinite(state->position.x) || !std::isfinite(state->position.y) ||
            !std::isfinite(state->remaining_seconds)) {
            have_state_ = false;
            return;
        }

        if (last_state_stamp_ > 0 && stamp <= last_state_stamp_) {
            return;
        }

        if (last_state_stamp_ > 0 &&
            (state->frame_sequence < last_sequence_ || state->header.frame_id != state_frame_ ||
             state->remaining_seconds > last_remaining_ + 2.)) {
            spawn_refill_.reset();
            memory_.reset();
            rune_return_.reset();
            buff_done_ = false;
            buff_started_ = false;
            buff_exhausted_ = false;
            buff_effect_active_ = false;
            buff_count_ = 0;
            buff_rejections_ = 0;

            buff_min_sequence_ = state->frame_sequence;
            resetAlignment();
            have_goal_ = false;
            goal_event_ = "CLEARED_STATE_RESET";
            have_enemies_ = false;
            enemy_min_stamp_ = static_cast<uint64_t>(rclcpp::Time(state->header.stamp).nanoseconds());
        }

        if (last_state_stamp_ > 0 && state->frame_sequence == last_sequence_) {
            return;
        }
#ifdef CAMPUS_STATE_HAS_SESSION_ID
        std::ostringstream session;
        session << state->session_id;

        if (!state_session_.empty() && state_session_ != session.str()) {
            spawn_refill_.reset();
            memory_.reset();
            rune_return_.reset();
            buff_done_ = false;
            buff_started_ = false;
            buff_exhausted_ = false;
            buff_effect_active_ = false;
            buff_count_ = 0;
            buff_rejections_ = 0;

            buff_min_sequence_ = state->frame_sequence;
            resetAlignment();
            have_goal_ = false;
            goal_event_ = "CLEARED_STATE_SESSION";
            have_enemies_ = false;
            enemy_min_stamp_ = static_cast<uint64_t>(rclcpp::Time(state->header.stamp).nanoseconds());
        }

        state_session_ = session.str();
#endif
        last_state_stamp_ = stamp;
        last_remaining_ = state->remaining_seconds;
        last_sequence_ = state->frame_sequence;
        state_frame_ = state->header.frame_id;
        own_alive_ = state->health > 0 && state->remaining_seconds > 0;

        if (!own_alive_) {
            rune_return_.reset();
            if (mission_stage_ == "BUFF")
                buff_done_ = true;
            have_goal_ = false;
            goal_event_ = "CLEARED_DEATH_OR_MATCH_END";
        }

        if (at_attack_pose_ && mission_target_ >= 1 && mission_target_ <= 7 &&
            state->enemy_healths[mission_target_ - 1] < enemy_healths_[mission_target_ - 1])
            attack_progress_ = SteadyClock::now();
        enemy_healths_ = state->enemy_healths;
        ammo_ = state->ammo_remaining;
        capacity_ = state->ammo_capacity;
        weakened_ = state->is_weakened;
        own_health_ = state->health;
        max_health_ = state->max_health;
        // map=(Unity Z, -Unity X, Unity Y)，状态位置仍是场景单位。
        start_ = projection_.fromUnity(-state->position.y / fields_.scene_units_per_m,
                                       state->position.x / fields_.scene_units_per_m);
        have_start_ = finitePoint(start_);
        auto capture=start_;
        bool capture_valid=have_start_ && (highland_cells_.empty() || campus::onCombatHighland(base_,highland_cells_,start_));
        if (!capture_valid && mission_target_==4 && have_goal_ && campus::onCombatHighland(base_,highland_cells_,goal_)) {
            capture=goal_;capture_valid=true;
        }
        memory_.observe(state->enemy_healths[3], capture, capture_valid);

        if (strict_order_ && own_alive_ && have_start_) {
            spawn_refill_.capture(start_);
            if (ammo_ <= 0 || weakened_ || (max_health_ > 0 && own_health_ < max_health_ * .3))
                spawn_refill_.begin();
            double vx = -state->twist.linear.y / fields_.scene_units_per_m;
            double vz = state->twist.linear.x / fields_.scene_units_per_m;
            spawn_refill_.update(start_,
                                 {projection_.matrix[0] * vx + projection_.matrix[1] * vz,
                                  projection_.matrix[3] * vx + projection_.matrix[4] * vz},
                                 ammo_, capacity_, weakened_, now().seconds());
        }

        state_received_ = SteadyClock::now();
        have_state_ = true;
    }

    bool acceptPose(const std_msgs::msg::Header& header, campus::PlanPoint point) {
        double age = now().seconds() - stampSeconds(header.stamp);
        return header.frame_id == frame_ && finitePoint(point) && std::isfinite(age) && age >= -0.1 && age <= 5;
    }

    void goalCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        ++goals_received_;
        campus::PlanPoint point{message->pose.position.x, message->pose.position.y};
        if (acceptPose(message->header, point)) {
            goal_ = point;
            manual_goal_ = true;
            mission_paused_ = false;
            have_goal_ = true;
            // 同一点重新选择也是新任务，允许显式恢复已取消的任务。
            goal_id_ = std::max(goal_id_ + 1, static_cast<uint64_t>(now().nanoseconds()));
            goal_event_ = "ACCEPTED";
            RCLCPP_INFO(get_logger(), "Accepted goal %lu on %s: (%.3f, %.3f)", static_cast<unsigned long>(goal_id_),
                        goal_sub_->get_topic_name(), point.x, point.y);
        } else {
            have_goal_ = false;
            goal_event_ = message->header.frame_id != frame_
                              ? "REJECTED_FRAME"
                              : (!finitePoint(point) ? "REJECTED_COORDINATES" : "REJECTED_TIMESTAMP");
            RCLCPP_WARN(get_logger(), "%s: received frame '%s', required '%s'; use current timestamp",
                        goal_event_.c_str(), message->header.frame_id.c_str(), frame_.c_str());
        }
    }

    void cancelCallback(std_msgs::msg::Bool::ConstSharedPtr message) {
        if (message->data) {
            have_goal_ = false;
            goal_event_ = "CLEARED_CANCEL";
            mission_paused_ = true;
            recovery_replan_ = false;
            history_.poses.clear();
        }
    }

    void resetAlignment() {
        rune_return_.reset();
        history_.poses.clear();
        recovery_replan_ = false;

        projection_.matrix = initial_projection_;
        aligned_ = false;
    }

    void startCallback(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message) {
        campus::PlanPoint point{message->pose.pose.position.x, message->pose.pose.position.y};
        if (demo_ && acceptPose(message->header, point)) {
            start_ = point;
            have_start_ = true;
        } else if (allow_alignment_ && have_state_ && have_map_ && silence(state_received_) <= state_timeout_ &&
                   acceptPose(message->header, point)) {
            int cell = base_.cellAt(point);

            if (cell < 0 || base_.cells[cell] != 0) {
                RCLCPP_WARN(
                    get_logger(),
                    "Alignment rejected: choose the observed robot position with footprint clearance, not a wall");
                return;
            }

            // 用户明确给出机器人在参考图上的位置；只修改查看平移，不挖墙、不自动吸附。
            projection_.matrix[2] += point.x - start_.x;
            projection_.matrix[5] += point.y - start_.y;
            start_ = point;
            have_goal_ = false;
            goal_event_ = "CLEARED_VIEW_ALIGNMENT";
            aligned_ = true;
            RCLCPP_INFO(get_logger(),
                        "Reference view aligned explicitly; select a new goal. Geometry remains unverified");
        }
    }

    visualization_msgs::msg::Marker marker(const std::string& space, int id, int type) {
        visualization_msgs::msg::Marker output;
        output.header.frame_id = frame_;
        output.header.stamp = now();
        output.ns = space;
        output.id = id;
        output.type = type;
        output.action = visualization_msgs::msg::Marker::ADD;
        output.pose.orientation.w = 1;

        output.color.a = 1;
        return output;
    }

    bool enemyAlive(int id) const {
        // 摧毁建筑不等于其实体消失；保留建筑占用，机器人战亡则移除。
        return id >= 6 || demo_ || !use_healths_ || !have_state_ || enemy_healths_[id - 1] > 0;
    }

    double bodyRadius(int id) const {
        if (id == 6) {
            return outpost_radius_;
        }

        if (id == 7) {
            return base_radius_;
        }

        return enemy_radius_;
    }

    void publishEnemies(double age, bool fresh) {
        visualization_msgs::msg::MarkerArray output;
        visualization_msgs::msg::Marker remove = marker("", 0, visualization_msgs::msg::Marker::SPHERE);
        remove.action = visualization_msgs::msg::Marker::DELETEALL;
        output.markers.push_back(remove);

        for (const campus::BroadcastEnemy& enemy : planningEnemies()) {
            if (!enemyAlive(enemy.id)) {
                continue;
            }

            campus::PlanPoint point = projection_.fromUnity(enemy.unity_x_m, enemy.unity_z_m);
            visualization_msgs::msg::Marker body =
                marker("enemies", enemy.id, visualization_msgs::msg::Marker::CYLINDER);
            body.pose.position.x = point.x;
            body.pose.position.y = point.y;
            body.pose.position.z = 0.2;
            body.scale.x = body.scale.y = 2 * bodyRadius(enemy.id);
            body.scale.z = 0.4;
            body.color.r = fresh && enemy.health > 0 ? 1 : 0.5;
            body.color.g = fresh ? 0.15 : 0.5;
            body.color.b = enemy.id >= 6 ? 0.6 : 0.15;

            output.markers.push_back(body);
            visualization_msgs::msg::Marker text =
                marker("enemy_labels", enemy.id, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
            text.pose = body.pose;
            text.pose.position.z = 0.8;
            text.scale.z = 0.45;
            text.color.r = text.color.g = text.color.b = 1;
            std::ostringstream label;
            label << "E" << enemy.id << " age=" << static_cast<int>(std::min(age, 1000.0) * 1000) << "ms";

            if (enemy.id == 6) {
                label << " OUTPOST";
            } else if (enemy.id == 7) {
                label << " BASE";
            }

            if (!fresh) {
                label << " STALE";
            }

            if (enemy.health == 0) {
                label << " DESTROYED";
            }

            text.text = label.str();
            output.markers.push_back(text);
            visualization_msgs::msg::Marker disc =
                marker("enemy_uncertainty", enemy.id, visualization_msgs::msg::Marker::CYLINDER);
            disc.pose = body.pose;
            disc.pose.position.z = 0.04;
            double radius = dynamicRadius(enemy.id, age);
            disc.scale.x = disc.scale.y = 2 * radius;
            disc.scale.z = 0.05;
            disc.color.r = 1;
            disc.color.g = 0.45;
            disc.color.a = 0.25;

            output.markers.push_back(disc);
        }

        enemy_pub_->publish(output);
    }

    double dynamicRadius(int id, double age) const {
        // A lost visual unit must retain a growing collision margin even
        // though stale vision can no longer nominate an attack position.
        if (id >= 1 && id <= 5) {
            double stamp = visual_enemy_stamps_[id - 1], visual_age = planning_time_ - stamp;
            if (strict_order_ && stamp > 0 && stamp * 1e9 >= enemy_min_stamp_ && visual_age >= -.1 &&
                visual_age <= (id == 1 ? enemy_timeout_ : .2))
                age = std::max(0., visual_age);
        }

        double radius = bodyRadius(id) + own_radius_ + margin_ + uncertainty_;
        if (id <= 5) {
            radius += speed_bound_ * (std::min(age, enemy_timeout_) + forecast_time_);
        }

        return radius;
    }

    void publishPoints(bool start_fresh, const std::string& status, const std::string& start_block,
                       const std::string& goal_block, const std::string& no_path_reason) {
        visualization_msgs::msg::MarkerArray output;
        visualization_msgs::msg::Marker remove = marker("", 0, visualization_msgs::msg::Marker::SPHERE);
        remove.action = visualization_msgs::msg::Marker::DELETEALL;
        output.markers.push_back(remove);

        for (int index = 0; index < 2; ++index) {
            if ((index == 0 && !have_start_) || (index == 1 && !have_goal_)) {
                continue;
            }

            campus::PlanPoint point = index == 0 ? start_ : goal_;
            std::string blocked = index == 0 ? start_block : goal_block;
            visualization_msgs::msg::Marker sphere = marker("points", index, visualization_msgs::msg::Marker::SPHERE);
            sphere.pose.position.x = point.x;
            sphere.pose.position.y = point.y;
            sphere.pose.position.z = 0.15;
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.4;
            sphere.color.g = index == 0 && start_fresh ? 1 : 0.3;

            sphere.color.b = index == 1 ? 1 : 0.3;
            output.markers.push_back(sphere);
            visualization_msgs::msg::Marker label =
                marker("point_labels", index, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
            label.pose = sphere.pose;
            label.pose.position.y += index == 0 ? 0.65 : -0.65;
            label.pose.position.z = 0.8;
            label.scale.z = 0.45;
            label.color.r = label.color.g = label.color.b = 1;
            std::ostringstream description;
            description.setf(std::ios::fixed);
            description.precision(2);

            description << (index == 0 ? "START" : "GOAL") << " (" << point.x << ',' << point.y << ")\n" << blocked;
            label.text = description.str();
            output.markers.push_back(label);

            if (blocked != "FREE" && blocked != "WAIT_MAP") {
                visualization_msgs::msg::Marker cross =
                    marker("blocked_points", index, visualization_msgs::msg::Marker::LINE_LIST);
                cross.pose = sphere.pose;
                cross.scale.x = 0.10;
                cross.color.r = 1;

                for (int corner = 0; corner < 4; ++corner) {
                    geometry_msgs::msg::Point end;
                    end.x = corner % 2 == 0 ? -0.3 : 0.3;
                    end.y = corner < 2 ? end.x : -end.x;
                    end.z = 0.15;
                    cross.points.push_back(end);
                }

                output.markers.push_back(cross);
            }
        }

        visualization_msgs::msg::Marker text =
            marker("planner_status", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
        text.pose.position.x = have_map_ ? base_.origin_x + base_.width * base_.resolution / 2 : 14;
        text.pose.position.y = have_map_ ? base_.origin_y + base_.height * base_.resolution + .8 : 15.8;
        text.scale.z = 0.6;
        text.color.r = status == "OK" ? 0.2 : 1;
        text.color.g = status == "OK" ? 1 : 0.6;
        text.color.b = 0.2;
        text.text = status + (demo_ ? " | DEMO" : " | LIVE") +
                    (display_only_ ? " | DISPLAY ONLY" : " | SEE FOLLOWER OUTPUT MODE");

        if (autonomous_)
            text.text += "\nMISSION: " +
                         (mission_paused_ ? std::string("PAUSED")
                                          : (manual_goal_ ? std::string("MANUAL_OVERRIDE") : mission_stage_)) +
                         "  AMMO: " + std::to_string(ammo_);
        if (status == "GOAL_BLOCKED") {
            text.text += "\nGOAL: " + goal_block + " - select a free goal";
        } else if (status == "WAIT_GOAL") {
            text.text += autonomous_ ? "\nSee mission status; resume a paused mission or select a manual goal"
                                     : "\nUse 2D Goal Pose: drag on a free cell, then release";
            text.text += "\nGoal: " + goal_event_;
        } else if (status == "START_BLOCKED") {
            text.text += "\nSTART: " + start_block;

            if (allow_alignment_) {
                text.text += "\n2D Pose Estimate: align the observed robot on the reference map";
            }
        } else if (status == "NO_PATH") {
            text.text += "\n" + no_path_reason;
            if (no_path_reason == "INFLATION_DISCONNECTED") {
                text.text += "\nMap passage closes after safety inflation";
            } else if (no_path_reason == "DYNAMIC_DISCONNECTED") {
                text.text += "\nEnemy uncertainty blocks the route; replanning";
            }
        }

        output.markers.push_back(text);
        point_pub_->publish(output);
    }

    campus::RuneReturnRoute rune_return_;
    std::vector<int64_t> buff_ground_excluded_, buff_low_ledges_;
    bool runeRampTransitActive() const {
        return autonomous_ && !manual_goal_ && !mission_paused_ && strict_order_ && opening_enabled_ &&
            (mission_stage_=="OPENING_RAMP" || mission_stage_=="RUNE_RETURN" ||
             (mission_stage_=="BUFF" && memory_.opening_done));
    }
    bool runeTaskDue() const {
        if (buff_count_>=unsigned(buff_limit_) && !(mission_stage_=="BUFF" && !buff_done_)) return false;
        return buff_enabled_ && strict_order_ &&
               campus::runeVisitDue(buff_exhausted_, buff_done_, buff_effect_active_, buff_count_,
                                    silence(buff_finished_),
                                    campus::robotCleanupComplete(enemy_healths_, ignore_one_) &&
                                        enemy_healths_[5] <= 0 && enemy_healths_[6] > 0);
    }

    std::pair<campus::PlanPoint,bool> sentryOutpost() const {
        for(const auto& enemy:planningEnemies()) if(enemy.id==6)
            return {projection_.fromUnity(enemy.unity_x_m,enemy.unity_z_m),true};
        return {{NAN,NAN},false};
    }
    campus::DijkstraGrid sentrySightGrid() const {
        auto sight=raw_;
        // The highland lip blocks chassis motion below the barrel, while the
        // outpost tower and all other real obstacles still occlude the shot.
        for(auto cell:buff_low_ledges_) if(cell>=0 && size_t(cell)<sight.cells.size() && sight.cells[cell]>0)
            sight.cells[cell]=0;
        return sight;
    }
    bool sentryLocalHere() const {
        auto [outpost,known]=sentryOutpost();
        return memory_.anchor_known && campus::sentryLocalContains(start_,memory_.four_origin,outpost,known);
    }
    void restrictLocalSentry(campus::DijkstraGrid& grid) const {
        auto [outpost,known]=sentryOutpost();
        campus::restrictSentryLocal(grid,memory_.four_origin,outpost,known);
    }
    bool anchorTask() const {
        return !runeTaskDue() && anchor_enabled_ && strict_order_ && (memory_.opening_done || !opening_enabled_) &&
               memory_.anchor_known && campus::fieldRobotsCleared(enemy_healths_) && enemy_healths_[4] > 0 &&
               (ignore_one_ || enemy_healths_[0] <= 0);
    }

    bool tacticalGoal(const campus::DijkstraGrid& working, const std::vector<double>& risk,
                      campus::PlanPoint& objective, std::string& stage) {
        // Unit 1 has no public position. A brief loss during a turn must stop
        // pursuit and reacquire it, rather than oscillate to a distant unit.
        // The last observation never grants movement or firing permission.
        double unseen_one_age = planning_time_ - visual_enemy_stamps_[0];
        if (strict_order_ && !ignore_one_ && ordered_phase_ == "CLEAR_ROBOTS" && mission_target_ == 1 &&
            enemy_healths_[0] > 0 && visual_enemy_stamps_[0] > 0 && unseen_one_age > .2 &&
            unseen_one_age <= enemy_timeout_) {
            have_goal_ = false;
            stage = "WAIT_CLEAR_ROBOTS_POSITION";
            return false;
        }

        const auto costs = campus::routeCosts(working, start_, risk);
        std::vector<campus::TacticalCandidate> candidates;
        if (campus::pointDistance(start_, blocked_origin_) > .3)
            blocked_replans_ = 0;
        bool blocked_approach = recovery_replan_ || (locally_blocked_ && blocked_replans_ < 3);
        bool reposition = at_attack_pose_ &&
                          (mission_stage_ == "OUTPOST" || mission_stage_ == "BASE" || mission_stage_ == "ROBOT" ||
                           mission_stage_ == "CLEAR_ROBOTS") &&
                          silence(attack_progress_) > 8.;
        bool hold_mobile = combat_holding_ && silence(combat_received_) < .3 &&
                           (mission_stage_ == "ROBOT" || mission_stage_ == "CLEAR_ROBOTS");

        // A planar clear ray is not proof that the camera sees over a ramp.
        // Remember failed real arrival positions across rune visits instead
        // of repeatedly returning to the same nominally clear station.
        const bool sentry_station_failed=anchorTask() && mission_target_==5 && at_attack_pose_ &&
            campus::pointDistance(start_,memory_.anchor)<.3 &&
            silence(attack_progress_)>(planning_time_-visual_enemy_stamps_[4]>3. ? 3. : 8.);
        if(sentry_station_failed) {
            memory_.rejectSentryStation(memory_.anchor);
            memory_.sentry_advanced=false;
            hold_mobile=false;
            reposition=true;
        }

        if (hold_mobile)
            reposition = false;
        reposition = reposition || blocked_approach;
        if (reposition)
            avoid_goal_ = goal_;

        for (const auto& enemy : planningEnemies()) {
            if (strict_order_ && !campus::fieldRobotOrderAllowed(ordered_phase_, enemy.id,
                                                                enemy_healths_, field_order_))
                continue;
            if (sentry_last_ && strict_order_ && ordered_phase_ == "CLEAR_ROBOTS" && enemy.id == 5 &&
                !campus::fieldRobotsCleared(enemy_healths_))
                continue;
            if (ignore_one_ && enemy.id == 1)
                continue;
            if (enemy.id == 1 && planning_time_ - visual_enemy_stamps_[0] > .2)
                continue;

            if (!enemyAlive(enemy.id) || enemy.id < 1 || enemy.id > 7 || (enemy.id == 7 && enemy_healths_[5] > 0))
                continue;
            if (strict_order_ &&
                (enemy_healths_[enemy.id - 1] <= 0 ||
                 (ordered_phase_ == "CLEAR_ROBOTS" ? enemy.id >= 6
                                                   : (ordered_phase_ == "OUTPOST" ? enemy.id != 6 : enemy.id != 7))))
                continue;
            std::string kind = enemy.id == 7 ? "BASE" : (enemy.id == 6 ? "OUTPOST" : "ROBOT");

            if (strict_order_ && kind == "ROBOT")
                kind = "CLEAR_ROBOTS";
            double reward = enemy.id == 7 ? gains_.base : (enemy.id == 6 ? gains_.unlock : gains_.robot);
            auto point = projection_.fromUnity(enemy.unity_x_m, enemy.unity_z_m);
            campus::PlanPoint attack;
            bool same = mission_stage_ == kind && mission_target_ == enemy.id;
            auto excluded = same ? avoid_goal_ : campus::PlanPoint{INFINITY, INFINITY};
            bool seen = enemy.id <= 5 && visual_enemy_stamps_[enemy.id - 1] > 0 &&
                        planning_time_ - visual_enemy_stamps_[enemy.id - 1] <= .2;
            // A far attack ring is useful once armor is visible. Without any
            // observation it can coincide with the robot's present location,
            // incorrectly declaring arrival and waiting forever for vision.
            // Public positions nominate a closer Dijkstra observation pose;
            // only real vision subsequently enables engagement.
            double observation_range = enemy.id <= 5 && !seen ? 3.6 : INFINITY;
            const bool anchored = anchorTask() && enemy.id == 5;
            const bool elevated = !highland_cells_.empty() && (enemy.id==4 || anchored);
            auto firing_grid=working;
            if (elevated) campus::restrictCombatHighland(firing_grid,highland_cells_);
            if(anchored && sentryLocalHere()) restrictLocalSentry(firing_grid);
            auto firing_costs=elevated && campus::onCombatHighland(working,highland_cells_,start_)
                ? campus::routeCosts(firing_grid,start_,risk) : costs;
            if(anchored && !sentryLocalHere()) {
                restrictLocalSentry(firing_grid);
                // Costs above still allow returning from outside the local area;
                // all nominated destinations stay in the original neighbourhood.
            }
            const bool close_combat=enemy.id==4 || anchored;
            auto firing_sight=anchored?sentrySightGrid():raw_;
            auto [outpost,outpost_known]=sentryOutpost();
            if(anchored && outpost_known) firing_sight.blockCircle(outpost,outpost_radius_+.1);
            const int parked_cell=working.cellAt(memory_.anchor);
            const bool parked_sentry=anchored && memory_.sentry_advanced && parked_cell>=0 &&
                memory_.sentryStationAllowed(memory_.anchor) &&
                firing_grid.cells[parked_cell]==0 && std::isfinite(firing_costs[parked_cell]) &&
                campus::sentrySightClear(firing_sight,memory_.anchor,point,bodyRadius(enemy.id),outpost,outpost_known,outpost_radius_);
            if(parked_sentry) attack=memory_.anchor;
            bool attack_found = parked_sentry ? true : anchored
                ? campus::chooseSentryFrontier(firing_grid,firing_sight,highland_cells_,firing_costs,
                        memory_.four_origin,point,bodyRadius(enemy.id),attack,outpost,outpost_known,outpost_radius_,memory_.rejected_sentry)
                : close_combat
                ? campus::chooseCloseCombatPosition(firing_grid,firing_sight,firing_costs,point,bodyRadius(enemy.id),attack,
                         excluded, anchored ? memory_.anchor : campus::PlanPoint{INFINITY,INFINITY}, anchored ? anchor_radius_ : 0.)
                : anchored ? campus::chooseAnchorPosition(firing_grid, raw_, firing_costs, memory_.anchor, anchor_radius_, point,
                                                        bodyRadius(enemy.id), attack, excluded)
                : enemy.id >= 6
                    ? campus::chooseStandoffPosition(working, raw_, costs, point, bodyRadius(enemy.id),
                                                     enemy.id == 6 ? outpost_band_ : base_band_, attack, excluded)
                    : campus::chooseAttackPosition(firing_grid, raw_, firing_costs, point, bodyRadius(enemy.id), attack, excluded,
                                                   observation_range);

            if (blocked_approach && seen && !anchored && !elevated) {
                campus::PlanPoint alternative;
                if (campus::stoppedAttackPosition(working, raw_, start_, point, bodyRadius(enemy.id), alternative)) {
                    attack = alternative;
                    attack_found = true;
                    reposition = true;
                }
            }

            if (!attack_found && anchored && !campus::anchorContains(start_,memory_.anchor,anchor_radius_))
                attack_found=campus::chooseAnchorReturn(firing_grid,firing_costs,memory_.anchor,anchor_radius_,attack);
            if (!attack_found)
                continue;
            candidates.push_back(campus::scoreTacticalCandidate(kind, enemy.id, point, attack, reward, working, costs,
                                                                gains_, enemy_healths_[enemy.id - 1]));
            if (!anchored && !parked_sentry && have_goal_ && same && !reposition && (!elevated || campus::onCombatHighland(working,highland_cells_,goal_)) && working.cellAt(goal_) >= 0 &&
                working.cells[working.cellAt(goal_)] == 0 &&
                (!close_combat || campus::pointDistance(goal_,point)<=std::max(3.,campus::pointDistance(attack,point)+.3)) &&
                (anchored
                     ? campus::anchorContains(goal_, memory_.anchor, anchor_radius_, .4)
                     : ((enemy.id < 6 ||
                         (enemy.id == 6 ? outpost_band_ : base_band_).contains(campus::pointDistance(goal_, point))) &&
                        campus::pointDistance(goal_, point) <= observation_range + .15)) &&
                campus::attackRayFree(firing_sight, goal_, point, bodyRadius(enemy.id)))
                candidates.push_back(campus::scoreTacticalCandidate(kind, enemy.id, point, goal_, reward, working,
                                                                    costs, gains_, enemy_healths_[enemy.id - 1]));
        }

        double ammo_deficit = capacity_ > 0 ? std::clamp(1. - static_cast<double>(ammo_) / capacity_, 0., 1.) : 0;
        double health_deficit =
            max_health_ > 0 ? std::clamp(1. - static_cast<double>(own_health_) / max_health_, 0., 1.) : 0;
        double recovery_gain = gains_.ammo * ammo_deficit + gains_.health * health_deficit;

        if (!strict_order_ && (ammo_deficit > .2 || health_deficit > .2))
            for (size_t i = 0; i < refill_points_.size(); i += 2) {
                campus::PlanPoint point{refill_points_[i], refill_points_[i + 1]};
                candidates.push_back(
                    campus::scoreTacticalCandidate("REFILL", 0, point, point, recovery_gain, working, costs, gains_));
            }

        double time = now().seconds();
        if (at_attack_pose_ && mission_stage_ == "GAIN" && silence(attack_progress_) > 3. && mission_target_ < 0) {
            gain_used_until_[-mission_target_ - 1] = time + 120.;
            have_goal_ = false;
        }

        for (size_t i = 0; !strict_order_ && i < gain_values_.size(); ++i)
            if (time >= gain_used_until_[i]) {
                campus::PlanPoint point{gain_points_[2 * i], gain_points_[2 * i + 1]};
                candidates.push_back(campus::scoreTacticalCandidate("GAIN", -static_cast<int>(i) - 1, point, point,
                                                                    gain_values_[i], working, costs, gains_));
            }

        campus::TacticalCandidate current, best;
        for (const auto& candidate : candidates) {
            if (campus::betterTacticalCandidate(candidate, best, false, gains_))
                best = candidate;
            if (have_goal_ && candidate.stage == mission_stage_ && candidate.target_id == mission_target_ &&
                campus::pointDistance(candidate.goal, goal_) < 1e-6)
                current = candidate;
        }

        if (!reposition && std::isfinite(current.score) &&
            !campus::betterTacticalCandidate(best, current, true, gains_))
            best = current;
        if ((mission_target_==4 || mission_target_==5) && !reposition && std::isfinite(current.score) &&
            best.target_id==mission_target_ && campus::pointDistance(best.goal,goal_)<.6)
            best=current;
        if (hold_mobile && !blocked_approach && std::isfinite(current.score))
            best = current;
        if (!std::isfinite(best.score)) {
            have_goal_ = false;
            stage = strict_order_ ? "WAIT_" + ordered_phase_ + "_POSITION" : "NO_TACTICAL_POSITION";
            return false;
        }

        // Preserve target selection, then replace only robot 4's destination.
        // The destination is stable across visual gaps and no-damage timers.
        if (four_station_enabled_ && strict_order_ && best.stage=="CLEAR_ROBOTS" && best.target_id==4) {
            int cell=working.cellAt(four_station_);
            if (cell<0 || working.cells[cell]!=0 || !std::isfinite(costs[cell]) ||
                !campus::segmentWithReserve(working,four_station_,four_station_,.03) ||
                (!highland_cells_.empty() && !campus::onCombatHighland(working,highland_cells_,four_station_))) {
                have_goal_=false;
                stage="WAIT_FOUR_STATION";
                return false;
            }
            best.goal=four_station_;
            four_station_active_=true;
        }
        bool changed = !have_goal_ || best.stage != mission_stage_ || best.target_id != mission_target_ ||
                       campus::pointDistance(goal_, best.goal) > .01;
        if (changed) {
            if (blocked_approach) {
                ++blocked_replans_;
                blocked_origin_ = start_;
            }

            if (best.target_id != mission_target_ || best.stage != mission_stage_)
                avoid_goal_ = {INFINITY, INFINITY};
            mission_target_ = best.target_id;
            assignMissionGoal(best.goal, best.stage);
        }

        selected_gain_ = best.gain;
        selected_cost_ = best.route_cost;
        selected_score_ = best.score;
        objective = best.objective;
        stage = best.stage;

        return true;
    }

    void autonomousGoal(const campus::DijkstraGrid& working, const std::vector<double>& risk, bool ready) {
        four_station_active_=false;
        std::string stage = !autonomous_
                                ? "MANUAL"
                                : (mission_paused_ ? "PAUSED" : (manual_goal_ ? "MANUAL_OVERRIDE" : "WAIT_FEEDBACK"));
        campus::PlanPoint objective;
        bool target_known = false;

        if (autonomous_ && !mission_paused_ && !manual_goal_ && ready && have_raw_map_) {
            stage = strict_order_ ? campus::orderedAssaultObjective(enemy_healths_, ammo_, weakened_, ignore_one_)
                                  : campus::assaultObjective(enemy_healths_, ammo_, weakened_);
            ordered_phase_ = stage;
            if (strict_order_ && stage != "COMPLETE" && spawn_refill_.active())
                stage = "REFILL";

            if (stage != "COMPLETE" && max_health_ > 0 && own_health_ < max_health_ * .3)
                stage = "REFILL";
            if (!strict_order_ && stage != "COMPLETE" && mission_stage_ == "REFILL" &&
                (ammo_ < std::max(1, static_cast<int>(capacity_ * .8)) ||
                 (max_health_ > 0 && own_health_ < max_health_ * .8) || weakened_))
                stage = "REFILL";
            if (stage == "COMPLETE") {
                have_goal_ = false;
                history_.poses.clear();
                goal_event_ = "BASE_DESTROYED";
            } else if (stage == "REFILL" && strict_order_) {
                rune_return_.reset();
                if (!spawn_refill_.captured()) {
                    have_goal_ = false;
                    stage = "WAIT_SPAWN_POSITION";
                } else {
                    objective = spawn_refill_.point();
                    target_known = true;

                    if (!have_goal_ || mission_stage_ != "REFILL" || campus::pointDistance(goal_, objective) > .001) {
                        mission_target_ = 0;
                        assignMissionGoal(objective, "REFILL");
                    }
                }
            } else if (stage == "REFILL") {
                // Public tactical-map candidates. Arrival alone is not proof of
                // replenishment: only ammo/weakness feedback permits departure.
                std::vector<campus::PlanPoint> candidates;
                for (size_t i = 0; i < refill_points_.size(); i += 2)
                    candidates.push_back({refill_points_[i], refill_points_[i + 1]});

                if (mission_stage_ != "REFILL") {
                    refill_index_ = 0;
                    refill_since_ = SteadyClock::now();
                }

                if (have_goal_ && campus::pointDistance(start_, goal_) < .08) {
                    if (silence(refill_since_) > 6.) {
                        ++refill_index_;
                        have_goal_ = false;
                        refill_since_ = SteadyClock::now();
                    }
                } else
                    refill_since_ = SteadyClock::now();

                if (!have_goal_ || mission_stage_ != stage) {
                    auto costs = campus::routeCosts(working, start_, risk);
                    for (size_t attempt = 0; attempt < candidates.size(); ++attempt) {
                        objective = candidates[(refill_index_ + attempt) % candidates.size()];
                        int cell = working.cellAt(objective);

                        if (cell >= 0 && std::isfinite(costs[cell])) {
                            target_known = true;
                            break;
                        }
                    }

                    if (target_known)
                        assignMissionGoal(objective, stage);
                }

                objective = goal_;
                target_known = have_goal_;
            } else if (runeTaskDue()) {
                stage = "BUFF";
                if (buff_started_ && silence(buff_since_) > (buff_effect_active_ ? 90. : 45.)) {
                    // One timed-out visit cannot consume all remaining rune
                    // opportunities. The accepted count is authoritative.
                    buff_done_ = true;
                    buff_started_ = false;
                    buff_finished_ = SteadyClock::now();
                    have_goal_ = false;
                    stage = "BUFF_TIMEOUT";
                    RCLCPP_WARN(get_logger(), "Rune visit timeout; preserving remaining opportunities=%u",
                                buff_count_ < unsigned(buff_limit_) ? buff_limit_ - buff_count_ : 0);
                } else {
                    auto u = campus::runeUnityCentre(player_);
                    objective = projection_.fromUnity(u.x, u.y);
                    target_known = true;
                    auto n = player_ == 1 ? campus::PlanPoint{-std::sqrt(.5), -std::sqrt(.5)}
                                          : campus::PlanPoint{std::sqrt(.5), std::sqrt(.5)};
                    campus::PlanPoint outward{projection_.matrix[0] * n.x + projection_.matrix[1] * n.y,
                                              projection_.matrix[3] * n.x + projection_.matrix[4] * n.y};

                    if (!have_goal_ || mission_stage_ != "BUFF" || locally_blocked_) {
                        buff_done_ = false;
                        campus::PlanPoint attack;
                        if (campus::chooseRunePosition(working, raw_, campus::routeCosts(working, start_, risk),
                                                       objective, outward, attack, buff_ground_excluded_,
                                                       buff_low_ledges_)) {
                            mission_target_ = 0;
                            assignMissionGoal(attack, "BUFF");
                        } else {
                            have_goal_ = false;
                            stage = "WAIT_BUFF_APPROACH";
                        }
                    }
                }
            } else if (opening_enabled_ && strict_order_ && (stage == "CLEAR_ROBOTS" || buff_enabled_) &&
                       !memory_.opening_done) {
                stage = "OPENING_RAMP";
                auto costs = campus::routeCosts(working, start_, risk);
                target_known = campus::chooseOpeningRamp(working, costs, opening_cells_, player_, objective);

                if (target_known) {
                    if (!have_goal_ || mission_stage_ != stage || campus::pointDistance(goal_, objective) > .01) {
                        mission_target_ = 0;
                        assignMissionGoal(objective, stage);
                    }
                } else {
                    have_goal_ = false;
                    stage = "WAIT_OPENING_RAMP";
                }
            } else if (strict_order_ && rune_return_.available()) {
                stage = "RUNE_RETURN";
                if (!have_goal_ || mission_stage_ != stage) {
                    auto costs=campus::routeCosts(working,start_,risk);
                    if(rune_return_.prepareArrival(working,highland_cells_,costs)) {
                        mission_target_=0;
                        assignMissionGoal(rune_return_.destination(),stage);
                    } else {
                        have_goal_=false;
                        stage="WAIT_RUNE_RETURN_POSITION";
                    }
                }
                objective = rune_return_.destination();
                target_known = have_goal_;
            } else if(anchorTask() && !memory_.sentry_advanced) {
                stage="SENTRY_ADVANCE";
                target_known=false;
                for(const auto& enemy:planningEnemies()) if(enemy.id==5 && enemyAlive(5)) {
                    objective=projection_.fromUnity(enemy.unity_x_m,enemy.unity_z_m);
                    if(!have_goal_ || mission_stage_!=stage) {
                        auto top=working;
                        if(campus::onCombatHighland(working,highland_cells_,start_))
                            campus::restrictCombatHighland(top,highland_cells_);
                        if(sentryLocalHere()) restrictLocalSentry(top);
                        auto [outpost,outpost_known]=sentryOutpost();
                        campus::PlanPoint front;
                        if(campus::chooseSentryFrontier(top,sentrySightGrid(),highland_cells_,campus::routeCosts(top,start_,risk),
                                                        memory_.four_origin,objective,bodyRadius(5),front,outpost,outpost_known,outpost_radius_,memory_.rejected_sentry)) {
                            mission_target_=5;
                            assignMissionGoal(front,stage);
                        } else {have_goal_=false;stage="WAIT_SENTRY_ADVANCE_POSITION";}
                    }
                    target_known=have_goal_;
                    break;
                }
                if(!target_known) {have_goal_=false;stage="WAIT_SENTRY_ADVANCE_POSITION";}
            } else if (tactical_ || strict_order_) {
                target_known = tacticalGoal(working, risk, objective, stage);
            } else {
                int broadcast_id = stage == "OUTPOST" ? 6 : 7;

                for (const auto& enemy : planningEnemies())
                    if (enemy.id == broadcast_id) {
                        objective = projection_.fromUnity(enemy.unity_x_m, enemy.unity_z_m);
                        target_known = true;
                        break;
                    }

                if (!target_known) {
                    have_goal_ = false;
                    stage = "WAIT_OBJECTIVE_POSITION";
                } else if (!have_goal_ || stage != mission_stage_ || working.cellAt(goal_) < 0 ||
                           working.cells[working.cellAt(goal_)] != 0) {
                    campus::PlanPoint attack;
                    if (campus::chooseStandoffPosition(working, raw_, campus::routeCosts(working, start_, risk),
                                                       objective, bodyRadius(broadcast_id),
                                                       broadcast_id == 6 ? outpost_band_ : base_band_, attack))
                        assignMissionGoal(attack, stage);
                    else {
                        have_goal_ = false;
                        stage = "NO_ATTACK_POSITION";
                    }
                }
            }

            mission_stage_ = stage;
        }

        std_msgs::msg::String status;
        std::ostringstream text;
        text << stage << " autonomous=" << autonomous_ << " goal=" << goal_id_ << " target_class="
             << (stage == "OUTPOST"
                     ? 6
                     : (stage == "BASE" ? 8 : ((stage == "ROBOT" || stage == "CLEAR_ROBOTS" || stage=="SENTRY_ADVANCE") ? mission_target_ : 0)))
             << " target_known=" << target_known << " target_x_m=" << objective.x << " target_y_m=" << objective.y
             << " goal_x_m=" << goal_.x << " goal_y_m=" << goal_.y << " ammo=" << ammo_
             << " final_objective=BASE tactical=" << tactical_ << " expected_gain=" << selected_gain_
             << " route_cost=" << selected_cost_ << " utility=" << selected_score_
             << " base_shielded=" << (enemy_healths_[5] > 0) << " rune_opportunities=" << buff_count_
             << " rune_effect_active=" << buff_effect_active_ << " rune_exhausted=" << buff_exhausted_
             << " opening_ramp_done=" << memory_.opening_done << " sentry_last=" << sentry_last_
             << " sentry_anchor_active="
             << (anchorTask() && (stage == "CLEAR_ROBOTS" || stage == "WAIT_CLEAR_ROBOTS_POSITION"))
             << " sentry_local_required=" << (anchorTask() && (stage=="SENTRY_ADVANCE" || stage=="CLEAR_ROBOTS" || stage=="WAIT_CLEAR_ROBOTS_POSITION"))
             << " four_station_fixed=" << four_station_active_
             << " sentry_local_here=" << sentryLocalHere()
             << " sentry_origin_x_m=" << memory_.four_origin.x << " sentry_origin_y_m=" << memory_.four_origin.y
             << " sentry_advanced=" << memory_.sentry_advanced
             << " sentry_rejected_stations=" << memory_.rejected_sentry.size()
             << " robot_highland_here=" << (highland_cells_.empty() ? -1 : int(campus::onCombatHighland(base_,highland_cells_,start_)))
             << " robot_highland_required=" << (!highland_cells_.empty() && (stage=="SENTRY_ADVANCE" || (stage=="CLEAR_ROBOTS" && mission_target_==4) || (anchorTask() && (stage=="CLEAR_ROBOTS" || stage=="WAIT_CLEAR_ROBOTS_POSITION"))))
             << " sentry_anchor_known=" << memory_.anchor_known << " sentry_anchor_source="
             << (memory_.anchor_known ? (memory_.sentry_advanced ? "LOCAL_FROM_FOUR" : (memory_.anchor_from_four ? "FOUR_HP_ZERO" : "LAST_STAGE_ENTRY")) : "NONE")
             << " sentry_anchor_x_m=" << memory_.anchor.x << " sentry_anchor_y_m=" << memory_.anchor.y
             << " sentry_anchor_radius_m=" << anchor_radius_ << " ignored_enemy_one=" << ignore_one_
             << " outpost_min_range_m=" << outpost_band_.minimum
             << " outpost_preferred_range_m=" << outpost_band_.preferred
             << " outpost_max_range_m=" << outpost_band_.maximum << " base_min_range_m=" << base_band_.minimum
             << " base_preferred_range_m=" << base_band_.preferred << " base_max_range_m=" << base_band_.maximum
             << " attack_position_reason=" << (blocked_replanned_ ? "REPLAN_BLOCKED_APPROACH" : "WEIGHTED_GAIN");

        if (strict_order_)
            text << " mission_order="
                 << (buff_enabled_ ? campus::kBuffMissionOrder : campus::kRobotMissionOrder)
                 << " spawn_known=" << spawn_refill_.captured() << " spawn_x_m=" << spawn_refill_.point().x
                 << " spawn_y_m=" << spawn_refill_.point().y << " refill_active=" << spawn_refill_.active();
        text << " rune_max_opportunities=" << buff_limit_;
        for(const auto& e:planningEnemies()) if(e.id==5 && enemyAlive(5)) {
            auto p=projection_.fromUnity(e.unity_x_m,e.unity_z_m);
            text << " sentry_search_known=1 sentry_search_x_m=" << p.x << " sentry_search_y_m=" << p.y;
            break;
        }
        status.data = text.str();
        mission_pub_->publish(status);
    }

    void assignMissionGoal(campus::PlanPoint point, const std::string& stage) {
        // A new return trip must not inherit the previous activation's
        // watchdog. It starts only on a current unfinished request status.
        if (stage == "BUFF" && mission_stage_ != "BUFF")
            buff_started_ = false;
        goal_ = point;
        have_goal_ = true;
        blocked_replanned_ = locally_blocked_;
        locally_blocked_ = false;
        approach_progress_.reset();
        recovery_replan_ = false;
        at_attack_pose_ = false;
        attack_progress_ = SteadyClock::now();

        goal_id_ = std::max(goal_id_ + 1, static_cast<uint64_t>(now().nanoseconds()));
        goal_event_ = "AUTO_" + stage;
        RCLCPP_INFO(get_logger(), "Autonomous %s goal: %.2f,%.2f", stage.c_str(), point.x, point.y);
    }

    void tick() {
        const auto plan_time = now();
        planning_time_ = plan_time.seconds();
        double enemy_age = enemy_timeout_;

        if (have_enemies_) {
            enemy_age = std::max(silence(enemy_received_), now().seconds() - last_enemy_stamp_ * 1e-9);
            enemy_age = std::max(0.0, enemy_age);
        }

        bool enemy_fresh =
            have_enemies_ && enemy_age <= enemy_timeout_ && now().seconds() - last_enemy_stamp_ * 1e-9 >= -0.1;
        bool state_fresh = demo_ || (have_state_ && silence(state_received_) <= state_timeout_ &&
                                     now().seconds() - last_state_stamp_ <= state_timeout_);
        publishEnemies(enemy_age, enemy_fresh);
        campus::GridPlan result;

        campus::DijkstraGrid working = base_;
        std::vector<double> risk = have_map_ ? wall_cost_ : std::vector<double>{};
        if (have_map_) {
            for (const campus::BroadcastEnemy& enemy : planningEnemies()) {
                if (enemyAlive(enemy.id)) {
                    campus::PlanPoint centre = projection_.fromUnity(enemy.unity_x_m, enemy.unity_z_m);
                    double uncertain = dynamicRadius(enemy.id, enemy_age);
                    double hard = risk_mode_ ? bodyRadius(enemy.id) + own_radius_ + margin_ : uncertain;

                    // Buildings do not move: route around the same uncertainty
                    // boundary checked by the local controller. A start already
                    // in its outer margin needs an outward recovery first.
                    if (enemy.id >= 6) {
                        double physical = bodyRadius(enemy.id) + own_radius_ + margin_;
                        double distance = campus::pointDistance(start_, centre);
                        int cell = working.cellAt(start_);
                        bool raster_overlap = cell >= 0 && campus::pointDistance(working.centre(cell), centre) <=
                                                               uncertain + working.resolution * std::sqrt(2.) / 2;
                        hard = (distance > physical && raster_overlap) ? physical : uncertain;
                    }

                    working.blockCircle(centre, hard);
                    if (risk_mode_) {
                        for (std::size_t i = 0; i < risk.size(); ++i) {
                            campus::PlanPoint point = base_.centre(static_cast<int>(i));
                            double distance = std::hypot(point.x - centre.x, point.y - centre.y);

                            if (distance < uncertain) {
                                risk[i] += risk_weight_ * (1 - distance / uncertain);
                            }
                        }
                    }
                }
            }

            nav_msgs::msg::OccupancyGrid combined = map_message_;
            combined.header.stamp = now();
            combined.data = working.cells;
            grid_pub_->publish(combined);
        }

        if (state_fresh && have_start_ && own_alive_ && !manual_goal_ && !mission_paused_) {
            const bool on_top=campus::onCombatHighland(base_,highland_cells_,start_);
            if (mission_stage_ == "BUFF" && memory_.opening_done)
                rune_return_.record(start_,on_top);
            else if(mission_stage_=="SENTRY_ADVANCE" && have_goal_ && at_attack_pose_ && on_top &&
                    campus::pointDistance(start_,goal_)<=.2) {
                memory_.anchor=start_;
                memory_.sentry_advanced=true;
                have_goal_=false;
            }
            else if (mission_stage_ == "RUNE_RETURN" && rune_return_.reached(start_,on_top)) {
                rune_return_.reset();
                have_goal_=false;
            }
        }
        if (anchor_enabled_ && strict_order_ && (memory_.opening_done || !opening_enabled_) &&
            campus::fieldRobotsCleared(enemy_healths_) && enemy_healths_[4] > 0 && have_start_ && own_alive_)
        {
            if (highland_cells_.empty() || campus::onCombatHighland(working,highland_cells_,start_))
                memory_.fallback(start_);
            else if (!memory_.anchor_known) {
                auto costs=campus::routeCosts(working,start_,risk);
                double best=INFINITY;campus::PlanPoint point;
                for (auto cell:highland_cells_) if(cell>=0 && size_t(cell)<costs.size() && working.cells[cell]==0 && costs[cell]<best) {
                    best=costs[cell];point=working.centre(cell);
                }
                if(std::isfinite(best)) memory_.fallback(point);
            }
        }
        if (anchorTask() && memory_.sentry_advanced && !rune_return_.available() && !manual_goal_ && !mission_paused_ && ammo_ > 0 && !weakened_ && !spawn_refill_.active() &&
            own_health_ >= max_health_ * .3 && campus::anchorContains(start_, memory_.anchor, anchor_radius_) &&
            (highland_cells_.empty() || campus::onCombatHighland(working,highland_cells_,start_)))
            campus::restrictAnchorGrid(working, memory_.anchor, anchor_radius_);

        autonomousGoal(working, risk, have_map_ && state_fresh && enemy_fresh && own_alive_ && have_start_);
        if(runeRampTransitActive())
            campus::restrictRuneRampTransit(working,opening_cells_,player_,start_,&risk,&rune_transit_cells_);
        if (mission_stage_ == "RUNE_RETURN")
            rune_return_.restrictGrid(working);
        if (!highland_cells_.empty() &&
            ((mission_stage_=="CLEAR_ROBOTS" && (mission_target_==4 || anchorTask())) || mission_stage_=="SENTRY_ADVANCE") && !manual_goal_ && !spawn_refill_.active() &&
            campus::onCombatHighland(working,highland_cells_,start_))
            campus::restrictCombatHighland(working,highland_cells_);
        const bool local_sentry_task=mission_stage_=="SENTRY_ADVANCE" ||
            (anchorTask() && (mission_stage_=="CLEAR_ROBOTS" || mission_stage_=="WAIT_CLEAR_ROBOTS_POSITION"));
        if(local_sentry_task && sentryLocalHere()) restrictLocalSentry(working);
        if (!have_map_) {
            result.status = "WAIT_MAP";
        } else if (!state_fresh || !have_start_) {
            result.status = "WAIT_SELF_STATE";
        } else if (!demo_ && !own_alive_) {
            result.status = "SELF_DEAD_OR_MATCH_END";
        } else if (!enemy_fresh) {
            result.status = "STALE_ENEMIES";
        } else if (!have_goal_) {
            const int cell = working.cellAt(start_);
            // Autonomous candidate selection also fails when its start is
            // blocked. Report the actual cause rather than inviting another
            // goal selection at an unusable start.
            result.status = autonomous_ && !manual_goal_ && !mission_paused_ && (cell < 0 || working.cells[cell] != 0)
                                ? "START_BLOCKED"
                                : "WAIT_GOAL";
        } else {
            result = runeRampTransitActive()?campus::planRuneRampTransit(working,start_,goal_,player_,risk)
                                            : working.plan(start_, goal_, risk);
        }

        bool retained_route = false;
        if (route_goal_id_ == goal_id_)
            retained_route = campus::retainGridRoute(working, risk, route_cells_, result);
        route_cells_ = result.cells;
        route_goal_id_ = goal_id_;
        std::string start_block = have_start_ ? campus::pointBlockReason(base_, working, start_) : "NOT_SET";
        if(result.status=="START_BLOCKED" && start_block=="ENEMY_REGION") {
            std::ostringstream overlaps;
            for(const auto& enemy:planningEnemies()) {
                if(!enemyAlive(enemy.id))continue;
                auto p=projection_.fromUnity(enemy.unity_x_m,enemy.unity_z_m);
                double hard=risk_mode_?bodyRadius(enemy.id)+own_radius_+margin_:dynamicRadius(enemy.id,enemy_age);
                if(campus::pointDistance(start_,p)<=hard+working.resolution*std::sqrt(2.))
                    overlaps<<" id="<<enemy.id<<" centre="<<p.x<<','<<p.y<<" distance="<<campus::pointDistance(start_,p)
                            <<" hard="<<hard<<" age="<<enemy_age;
            }
            RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"START_ENEMY_OVERLAP own=(%.3f,%.3f)%s",
                start_.x,start_.y,overlaps.str().c_str());
        }
        std::string goal_block = have_goal_ ? campus::pointBlockReason(base_, working, goal_) : "NOT_SET";
        std::string no_path_reason = result.status == "NO_PATH" ? noPathReason() : "NONE";
        publishPoints(state_fresh, result.status, start_block, goal_block, no_path_reason);

        nav_msgs::msg::Path path;
        path.header.frame_id = frame_;
        // Radius ages were computed at tick start. Keep that timestamp so the
        // controller also accounts for time spent constructing this snapshot.
        path.header.stamp = plan_time;

        for (int cell : result.cells) {
            campus::PlanPoint point = working.centre(cell);
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = 0.12;
            pose.pose.orientation.w = 1;
            path.poses.push_back(pose);
        }

        path_pub_->publish(path);
        // The display must describe the same current route as the controller,
        // including return missions and invalid replans; never retain a stale goal.
        history_ = path;
        history_pub_->publish(history_);

        // 控制器只消费这个原子快照，不能拼接不同周期的 Path 与障碍图。
        campus_aim_interfaces::msg::NavigationPlan snapshot;
        snapshot.header = path.header;
        snapshot.goal_id = goal_id_;
        snapshot.valid = result.status == "OK";
        snapshot.status = result.status;
        snapshot.path = path;
        snapshot.static_grid = map_message_;
        snapshot.static_grid.header = path.header;
        if (mission_stage_ == "RUNE_RETURN" || runeRampTransitActive()) {
            auto corridor = base_;
            if(mission_stage_=="RUNE_RETURN") rune_return_.restrictGrid(corridor);
            if(runeRampTransitActive()) campus::restrictRuneRampTransit(corridor,opening_cells_,player_,start_,nullptr,&rune_transit_cells_);
            snapshot.static_grid.data = corridor.cells;
        }

        if(local_sentry_task && sentryLocalHere()) {
            auto local=base_;restrictLocalSentry(local);snapshot.static_grid.data=local.cells;
        }
        snapshot.goal.x = goal_.x;
        snapshot.goal.y = goal_.y;
        snapshot.enemy_stamp = rclcpp::Time(static_cast<int64_t>(last_enemy_stamp_));
        snapshot.enemy_speed_bound_mps = speed_bound_;
        snapshot.unity_to_map = projection_.matrix;

        for (const campus::BroadcastEnemy& enemy : planningEnemies()) {
            if (!enemyAlive(enemy.id)) {
                continue;
            }

            campus::PlanPoint point = projection_.fromUnity(enemy.unity_x_m, enemy.unity_z_m);
            geometry_msgs::msg::Point centre;
            centre.x = point.x;
            centre.y = point.y;
            snapshot.enemy_centres.push_back(centre);
            // 全局额外预留的未来余量不重复传入局部控制器；局部按候选轨迹时间逐步扩展。
            double radius = dynamicRadius(enemy.id, enemy_age);

            if (enemy.id <= 5) {
                radius -= speed_bound_ * forecast_time_;
            }

            snapshot.enemy_radii.push_back(radius);
            snapshot.enemy_hard_radii.push_back(bodyRadius(enemy.id) + own_radius_ + margin_);
            snapshot.enemy_mobile.push_back(enemy.id <= 5);
        }

        plan_pub_->publish(snapshot);

        std_msgs::msg::String status;
        std::ostringstream text;
        text << result.status << " mode=" << (demo_ ? "DEMO" : "LIVE") << " enemy_age_s=" << enemy_age
             << " length_m=" << result.length_m << " expanded=" << result.expanded
             << " weighted_cost=" << result.total_cost << " retained_route=" << retained_route
             << " REFERENCE_PROJECTION_UNVERIFIED" << (display_only_ ? " DISPLAY_ONLY" : " FOLLOWER_CONNECTED");
        text << (risk_mode_ ? " UNCERTAINTY_IS_COST_LOCAL_CONTROL_CHECKS_SAFETY" : " UNCERTAINTY_HARD_BLOCK")
             << (aligned_ ? " EXPLICIT_VIEW_ALIGNMENT" : " DEFAULT_VIEW_PROJECTION");
        text << " start_xy=" << start_.x << ',' << start_.y << " start_block=" << start_block << " goal_xy=" << goal_.x
             << ',' << goal_.y << " goal_block=" << goal_block << " path_poses=" << path.poses.size()
             << " no_path_reason=" << no_path_reason;
        text << " goal_received=" << goals_received_ << " goal_event=" << goal_event_
             << " goal_topic=" << goal_sub_->get_topic_name();
        status.data = text.str();
        status_pub_->publish(status);

        // 失败原因发生变化时直接打印，不需要用户去猜 RViz 小字。
        std::string diagnostic = result.status + '/' + start_block + '/' + goal_block + '/' + no_path_reason;
        if (diagnostic != last_diagnostic_ || goal_id_ != last_diagnostic_goal_) {
            RCLCPP_INFO(get_logger(), "%s", status.data.c_str());
            last_diagnostic_ = diagnostic;
            last_diagnostic_goal_ = goal_id_;
        }
    }

    std::string frame_, enemy_frame_, enemy_session_, state_frame_, state_session_;
    int player_ = 1;
    bool demo_ = false, use_healths_ = true;
    bool display_only_ = true;
    bool allow_alignment_ = false, aligned_ = false, risk_mode_ = false;
    bool four_station_enabled_=false, four_station_active_=false;
    campus::PlanPoint four_station_;
    bool autonomous_ = false, mission_paused_ = false, manual_goal_ = false, weakened_ = false, ignore_one_ = false;
    bool combat_holding_ = false;
    int buff_limit_=5;
    bool buff_enabled_ = false, buff_done_ = false, buff_started_ = false, buff_exhausted_ = false,
         buff_effect_active_ = false;
    unsigned buff_count_ = 0, buff_rejections_ = 0;
    uint64_t buff_min_sequence_ = 0;
    SteadyClock::time_point buff_finished_;
    SteadyClock::time_point buff_since_;
    rclcpp::Subscription<campus_aim_interfaces::msg::BuffStatus>::SharedPtr buff_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr buff_start_sub_;
    SteadyClock::time_point combat_received_;
    campus::StandoffBand outpost_band_, base_band_{1.8, 2.5, 3.2};

    bool tactical_ = true, at_attack_pose_ = false;
    bool opening_enabled_ = false, sentry_last_ = false, anchor_enabled_ = false;
    double anchor_radius_ = 2.;
    std::vector<int64_t> highland_cells_;
    std::vector<int64_t> opening_cells_, rune_transit_cells_;
    campus::OpeningAndAnchor memory_;
    bool field_order_ = false;
    bool strict_order_ = false;
    std::string ordered_phase_;
    campus::SpawnRefill spawn_refill_;

    bool locally_blocked_ = false, blocked_replanned_ = false, recovery_replan_ = false;
    int blocked_replans_ = 0;
    campus::PlanPoint blocked_origin_{INFINITY, INFINITY};
    campus::TacticalGains gains_;
    int mission_target_ = 0, own_health_ = 0, max_health_ = 400;
    double selected_gain_ = 0, selected_cost_ = 0, selected_score_ = 0;
    std::vector<double> gain_points_, gain_values_, gain_used_until_;
    campus::PlanPoint avoid_goal_{INFINITY, INFINITY};

    SteadyClock::time_point attack_progress_;
    campus::ApproachProgress approach_progress_;
    std::vector<double> refill_points_;
    int ammo_ = 0, capacity_ = 300;
    size_t refill_index_ = 0;
    std::string mission_stage_;
    SteadyClock::time_point refill_since_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_pub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr follow_sub_;
    double risk_weight_ = 8;
    double wall_distance_ = .6, wall_weight_ = 6;
    std::vector<double> wall_cost_;
    std::vector<int> route_cells_;
    uint64_t route_goal_id_ = 0;

    uint64_t goals_received_ = 0;
    std::string goal_event_ = "NEVER_RECEIVED";
    nav_msgs::msg::Path history_;
    bool have_map_ = false, have_start_ = false, have_goal_ = false, have_state_ = false;
    bool have_raw_map_ = false;
    bool have_enemies_ = false, own_alive_ = false;
    double enemy_timeout_, state_timeout_, replan_hz_, enemy_radius_, own_radius_, margin_, uncertainty_, speed_bound_;
    double outpost_radius_, base_radius_;

    double forecast_time_ = 0;
    double last_state_stamp_ = 0, last_remaining_ = 0;
    uint64_t last_sequence_ = 0, last_enemy_stamp_ = 0, enemy_min_stamp_ = 0;
    uint64_t goal_id_ = 0;
    uint64_t last_diagnostic_goal_ = 0;
    std::string last_diagnostic_;
    SteadyClock::time_point enemy_received_, state_received_;
    std::array<int32_t, 7> enemy_healths_{};

    campus::DijkstraGrid base_, raw_;
    campus::PlanningProjection projection_;
    std::vector<double> initial_projection_;
    campus::BroadcastFields fields_;
    campus::PlanPoint start_, goal_;
    std::vector<campus::BroadcastEnemy> enemies_;
    std::array<campus::BroadcastEnemy, 5> visual_enemies_;
    std::array<double, 5> visual_enemy_stamps_{};

    double planning_time_ = 0;
    rclcpp::Subscription<campus_aim_interfaces::msg::TrackFrame>::SharedPtr tracks_sub_;
    nav_msgs::msg::OccupancyGrid map_message_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, history_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr enemy_pub_, point_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<campus_aim_interfaces::msg::NavigationPlan>::SharedPtr plan_pub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_, raw_map_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr enemy_sub_;
    rclcpp::Subscription<tdt_interface::msg::AutoAimState>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        std::shared_ptr<DijkstraPlanner> node = std::make_shared<DijkstraPlanner>();
        rclcpp::spin(node);
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("dijkstra_planner"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
