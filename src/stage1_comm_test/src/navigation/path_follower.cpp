#include "navigation/path_following.hpp"
#include "navigation/travel_facing.hpp"
#include "navigation/combat_policy.hpp"
#include "navigation/motion_recovery.hpp"
#include "navigation/assault_mission.hpp"
#include "navigation/state_session.hpp"
#include "navigation/mission_order.hpp"
#include "navigation/snapshot_validation.hpp"

#include <campus_aim_interfaces/msg/navigation_plan.hpp>
#include <campus_aim_interfaces/msg/track_frame.hpp>
#include <campus_aim_interfaces/msg/armor_frame.hpp>
#include <tdt_interface/msg/auto_aim_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <array>
#include <csignal>
#include <functional>
#include <memory>
#include <sstream>
#include <thread>

static volatile std::sig_atomic_t stop_requested = 0;

static void stopSignal(int) {
    stop_requested = 1;
}

class PathFollowerNode : public rclcpp::Node {
  public:
    PathFollowerNode() : Node("path_follower") {
        simulation_ = declare_parameter<bool>("simulation", false);
        enabled_ = declare_parameter<bool>("command.enabled", false);
        verified_ = declare_parameter<bool>("map_alignment_verified", false);
        training_prior_ = declare_parameter<bool>("command.training_prior", false);
        landmark_checked_ = declare_parameter<bool>("command.landmark_alignment_checked", false);
        follow_projection_ = declare_parameter<bool>("follow_planner_projection", false);
        combat_enabled_ = declare_parameter<bool>("combat.enabled", false);
        structures_ = declare_parameter<bool>("combat.structures", false);

        acquire_unknown_ = declare_parameter<bool>("combat.acquire_unknown", false);
        route_decision_ = declare_parameter<bool>("combat.route_decision", false);
        after_arrival_ = declare_parameter<bool>("combat.after_arrival", false);
        strict_order_ = declare_parameter<bool>("mission.strict_order", false);
        field_order_ = declare_parameter<bool>("mission.field_robot_order", false);

        highland_cells_=declare_parameter<std::vector<int64_t>>("combat.highland_cells",std::vector<int64_t>{});
        std::sort(highland_cells_.begin(),highland_cells_.end());
        ramp_cells_ = declare_parameter<std::vector<int64_t>>("terrain.ramp_cells", std::vector<int64_t>());
        ramp_speed_ = declare_parameter<double>("terrain.max_speed_mps", .35);
        if (!std::isfinite(ramp_speed_) || ramp_speed_ <= 0 || ramp_speed_ > 4.5)
            throw std::invalid_argument("Invalid terrain speed limit");
        ramp_lookahead_ = declare_parameter<double>("terrain.lookahead_m", .4);

        if (!std::isfinite(ramp_lookahead_) || ramp_lookahead_ < .3 || ramp_lookahead_ > 2.0)
            throw std::invalid_argument("Invalid terrain lookahead");

        player_ = declare_parameter<int>("player_id", 1);
        frame_ = declare_parameter<std::string>("map_frame", "arena_reference");
        scale_ = declare_parameter<double>("scene_units_per_m", 3.0);
        projection_.matrix = declare_parameter<std::vector<double>>("unity_to_reference", std::vector<double>());
        state_timeout_ = declare_parameter<double>("state_timeout_s", 0.2);
        plan_timeout_ = declare_parameter<double>("plan_timeout_s", 0.45);
        enemy_timeout_ = declare_parameter<double>("enemy_timeout_s", 2.5);
        blocked_timeout_ = declare_parameter<double>("blocked_timeout_s", 6.0);

        double rate = declare_parameter<double>("control_rate_hz", 50.0);

        campus::FollowerConfig config;
        config.feedback_timeout = state_timeout_;
        config.max_speed = declare_parameter<double>("max_speed_mps", config.max_speed);
        config.acceleration = declare_parameter<double>("max_acceleration_mps2", config.acceleration);
        config.braking = declare_parameter<double>("braking_acceleration_mps2", config.braking);
        config.lateral_acceleration =
            declare_parameter<double>("max_lateral_acceleration_mps2", config.lateral_acceleration);
        config.lookahead = declare_parameter<double>("lookahead_m", config.lookahead);
        config.horizon = declare_parameter<double>("collision_horizon_s", config.horizon);
        config.tolerance = declare_parameter<double>("position_tolerance_m", config.tolerance);
        config.release_tolerance = declare_parameter<double>("release_tolerance_m", config.release_tolerance);
        config.stop_speed = declare_parameter<double>("stop_speed_mps", config.stop_speed);
        config.settle_time = declare_parameter<double>("settle_time_s", config.settle_time);
        config.path_deviation = declare_parameter<double>("max_path_deviation_m", config.path_deviation);
        config.response_time = declare_parameter<double>("drive_response_time_s", config.response_time);

        config.tracking_reserve = declare_parameter<double>("tracking_reserve_m", .03);
        tracking_reserve_ = config.tracking_reserve;
        enemy_recovery_margin_ = declare_parameter<double>("enemy_recovery_margin_m", .15);
        if(!std::isfinite(enemy_recovery_margin_) || enemy_recovery_margin_<0 || enemy_recovery_margin_>.3)
            throw std::invalid_argument("Invalid enemy recovery margin");
        release_tolerance_ = config.release_tolerance;
        max_speed_ = config.max_speed;
        follower_ = campus::PathFollower(config);

        auto evade_config = config;

        evade_config.horizon = std::min(config.horizon, .25);
        evader_ = campus::PathFollower(evade_config);

        auto recovery_config = evade_config;
        recovery_config.max_speed = .25;
        recovery_config.tolerance = .03;
        recovery_config.release_tolerance = .06;
        recovery_config.settle_time = .1;
        recovery_controller_ = campus::PathFollower(recovery_config);

        double values[] = {scale_, state_timeout_, plan_timeout_, enemy_timeout_, blocked_timeout_, rate};
        for (double value : values) {
            if (!std::isfinite(value) || value <= 0) {
                throw std::invalid_argument("Invalid follower rates/timeouts");
            }
        }

        if (!projection_.valid() || frame_ != "arena_reference" || (player_ != 1 && player_ != 2) ||
            state_timeout_ > 0.3 || plan_timeout_ > 0.5 || enemy_timeout_ > 3 || rate < 20 || rate > 100) {
            throw std::invalid_argument("Invalid follower frame/projection or watchdog limits");
        }

        // 参考图米与世界米必须近似等尺度，不能使用退化/任意缩放的查看投影控制底盘。
        const std::vector<double>& matrix = projection_.matrix;
        if (std::abs(std::hypot(matrix[0], matrix[3]) - 1) > 0.05 ||
            std::abs(std::hypot(matrix[1], matrix[4]) - 1) > 0.05 ||
            std::abs(matrix[0] * matrix[1] + matrix[3] * matrix[4]) > 0.05) {
            throw std::invalid_argument("Follower projection must preserve metre scale and orthogonal axes");
        }

        if (training_prior_ &&
            (simulation_ || !enabled_ || verified_ || !landmark_checked_ || follow_projection_ ||
             config.max_speed > 4.5 + 1e-9 || config.acceleration > 8.5 + 1e-9 ||
             config.lateral_acceleration > 2.8 + 1e-9 || config.braking < 0.7 || std::abs(scale_ - 3) > 1e-9 ||
             std::abs(matrix[0] * matrix[4] - matrix[1] * matrix[3] - 1) > 1e-6 ||
             std::abs(std::hypot(matrix[0], matrix[3]) - 1) > 1e-6 ||
             std::abs(std::hypot(matrix[1], matrix[4]) - 1) > 1e-6 ||
             std::abs(matrix[0] * matrix[1] + matrix[3] * matrix[4]) > 1e-6)) {
            throw std::invalid_argument("Unity prior-map training requires checked fixed landmarks, <=4.5 m/s, <=8.5 "
                                        "m/s^2, and unchanged watchdogs");
        }

        if (enabled_ && !simulation_ && !verified_ && !training_prior_) {
            throw std::invalid_argument("Reference map alignment is unverified: use simulation or preview; verify "
                                        "geometry before enabling real motion");
        }

        if (follow_projection_ && enabled_ && !simulation_) {
            throw std::invalid_argument(
                "Interactive reference alignment is preview only; real motion requires a fixed checked transform");
        }

        rclcpp::QoS cached(1);
        cached.reliable().transient_local();
        rclcpp::QoS latest(1);
        latest.reliable();

        // 仿真速度只进入自己的运动模型；预览模式不创建比赛速度发布器。
        if (simulation_ || enabled_) {
            std::string topic =
                simulation_ ? "/navigation/sim_command" : "/target_speed_player_" + std::to_string(player_);
            command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(topic, latest);
        }

        preview_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/navigation/preview_command", latest);
        status_pub_ = create_publisher<std_msgs::msg::String>("/navigation/follow_status", cached);
        local_pub_ = create_publisher<nav_msgs::msg::Path>("/navigation/local_path", cached);
        markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/navigation/robot_markers", cached);

        plan_sub_ = create_subscription<campus_aim_interfaces::msg::NavigationPlan>(
            "/planning/snapshot", cached, std::bind(&PathFollowerNode::planCallback, this, std::placeholders::_1));
        state_sub_ = create_subscription<tdt_interface::msg::AutoAimState>(
            "/autoaim/state", latest, std::bind(&PathFollowerNode::stateCallback, this, std::placeholders::_1));
        cancel_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/planning/cancel", latest, std::bind(&PathFollowerNode::cancelCallback, this, std::placeholders::_1));

        if (combat_enabled_) {
            tracks_sub_ = create_subscription<campus_aim_interfaces::msg::TrackFrame>(
                "/tracker/targets", latest, [this](campus_aim_interfaces::msg::TrackFrame::ConstSharedPtr m) {
                    if (m->reset) {
                        combat_.reset();
                        vision_status_ = "RESET";
                        return;
                    }

                    double a = age(m->frame.header.stamp);

                    int64_t stamp = rclcpp::Time(m->frame.header.stamp).nanoseconds();
                    if (!have_state_) {
                        vision_status_ = "WAIT_STATE";
                        return;
                    }

                    if (stamp <= tracks_stamp_ || stamp < vision_minimum_stamp_)
                        return;
                    if (!std::isfinite(a) || a < -0.1 || a > 0.2 || stamp < state_stamp_ - 200000000LL) {
                        vision_status_ = "STALE_TRACK_FRAME";
                        return;
                    }

                    if (campus::navigationStateSession(m->frame.state) != session_) {
                        vision_status_ = "STATE_SESSION_MISMATCH";
                        return;
                    }

                    tracks_stamp_ = stamp;
                    vision_status_ = m->targets.empty() ? "NO_TRACK" : "NO_FRESH_ARMOR";
                    if ((after_arrival_ && !arrived_for_goal_) || openingTask() || !anchorReady()) {
                        combat_decision_ = "WAIT_ARRIVAL";
                        return;
                    }

                    for (const auto& t : m->targets) {
                        if (strict_order_ && !campus::missionArmorAlive(campus::armorSelectionStage(mission_stage_),
                                                                        t.number, structure_healths_, ignore_one_, field_order_)) {
                            combat_decision_ = "MISSION_PRIORITY";
                            continue;
                        }

                        const bool coast = t.track_id == combat_.target_id && t.state == "TEMP_LOST" &&
                                           std::isfinite(t.last_observation_age) && t.last_observation_age >= 0 &&
                                           t.last_observation_age <= .2;
                        if (!std::isfinite(t.last_observation_age) || a + t.last_observation_age > .2)
                            continue;
                        if ((!coast && !campus::combatObservation(t.number, t.state, t.last_observation_age,
                                                                  structures_, acquire_unknown_)) ||
                            (t.number == 6 && structure_healths_[5] <= 0) ||
                            (t.number == 8 && (structure_healths_[5] > 0 || structure_healths_[6] <= 0)) ||
                            !finite(t.center.x, t.center.y))
                            continue;
                        auto enemy_point = projection_.fromUnity(-t.center.y, t.center.x);

                        if (route_decision_ && t.number != 6 && t.number != 8 &&
                            !campus::routeThreat(position_, enemy_point, path_)) {
                            combat_decision_ = "OUTSIDE_ROUTE";
                            continue;
                        }

                        combat_decision_ = t.number == 6 || t.number == 8 ? "MISSION_STRUCTURE" : "ROUTE_THREAT";

                        if (!coast)
                            combat_.see(t.track_id, enemy_point, steadyTime() - a - t.last_observation_age,
                                        t.state == "TRACKING", t.number != 6 && t.number != 8);

                        if (repositioning_ && combat_number_ == 4 && t.number != 4) {
                            repositioning_ = false;
                            evader_.reset();
                        }
                        combat_number_ = t.number;
                        vision_status_ = t.state;
                        break;
                    }
                });
        }

        armor_view_sub_ = create_subscription<campus_aim_interfaces::msg::ArmorFrame>(
            "/armor/observations", rclcpp::QoS(1).reliable(), [this](campus_aim_interfaces::msg::ArmorFrame::ConstSharedPtr m) {
                const auto& f=m->frame;
                double stamp=rclcpp::Time(f.state.header.stamp).seconds();
                if(m->reset) {four_view_.reset();return;}
                if(!have_state_ || campus::navigationStateSession(f.state)!=session_ ||
                   rclcpp::Time(f.state.header.stamp).nanoseconds()<vision_minimum_stamp_ ||
                   now().seconds()-stamp<-.1 || now().seconds()-stamp>.2) return;
                bool complete=false,maintained=false;
                for(const auto& a:m->armors)
                    if(a.number==4 && a.confidence>=.65 && std::isfinite(a.reprojection_error) && a.reprojection_error<=4.) {
                        complete=complete || campus::completeArmorView(a.image_points,f.camera_info.width,f.camera_info.height);
                        maintained=maintained || campus::completeArmorView(a.image_points,f.camera_info.width,f.camera_info.height,true);
                    }
                four_view_.observe(stamp,steadyTime(),complete,maintained);
            });

        mission_sub_ = create_subscription<std_msgs::msg::String>(
            "/mission/status", cached, [this](std_msgs::msg::String::ConstSharedPtr m) {
                auto stage = m->data.substr(0, m->data.find(' '));
                if (strict_order_ && (stage != mission_stage_ || (m->data.find(" sentry_anchor_active=1") !=
                                                                  std::string::npos) != anchor_active_)) {
                    combat_.reset();
                    motion_recovery_.reset();
                    recovery_goal_.reset();
                    stop_progress_.reset();
                }

                sentry_local_required_=m->data.find(" sentry_local_required=1")!=std::string::npos;
                sentry_local_here_=m->data.find(" sentry_local_here=1")!=std::string::npos;
                highland_active_=m->data.find(" robot_highland_required=1")!=std::string::npos;
                anchor_active_ = m->data.find(" sentry_anchor_active=1") != std::string::npos;
                anchor_known_ = false;
                anchor_point_ = {NAN, NAN};
                mission_stage_ = stage;
                four_station_fixed_ = m->data.find(" four_station_fixed=1") != std::string::npos;
                refill_ = stage == "REFILL";
                ignore_one_ = m->data.find(" ignored_enemy_one=1") != std::string::npos;
                mission_received_ = Clock::now();
                outpost_known_ = false;
                mission_target_number_ = 0;

                outpost_point_ = {NAN, NAN};
                std::istringstream fields(m->data);
                std::string token;

                while (fields >> token)
                    try {
                        if (token == "sentry_anchor_known=1")
                            anchor_known_ = true;
                        if (token.rfind("sentry_anchor_x_m=", 0) == 0)
                            anchor_point_.x = std::stod(token.substr(std::string("sentry_anchor_x_m=").size()));
                        if (token.rfind("sentry_anchor_y_m=", 0) == 0)
                            anchor_point_.y = std::stod(token.substr(std::string("sentry_anchor_y_m=").size()));

                        if (token.rfind("sentry_anchor_radius_m=", 0) == 0)
                            anchor_radius_ = std::stod(token.substr(std::string("sentry_anchor_radius_m=").size()));
                        if (token.rfind("target_class=", 0) == 0)
                            mission_target_number_ = std::stoi(token.substr(13));
                        if (token == "target_known=1")
                            outpost_known_ = true;
                        if (token.rfind("target_x_m=", 0) == 0)
                            outpost_point_.x = std::stod(token.substr(11));

                        if (token.rfind("target_y_m=", 0) == 0)
                            outpost_point_.y = std::stod(token.substr(11));
                        if (token.rfind("base_min_range_m=", 0) == 0)
                            base_band_.minimum = std::stod(token.substr(17));
                        if (token.rfind("base_preferred_range_m=", 0) == 0)
                            base_band_.preferred = std::stod(token.substr(23));

                        if (token.rfind("base_max_range_m=", 0) == 0)
                            base_band_.maximum = std::stod(token.substr(17));
                        if (token.rfind("outpost_min_range_m=", 0) == 0)
                            outpost_band_.minimum = std::stod(token.substr(20));
                        if (token.rfind("outpost_preferred_range_m=", 0) == 0)
                            outpost_band_.preferred = std::stod(token.substr(26));

                        if (token.rfind("outpost_max_range_m=", 0) == 0)
                            outpost_band_.maximum = std::stod(token.substr(20));
                    } catch (const std::exception&) {
                        outpost_known_ = false;
                    }

                if (strict_order_) {
                    auto field = m->data.find(" goal=");
                    try {
                        mission_goal_id_ = field == std::string::npos ? 0 : std::stoull(m->data.substr(field + 6));
                    } catch (const std::exception&) {
                        mission_goal_id_ = 0;
                    }
                }
            });

        last_tick_ = Clock::now();
        started_ns_ = now().nanoseconds();
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), std::bind(&PathFollowerNode::tick, this));
        RCLCPP_INFO(get_logger(), "Path following: %s, player=%d, %.2f m/s; select only a goal",
                    simulation_ ? "PRIVATE SIMULATION" : (enabled_ ? "REAL COMMANDS" : "PREVIEW"), player_,
                    config.max_speed);
    }

    void stopForShutdown() {
        publishCommand({});
    }

  private:
    typedef std::chrono::steady_clock Clock;
    double steadyTime() const {
        return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
    }

    double silence(Clock::time_point received) const {
        return std::chrono::duration<double>(Clock::now() - received).count();
    }

    double age(const builtin_interfaces::msg::Time& stamp) const {
        return now().seconds() - rclcpp::Time(stamp).seconds();
    }

    bool openingTask() const {
        return mission_stage_ == "OPENING_RAMP" || mission_stage_ == "WAIT_OPENING_RAMP" || mission_stage_ == "RUNE_RETURN" || mission_stage_ == "SENTRY_ADVANCE";
    }

    bool sentry_local_required_=false,sentry_local_here_=false;
    bool anchorReady() const {
        return (!sentry_local_required_ || sentry_local_here_) && (!highland_active_ || campus::onCombatHighland(grid_,highland_cells_,position_)) && (!anchor_active_ ||
               (anchor_known_ && silence(mission_received_) <= .5 && mission_goal_id_ == plan_.goal_id &&
                campus::anchorContains(position_, anchor_point_, anchor_radius_)));
    }

    bool structureReady() const {
        if (!strict_order_ || (mission_stage_ != "OUTPOST" && mission_stage_ != "BASE"))
            return true;
        return arrived_for_goal_ && silence(mission_received_) <= .5 && mission_goal_id_ == plan_.goal_id &&
               outpost_known_ &&
               (mission_stage_ == "BASE" ? base_band_ : outpost_band_)
                   .contains(campus::pointDistance(position_, outpost_point_));
    }

    bool finite(double x, double y) const {
        return std::isfinite(x) && std::isfinite(y);
    }

    void cancelTask(bool preserve_contact = false) {
        cancelled_id_ = std::max(cancelled_id_, plan_.goal_id);
        minimum_plan_ns_ = now().nanoseconds();
        follower_.reset();
        evader_.reset();
        recovery_controller_.reset();
        stalled_ = 0;

        if (preserve_contact)
            combat_.pause();
        else {
            combat_.reset();
            motion_recovery_.reset();
            recovery_goal_.reset();
        }

        combat_progress_.reset();
        if(!preserve_contact)four_view_.reset();
        four_retry_path_.clear();
        four_no_fire_.finish();
        if(!preserve_contact)four_hold_.reset();
        repositioning_ = false;
        if (!preserve_contact)
            stop_progress_.reset();
        arrived_for_goal_ = false;
        publishCommand({});
    }

    void cancelCallback(std_msgs::msg::Bool::ConstSharedPtr message) {
        if (message->data) {
            cancelTask();
        }
    }

    void stateCallback(tdt_interface::msg::AutoAimState::ConstSharedPtr message) {
        double message_age = age(message->header.stamp);

        int64_t stamp = rclcpp::Time(message->header.stamp).nanoseconds();
        if (stamp <= 0 || !std::isfinite(message_age) || message_age < -0.1 || message_age > state_timeout_ ||
            !finite(message->position.x, message->position.y) ||
            !finite(message->twist.linear.x, message->twist.linear.y) || !std::isfinite(message->remaining_seconds)) {
            have_state_ = false;
            cancelTask();
            return;
        }

        if (stamp <= state_stamp_) {
            return;
        }

        std::string session = campus::navigationStateSession(*message);
        bool session_reset = state_stamp_ > 0 && (message->frame_sequence < sequence_ || session != session_ ||
                                                 message->remaining_seconds > remaining_seconds_ + 2.);
        bool reset = have_state_ && (session_reset || silence(state_received_) > state_timeout_);

        if (state_stamp_ > 0 && !session_reset && message->frame_sequence == sequence_ && session == session_) {
            return;
        }

        if (reset || session_reset || message->health <= 0 || message->remaining_seconds <= 0) {
            cancelTask(reset && !session_reset && message->health > 0 && message->remaining_seconds > 0);
        }
        if(session_reset) {
            four_no_fire_.reset();
            // Old images can still be in flight after the clock restarts.
            vision_minimum_stamp_=stamp;
            combat_number_=0;
            tracks_stamp_=0;
            vision_status_="NEW_MATCH";
        }

        if (combat_enabled_ && have_state_ && !reset && alive_ && message->health > 0 && message->health < health_)
            combat_.damaged(steadyTime(), position_, velocity_);
        health_ = message->health;
        ammo_remaining_ = message->ammo_remaining;
        if (message->health <= 0) four_no_fire_.reset();
        needs_refill_ = strict_order_ && (message->ammo_remaining <= 0 || message->is_weakened);

        if (needs_refill_) {
            combat_.reset();
            combat_progress_.reset();
            repositioning_ = false;
        }

        structure_healths_ = message->enemy_healths;
        int target_health_index = campus::armorHealthIndex(combat_number_);
        if (strict_order_ && target_health_index >= 0 && structure_healths_[target_health_index] <= 0) {
            combat_.reset();
            combat_progress_.reset();
            repositioning_ = false;
        }

        feedback_dt_ = state_stamp_ > 0 ? std::clamp((stamp - state_stamp_) * 1e-9, 0.0, state_timeout_) : 0;
        state_stamp_ = stamp;
        sequence_ = message->frame_sequence;
        remaining_seconds_ = message->remaining_seconds;
        travel_yaw_ = message->yaw_degrees;
        session_ = session;
        position_ = projection_.fromUnity(-message->position.y / scale_, message->position.x / scale_);
        double vx = -message->twist.linear.y / scale_;
        double vz = message->twist.linear.x / scale_;
        velocity_ = {projection_.matrix[0] * vx + projection_.matrix[1] * vz,
                     projection_.matrix[3] * vx + projection_.matrix[4] * vz};
        alive_ = message->health > 0 && message->remaining_seconds > 0;
        have_state_ = finite(position_.x, position_.y) && finite(velocity_.x, velocity_.y) &&
                      std::hypot(velocity_.x, velocity_.y) <= 5;
        state_received_ = Clock::now();

        if (!have_state_) {
            cancelTask();
        }
    }

    void planCallback(campus_aim_interfaces::msg::NavigationPlan::ConstSharedPtr message) {
        int64_t stamp = rclcpp::Time(message->header.stamp).nanoseconds();
        double message_age = age(message->header.stamp);
        if (message->header.frame_id != frame_ || stamp < started_ns_ || stamp < minimum_plan_ns_ ||
            stamp <= plan_stamp_ || !std::isfinite(message_age) || message_age < -0.1 || message_age > plan_timeout_) {
            return;
        }

        if (follow_projection_) {
            campus::PlanningProjection next;
            next.matrix = message->unity_to_map;
            if (!next.valid() || frame_ != "arena_reference" ||
                std::abs(next.matrix[0] - projection_.matrix[0]) > 1e-9 ||
                std::abs(next.matrix[1] - projection_.matrix[1]) > 1e-9 ||
                std::abs(next.matrix[3] - projection_.matrix[3]) > 1e-9 ||
                std::abs(next.matrix[4] - projection_.matrix[4]) > 1e-9) {
                cancelTask();
                have_plan_ = false;
                return;
            }

            // 让已缓存反馈位置与同一快照的查看平移一致；速度没有平移项。
            position_.x += next.matrix[2] - projection_.matrix[2];
            position_.y += next.matrix[5] - projection_.matrix[5];
            projection_ = next;
        } else if (message->unity_to_map != projection_.matrix) {
            // A fixed controller must not consume a route generated in a
            // different projection, even if both messages name the same frame.
            have_plan_ = false;
            cancelTask();
            return;
        }

        if (message->goal_id != plan_.goal_id) {
            // A moving enemy changes its attack pose frequently. Restarting
            // the acceleration ramp at zero each time repeatedly brakes a
            // moving chassis. Seed from fresh physical feedback; rollout still
            // checks the new direction and the complete stop against obstacles.
            follower_.reset(have_state_ && silence(state_received_) <= state_timeout_ ? velocity_
                                                                                      : campus::PlanPoint{});
            arrived_for_goal_ = false;
            if (after_arrival_) {
                combat_.reset();
                combat_decision_ = "WAIT_ARRIVAL";
            }

            stalled_ = 0;
            progress_position_ = position_;
        }

        plan_ = *message;
        snapshot_reason_ = campus::enemySnapshotReason(*message);
        plan_stamp_ = stamp;
        plan_received_ = Clock::now();
        have_plan_ = true;

        // A blocked destination need not block a local combat escape. Validate
        // its static map and enemy snapshot independently from the global path.
        grid_.width = static_cast<int>(message->static_grid.info.width);
        grid_.height = static_cast<int>(message->static_grid.info.height);
        grid_.resolution = message->static_grid.info.resolution;
        grid_.origin_x = message->static_grid.info.origin.position.x;
        grid_.origin_y = message->static_grid.info.origin.position.y;
        grid_.cells = message->static_grid.data;
        const auto& q = message->static_grid.info.origin.orientation;

        local_snapshot_valid_ = grid_.valid() && message->static_grid.header.frame_id == frame_ &&
                                rclcpp::Time(message->static_grid.header.stamp).nanoseconds() == stamp &&
                                std::isfinite(q.w) && std::abs(std::abs(q.w) - 1) < 1e-6 && q.x == 0 && q.y == 0 &&
                                q.z == 0 && snapshot_reason_.empty();
        if (!message->valid) {
            bool combat_escape = combat_enabled_ && combat_.phase(steadyTime()) == "EVADE" && local_snapshot_valid_ &&
                                 (message->status == "NO_PATH" || message->status == "GOAL_BLOCKED" ||
                                  message->status == "START_BLOCKED");
            const bool margin_escape = message->status == "START_BLOCKED" && local_snapshot_valid_ &&
                grid_.cellAt(position_)>=0 && grid_.cells[grid_.cellAt(position_)]==0;
            if (!combat_escape && !margin_escape) {
                follower_.reset();
                publishCommand({});
            }

            if (message->status == "STALE_ENEMIES" || message->status == "WAIT_SELF_STATE" ||
                message->status == "SELF_DEAD_OR_MATCH_END" || message->status == "WAIT_GOAL") {
                const bool same_four_wait=message->status=="WAIT_GOAL" && mission_stage_=="CLEAR_ROBOTS" &&
                    mission_target_number_==4 && structure_healths_[3]>0 && !needs_refill_;
                cancelTask(same_four_wait);
            }

            return;
        }

        grid_.width = static_cast<int>(message->static_grid.info.width);
        grid_.height = static_cast<int>(message->static_grid.info.height);
        grid_.resolution = message->static_grid.info.resolution;
        grid_.origin_x = message->static_grid.info.origin.position.x;
        grid_.origin_y = message->static_grid.info.origin.position.y;
        grid_.cells = message->static_grid.data;
        bool valid = grid_.valid() && message->goal_id > 0 && finite(message->goal.x, message->goal.y) &&
                     message->path.header.frame_id == frame_ && message->static_grid.header.frame_id == frame_ &&
                     rclcpp::Time(message->path.header.stamp).nanoseconds() == stamp &&
                     rclcpp::Time(message->static_grid.header.stamp).nanoseconds() == stamp &&
                     !message->path.poses.empty() && message->path.poses.size() <= grid_.cells.size() &&
                     snapshot_reason_.empty();
        const geometry_msgs::msg::Quaternion& rotation = message->static_grid.info.origin.orientation;
        valid = valid && std::isfinite(rotation.w) && std::abs(std::abs(rotation.w) - 1) < 1e-6 && rotation.x == 0 &&
                rotation.y == 0 && rotation.z == 0;
        int goal_cell = grid_.cellAt({message->goal.x, message->goal.y});
        valid = valid && goal_cell >= 0 && grid_.cells[goal_cell] == 0;

        if (snapshot_reason_.empty() && !valid)
            snapshot_reason_ = "GRID_GOAL_OR_METADATA";
        path_.clear();
        for (const geometry_msgs::msg::PoseStamped& pose : message->path.poses) {
            campus::PlanPoint point{pose.pose.position.x, pose.pose.position.y};
            int cell = grid_.cellAt(point);

            if (!finite(point.x, point.y) || cell < 0 || grid_.cells[cell] != 0 ||
                (!path_.empty() && !campus::segmentFree(grid_, path_.back(), point))) {
                valid = false;
                if (snapshot_reason_.empty())
                    snapshot_reason_ = "PATH_STATIC_COLLISION";
                break;
            }

            path_.push_back(point);
        }

        valid = valid && !path_.empty() && grid_.cellAt(path_.back()) == goal_cell;
        if (!valid && snapshot_reason_.empty())
            snapshot_reason_ = "PATH_END_MISMATCH";
        if (!valid) {
            plan_.valid = false;
            plan_.status = "INVALID_SNAPSHOT";
            cancelTask();
        }
    }

    void publishCommand(campus::PlanPoint reference_velocity) {
        geometry_msgs::msg::TwistStamped preview;
        preview.header.stamp = now();
        preview.header.frame_id = frame_;
        preview.twist.linear.x = reference_velocity.x;
        preview.twist.linear.y = reference_velocity.y;
        preview_pub_->publish(preview);

        if (!command_pub_) {
            return;
        }

        if (simulation_) {
            command_pub_->publish(preview);
            return;
        }

        const std::vector<double>& matrix = projection_.matrix;
        double determinant = matrix[0] * matrix[4] - matrix[1] * matrix[3];
        geometry_msgs::msg::TwistStamped command;
        command.header = preview.header;
        command.header.frame_id = "unity_world_scene";
        // 逆变换只使用线性部分；速度不能加位置变换的平移项。
        campus::PlanPoint world_velocity{
            (matrix[4] * reference_velocity.x - matrix[1] * reference_velocity.y) / determinant,
            (-matrix[3] * reference_velocity.x + matrix[0] * reference_velocity.y) / determinant};
        world_velocity = campus::boundedVector(world_velocity, max_speed_);
        if (mission_stage_ == "BUFF" && std::hypot(world_velocity.x,world_velocity.y) > .02) {
            double route_yaw=std::atan2(world_velocity.x,world_velocity.y)*180/M_PI;
            world_velocity=campus::boundedVector(world_velocity,
                campus::runeTravelSpeedLimit(travel_yaw_,route_yaw,std::hypot(world_velocity.x,world_velocity.y)));
        }
        command.twist.linear.x = scale_ * world_velocity.x;
        command.twist.linear.y = scale_ * world_velocity.y;
        command_pub_->publish(command);
    }

    void publishView(const campus::FollowResult& result) {
        nav_msgs::msg::Path local;
        local.header.frame_id = frame_;
        local.header.stamp = now();

        for (campus::PlanPoint point : result.trajectory) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = local.header;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = 0.18;
            pose.pose.orientation.w = 1;
            local.poses.push_back(pose);
        }

        local_pub_->publish(local);

        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker robot;
        robot.header = local.header;
        robot.ns = "own_robot";
        robot.id = 0;
        robot.type = visualization_msgs::msg::Marker::CYLINDER;
        robot.action = have_state_ ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
        robot.pose.position.x = position_.x;

        robot.pose.position.y = position_.y;
        robot.pose.position.z = 0.2;
        robot.pose.orientation.w = 1;
        robot.scale.x = robot.scale.y = 0.6;
        robot.scale.z = 0.3;
        robot.color.g = robot.color.a = 1;
        markers.markers.push_back(robot);
        visualization_msgs::msg::Marker label = robot;

        label.ns = "follow_status";
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.pose.position.z = 0.9;
        label.scale.z = 0.35;
        label.text = result.status + (simulation_ ? " SIM" : (enabled_ ? " LIVE" : " PREVIEW: GAME OUTPUT OFF"));

        if (training_prior_)
            label.text += " UNITY TRAINING";
        markers.markers.push_back(label);
        visualization_msgs::msg::Marker mode = label;
        mode.ns = "output_mode";
        mode.action = visualization_msgs::msg::Marker::ADD;
        mode.pose.position.x = grid_.width > 0 ? grid_.origin_x + grid_.width * grid_.resolution / 2 : 14;
        mode.pose.position.y = grid_.height > 0 ? grid_.origin_y + grid_.height * grid_.resolution + 2.3 : 17.3;
        mode.scale.z = 0.5;
        mode.color.r = simulation_ || enabled_ ? 0.2 : 1;

        mode.color.g = 0.8;
        mode.color.b = simulation_ || enabled_ ? 1 : 0.1;
        mode.text =
            simulation_
                ? "OWN MODEL: NO GAME OUTPUT"
                : (enabled_
                       ? (training_prior_ ? "UNITY TRAINING: GAME OUTPUT ON | 0.2 m/s\nLANDMARKS CHECKED; TERRAIN PRIOR"
                                          : "GAME SPEED OUTPUT ENABLED")
                       : "PREVIEW ONLY: GAME SPEED OUTPUT DISABLED\nA goal moves the preview, not the game robot");

        if (training_prior_)
            mode.text = "UNITY TRAINING: GAME OUTPUT ON | " + std::to_string(max_speed_) +
                        " m/s\nLANDMARKS CHECKED; TERRAIN PRIOR";
        markers.markers.push_back(mode);
        markers_pub_->publish(markers);
    }

    void tick() {
        Clock::time_point time = Clock::now();
        double dt = std::chrono::duration<double>(time - last_tick_).count();

        last_tick_ = time;
        campus::FollowResult result;
        active_speed_limit_ = 0;
        ramp_limited_ = false;

        std::string phase = combat_enabled_ ? combat_.phase(steadyTime()) : "NAVIGATE";
        // Damage may still trigger a checked escape during travel, but visual
        // acquisition and firing remain disabled until this goal is reached.
        if (after_arrival_ && !arrived_for_goal_ && phase != "EVADE")
            phase = "NAVIGATE";
        if (strict_order_ && (mission_stage_ == "OUTPOST" || mission_stage_ == "BASE") && !structureReady() &&
            phase != "EVADE") {
            phase = "NAVIGATE";
            combat_decision_ = mission_stage_ == "BASE" ? "POSITION_BASE_RANGE" : "POSITION_OUTPOST_RANGE";
        }

        if (mission_stage_ == "BUFF") {
            phase = "NAVIGATE";
            combat_decision_ = "RUNE_APPROACH";
        }

        if (openingTask() || !anchorReady()) {
            phase = "NAVIGATE";
            combat_decision_ = openingTask() ? "OPENING_RAMP_FIRST" : "RETURN_SENTRY_ANCHOR";
        }

        if (refill_ && phase != "EVADE")
            phase = "NAVIGATE";
        const bool feedback_fresh = have_state_ && alive_ && silence(state_received_) <= state_timeout_ &&
                                    now().seconds() - state_stamp_ * 1e-9 >= -.1 &&
                                    now().seconds() - state_stamp_ * 1e-9 <= state_timeout_;
        const bool mobile_stage = !strict_order_ || mission_stage_ == "CLEAR_ROBOTS" || mission_stage_ == "ROBOT";
        const bool four_planned_approach = strict_order_ && mission_stage_ == "CLEAR_ROBOTS" &&
            outpost_known_ && silence(mission_received_) <= .5 && plan_.valid && mission_goal_id_ == plan_.goal_id &&
            campus::fourthRouteApproachPending(mission_target_number_, campus::pointDistance(position_,outpost_point_),
                                               campus::pointDistance(position_,{plan_.goal.x,plan_.goal.y}));
        const bool fixed_four = four_station_fixed_ && strict_order_ && mission_stage_=="CLEAR_ROBOTS" &&
            mission_target_number_==4 && structure_healths_[3]>0 && mission_goal_id_==plan_.goal_id &&
            silence(mission_received_)<=.5;
        const bool four_context = !fixed_four && combat_enabled_ && mobile_stage && feedback_fresh && !openingTask() && !anchor_active_ &&
            !refill_ && !needs_refill_ && structure_healths_[3] > 0 &&
            (!strict_order_ || mission_target_number_ == 4) &&
            (!combat_.freshEnemy(steadyTime()) || combat_number_ == 4);
        const bool four_observed_view = four_context && combat_number_ == 4 && four_view_.ready(steadyTime()) &&
            (!highland_active_ || campus::onCombatHighland(grid_,highland_cells_,position_));
        if(!four_context)four_hold_.reset();
        const bool station_allowed = four_context && grid_.valid() && plan_.valid && mission_goal_id_ == plan_.goal_id &&
            silence(mission_received_) <= .5 && phase != "EVADE" && !recovery_goal_ && !repositioning_;
        bool four_no_fire_forward = four_no_fire_.update(steadyTime(),position_,ammo_remaining_,structure_healths_[3],
            station_allowed, four_observed_view && std::hypot(velocity_.x,velocity_.y)<.1);
        if (four_no_fire_forward && four_retry_path_.empty()) {
            // Prefer one useful advance; shorten it only when the deck or wall
            // makes the longer segment unavailable. Do not retry many tiny legs.
            for(double length : {campus::kFourthNoFireStep, .48, .30}) {
                auto candidate=campus::forwardRouteSegment(path_,position_,length);
                bool safe=candidate.size()>=2 && campus::pointDistance(candidate.front(),candidate.back())>=.15;
                for (size_t i=1;safe && i<candidate.size();++i) {
                    safe=campus::segmentWithReserve(grid_,candidate[i-1],candidate[i],.03);
                    const int steps=std::max(1,int(std::ceil(campus::pointDistance(candidate[i-1],candidate[i])/grid_.resolution)));
                    for(int j=0;safe && j<=steps;++j) {
                        double f=double(j)/steps;
                        campus::PlanPoint p{candidate[i-1].x+f*(candidate[i].x-candidate[i-1].x),
                                            candidate[i-1].y+f*(candidate[i].y-candidate[i-1].y)};
                        if(highland_active_ && !campus::onCombatHighland(grid_,highland_cells_,p))safe=false;
                    }
                }
                if(safe) {four_retry_path_=std::move(candidate);break;}
            }
            if(four_retry_path_.empty()) {four_no_fire_.finish();four_no_fire_forward=false;}
        }
        const bool four_complete_view = four_context && !four_no_fire_forward &&
            four_hold_.update(steadyTime(),position_,structure_healths_[3],four_observed_view);
        const bool four_route_approach = combat_enabled_ && strict_order_ && mission_stage_ == "CLEAR_ROBOTS" &&
            !four_complete_view && !openingTask() && !anchor_active_ && !refill_ && !needs_refill_ && structure_healths_[3] > 0 &&
            (!combat_.freshEnemy(steadyTime()) || combat_number_ == 4) &&
            (four_planned_approach || four_no_fire_forward);
        if (four_route_approach && phase != "EVADE") {
            phase = "NAVIGATE";
            combat_decision_ = four_no_fire_forward ? "FOUR_NO_FIRE_ADVANCE" :
                (four_hold_.advancing() ? "FOUR_LOST_VIEW_ADVANCE" : "FOUR_ROUTE_APPROACH");
        }
        if(!four_context || (!four_hold_.advancing() && !four_no_fire_forward))four_retry_path_.clear();
        else if(four_route_approach && four_retry_path_.empty())
            four_retry_path_=campus::forwardRouteSegment(path_,position_,campus::kFourthLostViewStep);

        // A transport pause stops commands below, but must not grant another
        // unproductive combat stop when fresh feedback and a new plan return.
        if (anchor_active_ || !mobile_stage || refill_ || needs_refill_ || after_arrival_)
            stop_progress_.reset();
        const bool resume_path =
            !anchor_active_ && mobile_stage && !refill_ && !needs_refill_ && feedback_fresh && !after_arrival_ &&
            stop_progress_.update(steadyTime(), position_, structure_healths_,
                                  combat_.mobile_enemy &&
                                      (phase == "ENGAGE" || phase == "ACQUIRE" || phase == "REACQUIRE"),
                                  combat_number_ == 4 ? 1.6 : .6, four_complete_view);

        // Keep the complete view parked during normal combat. The single
        // no-fire station adjustment above can temporarily release this hold.
        const bool four_firing_view = four_complete_view;
        if (four_firing_view) {
            phase = "ENGAGE";
            repositioning_ = false;
            follower_.reset();
            evader_.reset();
            combat_decision_ = "FOUR_FULL_ARMOR_HOLD";
        }

        if (resume_path && phase != "EVADE") {
            phase = "NAVIGATE";
            repositioning_ = false;
            combat_decision_ = "NO_PROGRESS_ADVANCE";
        }

        if (phase == "EVADE" || phase == "NAVIGATE" || (phase == "REACQUIRE" && combat_number_ != 4) || refill_ || needs_refill_)
            repositioning_ = false;
        if (repositioning_)
            phase = "REPOSITION";
        if (fixed_four && phase!="EVADE" && !refill_ && !needs_refill_) {
            if (campus::pointDistance(position_,{plan_.goal.x,plan_.goal.y})>release_tolerance_)
                arrived_for_goal_=false;
            four_hold_.reset();
            four_no_fire_.reset();
            four_retry_path_.clear();
            repositioning_=false;
            phase=arrived_for_goal_ ? "ENGAGE" : "NAVIGATE";
            combat_decision_=arrived_for_goal_ ? "FOUR_FIXED_STATION_HOLD" : "FOUR_FIXED_STATION_TRANSIT";
        }
        if (phase != "NAVIGATE") {
            motion_recovery_.reset();
            recovery_goal_.reset();
        }

        double state_age = now().seconds() - state_stamp_ * 1e-9;
        feedback_age_ = std::max(0., state_age);
        double plan_age = now().seconds() - plan_stamp_ * 1e-9;
        double enemy_age = have_plan_ ? age(plan_.enemy_stamp) : INFINITY;

        if (enabled_ && !simulation_ && count_publishers(command_pub_->get_topic_name()) > 1) {
            result.status = "MULTIPLE_SPEED_PUBLISHERS";
            cancelTask();
        } else if (!have_state_) {
            result.status = "WAIT_STATE";
        } else if (silence(state_received_) > state_timeout_ || state_age < -0.1 || state_age > state_timeout_) {
            result.status = "STALE_STATE";
            cancelTask(true);
        } else if (!alive_) {
            result.status = "DEAD_OR_MATCH_END";
            cancelTask();
        } else if (!have_plan_) {
            result.status = "WAIT_PATH";
        } else if (strict_order_ && ((needs_refill_ && !refill_) ||
                                     (mission_goal_id_ != plan_.goal_id && (refill_ || phase == "NAVIGATE")))) {
            result.status = "WAIT_MISSION_PATH";
        } else if (silence(plan_received_) > plan_timeout_ || plan_age < -0.1 || plan_age > plan_timeout_) {
            result.status = "STALE_PATH";
            cancelTask();
        } else if (!plan_.valid && !(local_snapshot_valid_ && plan_.status=="START_BLOCKED") &&
                   !(phase != "NAVIGATE" && local_snapshot_valid_ &&
                                     (plan_.status == "NO_PATH" || plan_.status == "GOAL_BLOCKED" ||
                                      plan_.status == "START_BLOCKED"))) {
            result.status = plan_.status;
        } else if (!std::isfinite(enemy_age) || enemy_age < -0.1 || enemy_age > enemy_timeout_) {
            result.status = "STALE_ENEMIES";
            cancelTask();
        } else if (plan_.goal_id <= cancelled_id_) {
            result.status = "CANCELLED_SELECT_NEW_GOAL";
        } else if (!std::isfinite(dt) || dt <= 0 || dt > state_timeout_) {
            result.status = "CONTROL_DELAY";
            cancelTask();
        } else {
            if (anchor_active_ && anchor_known_ && campus::anchorContains(position_, anchor_point_, anchor_radius_) &&
                (!highland_active_ || campus::onCombatHighland(grid_,highland_cells_,position_)))
                campus::restrictAnchorGrid(grid_, anchor_point_, anchor_radius_);

            if (highland_active_ && campus::onCombatHighland(grid_,highland_cells_,position_))
                campus::restrictCombatHighland(grid_,highland_cells_);
            std::vector<campus::LocalObstacle> obstacles;
            for (std::size_t i = 0; i < plan_.enemy_centres.size(); ++i) {
                campus::LocalObstacle obstacle;
                obstacle.centre = {plan_.enemy_centres[i].x, plan_.enemy_centres[i].y};
                obstacle.speed_bound = plan_.enemy_mobile[i] ? plan_.enemy_speed_bound_mps : 0;
                obstacle.radius = plan_.enemy_radii[i] + obstacle.speed_bound * std::max(0.0, plan_age);

                if (!plan_.enemy_hard_radii.empty())
                    obstacle.hard_radius = plan_.enemy_hard_radii[i];
                obstacles.push_back(obstacle);
            }

            double speed_limit = max_speed_;
            // Resume the normal checked route speed after an unproductive
            // combat stop; rollout, turns and ramp limits still apply.
            bool near_ramp = false;
            // Slow before the surface, not only after the robot has entered it.
            for (const auto& point : path_) {
                if (campus::pointDistance(point, position_) > 0.9)
                    continue;
                int cell = grid_.cellAt(point);

                if (std::find(ramp_cells_.begin(), ramp_cells_.end(), cell) != ramp_cells_.end()) {
                    near_ramp = true;
                    speed_limit = std::min(speed_limit, ramp_speed_);
                }
            }

            int own_cell = grid_.cellAt(position_);
            if (std::find(ramp_cells_.begin(), ramp_cells_.end(), own_cell) != ramp_cells_.end()) {
                near_ramp = true;
                speed_limit = std::min(speed_limit, ramp_speed_);
            }

            active_speed_limit_ = speed_limit;
            ramp_limited_ = near_ramp;

            int health_index = campus::armorHealthIndex(combat_number_);
            if (plan_.valid && !fixed_four && !four_firing_view && !anchor_active_ && (phase == "ENGAGE" || phase == "ACQUIRE" || phase == "REPOSITION") &&
                combat_.mobile_enemy && combat_.freshEnemy(steadyTime()) && health_index >= 0) {
                combat_progress_.observe(combat_number_, structure_healths_[health_index], steadyTime());
                if (!repositioning_ &&
                    combat_progress_.needsAdjustment(steadyTime(), campus::pointDistance(position_, combat_.enemy),combat_number_==4 ? 3.1 : 3.8) &&
                    std::hypot(velocity_.x, velocity_.y) <= .03) {
                    std::optional<campus::PlanPoint> waypoint;
                    auto approach_obstacles = obstacles;
                    approach_obstacles.push_back({combat_.enemy, .75, plan_.enemy_speed_bound_mps});

                    for (auto candidate : campus::combatApproachWaypoints(grid_, position_, combat_.enemy,
                            campus::combatApproachStep(combat_number_,campus::pointDistance(position_,combat_.enemy)))) {
                        if (combat_number_ == 4 && !campus::segmentWithReserve(grid_,position_,candidate,.03))
                            continue;
                        double distance = campus::pointDistance(position_, candidate);
                        campus::PlanPoint trial{.2 * (candidate.x - position_.x) / distance,
                                                .2 * (candidate.y - position_.y) / distance};
                        std::vector<campus::PlanPoint> rollout;

                        if (evader_.rollout(grid_, approach_obstacles, position_, velocity_, trial, rollout)) {
                            waypoint = candidate;
                            break;
                        }
                    }

                    combat_progress_.adjusted(steadyTime());
                    if (waypoint) {
                        reposition_goal_ = *waypoint;
                        reposition_started_ = steadyTime();
                        repositioning_ = true;
                        phase = "REPOSITION";
                        evader_.reset();
                        combat_decision_ = campus::pointDistance(position_, combat_.enemy) > 3.8
                                               ? "CLOSE_RANGE"
                                               : "NO_DAMAGE_REPOSITION";
                    } else {
                        combat_decision_ = "ADJUSTMENT_NO_SAFE_PATH";
                    }
                }
            }

            if(!plan_.valid && plan_.status=="START_BLOCKED" && phase=="NAVIGATE") {
                auto escape=follower_.recoverEnemyMargin(grid_,obstacles,position_,velocity_,
                    {plan_.goal.x,plan_.goal.y},dt,std::max(0.,state_age),enemy_recovery_margin_);
                if(escape) {result=std::move(*escape);combat_decision_="EXIT_ENEMY_MARGIN";}
                else result.status="START_BLOCKED";
            } else if (phase == "REPOSITION") {
                std::vector<campus::PlanPoint> adjustment_path;
                int steps = std::max(1, static_cast<int>(std::ceil(campus::pointDistance(position_, reposition_goal_) /
                                                                   grid_.resolution)));
                for (int i = 0; i <= steps; ++i) {
                    double f = static_cast<double>(i) / steps;
                    adjustment_path.push_back({position_.x + f * (reposition_goal_.x - position_.x),
                                               position_.y + f * (reposition_goal_.y - position_.y)});
                }

                obstacles.push_back({combat_.enemy, .75, combat_.mobile_enemy ? plan_.enemy_speed_bound_mps : 0.});
                follower_.reset();
                result = evader_.update(grid_, adjustment_path, obstacles, position_, velocity_, reposition_goal_, dt,
                                        feedback_dt_, combat_number_==4 ? speed_limit : std::min(speed_limit,.35), INFINITY, std::max(0., state_age));
                bool finished = result.status == "ARRIVED";
                bool aborted = steadyTime() - reposition_started_ > (combat_number_ == 4 ? 6. : 4.);
                result.status =
                    std::hypot(result.command.x, result.command.y) > 1e-6
                        ? "COMBAT_REPOSITION"
                        : (result.status == "SETTLING" ? "COMBAT_REPOSITION_SETTLING" : "COMBAT_REPOSITION_BLOCKED");

                if (finished || aborted) {
                    result.command = {};
                    result.status = finished ? "COMBAT_ENGAGE" : "COMBAT_REPOSITION_BLOCKED";
                    repositioning_ = false;
                    combat_progress_.settled(steadyTime());
                    evader_.reset();
                    phase = "ENGAGE";
                }

                stalled_ = 0;
            } else if (phase == "ENGAGE" || phase == "ACQUIRE" || phase == "REACQUIRE") {
                follower_.reset();
                evader_.reset();
                stalled_ = 0;
                result.status = phase == "REACQUIRE" ? "COMBAT_REACQUIRE"
                                                     : (phase == "ACQUIRE" ? "COMBAT_ACQUIRE" : "COMBAT_ENGAGE");
            } else if (phase == "EVADE") {
                auto goal = combat_.dodge(grid_, position_, steadyTime());

                if (combat_.freshEnemy(steadyTime()))
                    obstacles.push_back({combat_.enemy, 0.75, combat_.mobile_enemy ? plan_.enemy_speed_bound_mps : 0.});
                std::vector<campus::PlanPoint> escape_path;
                int steps =
                    std::max(1, static_cast<int>(std::ceil(campus::pointDistance(position_, goal) / grid_.resolution)));

                for (int i = 0; i <= steps; ++i) {
                    double f = static_cast<double>(i) / steps;
                    escape_path.push_back(
                        {position_.x + f * (goal.x - position_.x), position_.y + f * (goal.y - position_.y)});
                }

                follower_.reset();
                result = evader_.update(grid_, escape_path, obstacles, position_, velocity_, goal, dt, feedback_dt_,
                                        std::min(speed_limit, 0.6), INFINITY, std::max(0., state_age));
                result.status =
                    std::hypot(result.command.x, result.command.y) > 1e-6 ? "COMBAT_EVADE" : "COMBAT_EVADE_BLOCKED";
                stalled_ = 0;
            } else {
                evader_.reset();
                const bool four_step=four_route_approach && (four_hold_.advancing() || four_no_fire_forward);
                const auto& drive_path=four_step?four_retry_path_:path_;
                const auto drive_goal=four_step && !drive_path.empty()?drive_path.back():campus::PlanPoint{plan_.goal.x,plan_.goal.y};
                result = follower_.update(grid_, drive_path, obstacles, position_, velocity_, drive_goal,
                                          dt, feedback_dt_, speed_limit, near_ramp ? ramp_lookahead_ : INFINITY,
                                          std::max(0., state_age), fixed_four ? .06 : INFINITY);
                // A valid Dijkstra route must retain its normal obstacle-wait
                // and recovery logic. Enemy-margin escape is authorized ONLY
                // by the START_BLOCKED branch above. Using it here reversed
                // the approach on alternate ticks and swung the travel aim.
                const bool can_recover = result.status == "FOLLOWING" || result.status == "LOCAL_BLOCKED" ||
                                         result.status == "WAIT_OBSTACLE";

                if (!four_no_fire_forward &&
                    campus::pointDistance(position_, {plan_.goal.x, plan_.goal.y}) > release_tolerance_ &&
                    (can_recover || recovery_goal_)) {
                    if (!recovery_goal_)
                        motion_recovery_.observe(position_, dt);
                    if (!recovery_goal_ && motion_recovery_.needed()) {
                        recovery_goal_ = motion_recovery_.probe(follower_, grid_, obstacles, position_, velocity_,
                                                                result.target, std::max(0., state_age));
                        if (recovery_goal_) {
                            recovery_elapsed_ = 0;
                            recovery_rejoining_ = false;
                            recovery_controller_.reset(velocity_);
                        } else if (motion_recovery_.exhausted()) {
                            result.command = {};
                            result.status = "STUCK_SELECT_NEW_GOAL";
                            cancelTask();
                        }
                    }

                    if (recovery_goal_) {
                        recovery_elapsed_ += dt;
                        result = recovery_controller_.update(grid_, {position_, *recovery_goal_}, obstacles, position_,
                                                             velocity_, *recovery_goal_, dt, feedback_dt_, .25,
                                                             INFINITY, std::max(0., state_age));
                        const bool done = result.status == "ARRIVED" || recovery_elapsed_ > 2.5;
                        result.status = "NAV_RECOVERY";

                        if (done) {
                            auto next =
                                (!recovery_rejoining_ && campus::pointDistance(position_, *recovery_goal_) <= .04)
                                    ? motion_recovery_.rejoin(follower_, grid_, obstacles, position_, velocity_,
                                                              std::max(0., state_age))
                                    : std::optional<campus::PlanPoint>{};
                            recovery_goal_ = next;
                            recovery_rejoining_ = true;
                            recovery_elapsed_ = 0;
                            recovery_controller_.reset(velocity_);
                            follower_.reset(velocity_);
                        }
                    }
                } else {
                    motion_recovery_.reset();
                    recovery_goal_.reset();
                }

                // ARRIVED includes position tolerance, measured stop speed
                // and settling; proximity or an empty command alone is insufficient.
                if (result.status == "ARRIVED") {
                    if(four_step) {
                        four_hold_.settled(steadyTime(),position_);
                        four_no_fire_.finish();
                        four_retry_path_.clear();
                        result.status="FOUR_ADVANCE_SETTLED";
                    } else arrived_for_goal_ = true;
                }
            }

            if ((simulation_ || enabled_) && phase == "NAVIGATE") {
                if (campus::pointDistance(progress_position_, position_) >= 0.05 || result.status == "ARRIVED" ||
                    result.status == "SETTLING") {
                    progress_position_ = position_;
                    stalled_ = 0;
                } else if (motion_recovery_.active()) {
                    // The bounded recovery sequence owns its timeout once a
                    // probe starts; allow its next direction to be tested.
                    stalled_ = 0;
                } else {
                    stalled_ += dt;
                }

                if (stalled_ >= blocked_timeout_) {
                    result.command = {};
                    result.trajectory.clear();
                    result.status = "STUCK_SELECT_NEW_GOAL";
                    cancelTask();
                }
            }
        }

        feedback_dt_ = 0;
        if (result.command.x == 0 && result.command.y == 0 && result.status != "ARRIVED" &&
            result.status != "SETTLING" && result.status != "COMBAT_REPOSITION_SETTLING") {
            // A checked pause can leave the real chassis coasting. Restarting
            // its acceleration ramp from zero on every pause produced repeated
            // 0.13 m/s requests despite fresh feedback showing ongoing travel.
            // Keep that physical velocity as the next search seed; this tick
            // still outputs zero and every next candidate is rolled out anew.
            const bool travel_pause = phase == "NAVIGATE" && feedback_fresh &&
                (result.status == "LOCAL_BLOCKED" || result.status == "WAIT_OBSTACLE");
            follower_.reset(travel_pause ? velocity_ : campus::PlanPoint{});
            evader_.reset();
        }

        publishCommand(result.command);
        publishView(result);

        std_msgs::msg::String status;
        std::ostringstream text;
        text << result.status << " goal=" << plan_.goal_id
             << " distance_m=" << campus::pointDistance(position_, {plan_.goal.x, plan_.goal.y})
             << " command=" << result.command.x << ',' << result.command.y
             << (simulation_ ? " SIMULATION" : (enabled_ ? " LIVE" : " PREVIEW_NO_GAME_OUTPUT"));
        text << " output_mode=" << (simulation_ ? "SIMULATION" : (enabled_ ? "REAL" : "PREVIEW"))
             << " speed_topic=" << (command_pub_ ? command_pub_->get_topic_name() : "NONE")
             << " alignment_verified=" << (verified_ ? "true" : "false");
        text << " training_prior=" << (training_prior_ ? "true" : "false")
             << " landmark_alignment_checked=" << (landmark_checked_ ? "true" : "false");
        text << " combat=" << phase << " combat_target=" << (phase == "NAVIGATE" ? 0 : combat_.target_id)
             << " combat_decision=" << combat_decision_ << " vision=" << vision_status_ << " combat_ready="
             << (((!after_arrival_ || arrived_for_goal_) && structureReady() && !openingTask() && anchorReady())
                     ? "true"
                     : "false");

        if (mission_stage_ == "BASE")
            text << " base_range_m=" << campus::pointDistance(position_, outpost_point_)
                 << " base_position_ready=" << structureReady();
        if (mission_stage_ == "OUTPOST")
            text << " outpost_range_m=" << campus::pointDistance(position_, outpost_point_)
                 << " outpost_position_ready=" << structureReady();
        text << " enemy_units=" << plan_.enemy_centres.size()
             << " snapshot_reason=" << (snapshot_reason_.empty() ? "NONE" : snapshot_reason_);
        text << " approach_target_number=" << combat_number_;
        text << " four_observed_view=" << four_observed_view << " four_hold=" << four_complete_view
             << " four_retry_forward=" << (four_context && four_hold_.advancing())
             << " four_no_fire_forward=" << four_no_fire_forward;
        text << " measured_speed_mps=" << std::hypot(velocity_.x, velocity_.y)
             << " speed_limit_mps=" << active_speed_limit_ << " ramp_limited=" << ramp_limited_
             << " feedback_age_s=" << feedback_age_ << " tracking_reserve_m=" << tracking_reserve_;
        text << " position_m=" << position_.x << ',' << position_.y << " available_reserve_m="
             << (grid_.valid() ? campus::availableTrackingReserve(grid_, position_, tracking_reserve_) : -1.)
             << " local_reason=" << (result.blocking_reason.empty() ? "NONE" : result.blocking_reason);

        if (!simulation_ && !enabled_) {
            text << " disabled_reason=REAL_OUTPUT_DISABLED";
        }

        status.data = text.str();
        status_pub_->publish(status);
        if (result.status != last_status_) {
            RCLCPP_INFO(get_logger(), "%s", status.data.c_str());
            last_status_ = result.status;
        }
    }

    bool simulation_ = false, enabled_ = false, verified_ = false, have_state_ = false, have_plan_ = false,
         alive_ = false;
    bool follow_projection_ = false;
    bool training_prior_ = false, landmark_checked_ = false;
    bool highland_active_=false;
    std::vector<int64_t> highland_cells_;
    bool anchor_active_ = false, anchor_known_ = false;
    campus::PlanPoint anchor_point_;
    double anchor_radius_ = 2.;
    bool combat_enabled_ = false;
    bool after_arrival_ = false, arrived_for_goal_ = false;
    bool field_order_ = false;
    bool strict_order_ = false, needs_refill_ = false;

    std::string snapshot_reason_;
    double travel_yaw_ = 0.;
    std::string mission_stage_ = "WAIT_MISSION";
    bool four_station_fixed_=false;
    uint64_t mission_goal_id_ = 0;
    bool local_snapshot_valid_ = false;
    int health_ = 0;
    int64_t tracks_stamp_ = 0;
    std::vector<int64_t> ramp_cells_;
    campus::CombatPolicy combat_;

    campus::CombatApproachProgress combat_progress_;
    campus::FourthArmorView four_view_;
    campus::FourthArmorHoldProgress four_hold_;
    campus::FourthNoFireAdvance four_no_fire_;
    int ammo_remaining_ = 0;
    std::vector<campus::PlanPoint> four_retry_path_;
    rclcpp::Subscription<campus_aim_interfaces::msg::ArmorFrame>::SharedPtr armor_view_sub_;
    campus::CombatStopProgress stop_progress_;
    campus::MotionRecovery motion_recovery_;
    std::optional<campus::PlanPoint> recovery_goal_;
    double recovery_elapsed_ = 0;
    bool recovery_rejoining_ = false;
    campus::PathFollower recovery_controller_;
    int combat_number_ = 0;
    int mission_target_number_ = 0;

    bool repositioning_ = false;
    double reposition_started_ = 0;
    campus::PlanPoint reposition_goal_;
    int player_ = 1;
    std::string frame_, session_, last_status_;
    bool structures_ = false, acquire_unknown_ = false, refill_ = false, route_decision_ = false, ignore_one_ = false,
         outpost_known_ = false;
    campus::StandoffBand outpost_band_, base_band_{1.8, 2.5, 3.2};
    campus::PlanPoint outpost_point_{NAN, NAN};
    Clock::time_point mission_received_;
    std::string combat_decision_ = "NO_TARGET";
    std::string vision_status_ = "NO_TRACK";
    std::array<int32_t, 7> structure_healths_{};
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
    double scale_, state_timeout_, plan_timeout_, enemy_timeout_, blocked_timeout_, max_speed_, ramp_speed_,
        ramp_lookahead_;
    double feedback_dt_ = 0, stalled_ = 0, remaining_seconds_ = 0;
    int64_t vision_minimum_stamp_ = 0;
    double active_speed_limit_ = 0, release_tolerance_ = 0;
    double feedback_age_ = 0, tracking_reserve_ = 0, enemy_recovery_margin_ = .15;
    bool ramp_limited_ = false;
    int64_t started_ns_ = 0, minimum_plan_ns_ = 0, state_stamp_ = 0, plan_stamp_ = 0;
    uint64_t sequence_ = 0, cancelled_id_ = 0;
    Clock::time_point last_tick_, state_received_, plan_received_;
    campus::PlanningProjection projection_;

    campus::DijkstraGrid grid_;
    campus::PathFollower follower_, evader_;
    campus::PlanPoint position_, velocity_, progress_position_;
    std::vector<campus::PlanPoint> path_;
    campus_aim_interfaces::msg::NavigationPlan plan_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_, preview_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_pub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Subscription<campus_aim_interfaces::msg::NavigationPlan>::SharedPtr plan_sub_;
    rclcpp::Subscription<tdt_interface::msg::AutoAimState>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_sub_;
    rclcpp::Subscription<campus_aim_interfaces::msg::TrackFrame>::SharedPtr tracks_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, stopSignal);
    std::signal(SIGTERM, stopSignal);

    try {
        std::shared_ptr<PathFollowerNode> node = std::make_shared<PathFollowerNode>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);

        while (rclcpp::ok() && !stop_requested) {
            executor.spin_once(std::chrono::milliseconds(10));
        }

        if (rclcpp::ok()) {
            for (int i = 0; i < 3; ++i) {
                node->stopForShutdown();
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }

        executor.remove_node(node);
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("path_follower"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
