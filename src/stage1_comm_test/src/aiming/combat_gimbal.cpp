#include "buff_detection/big_rune_gimbal.hpp"
#include "buff_detection/big_rune_trajectory.hpp"
#include "buff_detection/final_rune_accuracy.hpp"
#include "aiming/rune_gimbal_feedback.hpp"
#include "aiming/rune_single_shot.hpp"
#include "aiming/combat_gimbal_policy.hpp"
#include "aiming/reacquisition.hpp"
#include "aiming/fire_burst.hpp"
#include "aiming/fire_control.hpp"
#include "aiming/mobile_gimbal_feedback.hpp"
#include "navigation/mission_order.hpp"
#include "navigation/assault_mission.hpp"
#include <rclcpp/rclcpp.hpp>
#include <campus_aim_interfaces/msg/aim_solution.hpp>
#include <campus_aim_interfaces/msg/buff_solution.hpp>
#include <campus_aim_interfaces/msg/buff_status.hpp>
#include <tdt_interface/msg/auto_aim_state.hpp>
#include <tdt_interface/msg/send_data.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <map>
#include <sstream>
#include <thread>

namespace {
using State = tdt_interface::msg::AutoAimState;
using Solution = campus_aim_interfaces::msg::AimSolution;
using Speed = geometry_msgs::msg::TwistStamped;
using Command = tdt_interface::msg::SendData;
using Text = std_msgs::msg::String;
using Fields = std::map<std::string, std::string>;

bool mobileStage(const std::string& stage) {
    return stage == "CLEAR_ROBOTS" || stage == "ROBOT" || stage == "WAIT_CLEAR_ROBOTS_POSITION" ||
           stage == "OPENING_RAMP";
}

double steadyTime() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

double stampSeconds(const builtin_interfaces::msg::Time& t) {
    return t.sec + t.nanosec * 1e-9;
}

std::string session(const State& s) {
#ifdef CAMPUS_STATE_HAS_SESSION_ID
    return std::to_string(s.session_id);
#else
    (void)s;
    return {};
#endif
}

Fields fields(const std::string& text) {
    Fields result;
    std::istringstream stream(text);
    std::string token;
    stream >> result["stage"];
    while (stream >> token) {
        auto p = token.find('=');
        if (p != std::string::npos)
            result[token.substr(0, p)] = token.substr(p + 1);
    }

    return result;
}

double number(const Fields& f, const std::string& key, double fallback = 0) {
    auto it = f.find(key);
    if (it == f.end())
        return fallback;
    try {
        auto v = std::stod(it->second);
        return std::isfinite(v) ? v : fallback;
    } catch (...) {
        return fallback;
    }
}

uint64_t identifier(const Fields& f, const std::string& key) {
    auto it = f.find(key);
    if (it == f.end())
        return 0;
    try {
        return std::stoull(it->second);
    } catch (...) {
        return 0;
    }
}

template <class T> struct Sample {
    typename T::ConstSharedPtr message;
    double received = -INFINITY;

    void put(typename T::ConstSharedPtr value) {
        message = value;
        received = steadyTime();
    }

    bool recent(double limit) const {
        auto age = steadyTime() - received;
        return message && age >= 0 && age <= limit;
    }

    void clear() {
        message.reset();
        received = -INFINITY;
    }
};
}

//什么时候采用aimsolver的瞄准解，以及什么时候进行搜索、导航朝向控制或停止射击
class CombatGimbal final : public rclcpp::Node {
public:
    CombatGimbal() : Node("combat_gimbal") {
        int player = declare_parameter<int>("player_id", 1);
        if (player != 1 && player != 2)
            throw std::invalid_argument("player_id must be 1 or 2");
        scan_enabled_ = declare_parameter<bool>("scan.enabled", false);
        scan_rate_ = declare_parameter<double>("scan.rate_deg_s", 45.);

        pitch_search_enabled_ = declare_parameter<bool>("search.pitch_enabled", true);
        pitch_search_rate_ = declare_parameter<double>("search.pitch_rate_deg_s", 12.);

        after_arrival_ = declare_parameter<bool>("combat.after_arrival", false);

        strict_ = declare_parameter<bool>("mission.strict_order", false);
        mobile_feedback_enabled_ = declare_parameter<bool>("command.mobile_feedback_compensation", false);
        mobile_feedback_delay_ = declare_parameter<double>("command.mobile_feedback_delay_s", .04);
        if (!std::isfinite(mobile_feedback_delay_) || mobile_feedback_delay_ < 0 || mobile_feedback_delay_ > .1)
            throw std::invalid_argument("command.mobile_feedback_delay_s must be 0..0.1");
        field_order_ = declare_parameter<bool>("mission.field_robot_order", false);

        solution_timeout_ = declare_parameter<double>("command.solution_timeout_s", .2);
        mobile_coast_ = declare_parameter<double>("command.mobile_fire_coast_ms", 250.) / 1000.;

        outpost_source_age_ = declare_parameter<double>("outpost.target_timeout_ms", 280.) / 1000.;
        outpost_coast_ = declare_parameter<double>("outpost.fire_coast_ms", 100.) / 1000.;

        if (!std::isfinite(outpost_source_age_) || outpost_source_age_ < .2 || outpost_source_age_ > .3 ||
            !std::isfinite(outpost_coast_) || outpost_coast_ < 0 || outpost_coast_ > .15)
            throw std::invalid_argument("Invalid outpost source limits");

        burst_hold_ = declare_parameter<double>("command.fire_burst_hold_ms", 120.) / 1000.;
        matrix_ = declare_parameter<std::vector<double>>("unity_to_reference", {0., -1., 14., 1., 0., 9.});

        if (!std::isfinite(burst_hold_) || burst_hold_ < 0 || burst_hold_ > .15 || !std::isfinite(mobile_coast_) ||
            mobile_coast_ < 0 || mobile_coast_ > .25 || !std::isfinite(solution_timeout_) || solution_timeout_ <= 0 ||
            solution_timeout_ > .2 || !std::isfinite(scan_rate_) || scan_rate_ < 5 || scan_rate_ > 90 ||
            !std::isfinite(pitch_search_rate_) || pitch_search_rate_ < 3 || pitch_search_rate_ > 30 ||
            matrix_.size() != 6 ||
            !std::all_of(matrix_.begin(), matrix_.end(), [](double v) { return std::isfinite(v); }) ||
            std::abs(matrix_[0] * matrix_[4] - matrix_[1] * matrix_[3]) < 1e-6)
            throw std::invalid_argument("Invalid gimbal timing, scan or projection parameters");

        auto latest = rclcpp::QoS(1).reliable();
        auto cached = rclcpp::QoS(1).reliable().transient_local();

        output_ = create_publisher<Command>("/target_angles_player_" + std::to_string(player), rclcpp::QoS(10).reliable());
        status_ = create_publisher<Text>("/combat/status", latest);
        rune_shot_pub_ = create_publisher<std_msgs::msg::UInt64>("/buff/shot_sent", latest);

        state_sub_ = create_subscription<State>("/autoaim/state", latest, [this](State::ConstSharedPtr s) {
            if (state_.message && s->header.frame_id == state_.message->header.frame_id &&
                session(*s) == session(*state_.message) && s->frame_sequence == state_.message->frame_sequence &&
                s->remaining_seconds <= state_.message->remaining_seconds+2.)
                return;
            if (!state_.message || s->header.frame_id != state_.message->header.frame_id ||
                session(*s) != session(*state_.message) || s->frame_sequence < state_.message->frame_sequence ||
                s->remaining_seconds > state_.message->remaining_seconds + 2.) {
                buff_feedback_.reset();
                mobile_feedback_.reset();
                big_feedback_.reset();
                big_successes_.reset();final_trim_.reset();final_request_=0;
                buff_sequence_min_ = s->frame_sequence;
                rune_shots_.reset();
                rune_shot_sent_=rune_shot_goal_=rune_shot_request_=0;
                rune_shot_observation_=builtin_interfaces::msg::Time();
                buff_match_start_stamp_=stampSeconds(s->header.stamp);
                last_command_.if_shoot=false;
                if(std::isfinite(s->yaw_degrees)&&std::isfinite(s->pitch_degrees)) {
                    last_command_.yaw=s->yaw_degrees;last_command_.pitch=s->pitch_degrees;
                }
                solution_.clear();
                buff_solution_.clear();
                follow_.clear();
                mission_.clear();
                speed_.clear();
                heading_.reset();
                sentry_search_centre_.reset();
                scan_.reset();
                pitch_search_.reset();
                aim_hold_.reset();
                contact_.reset();
                burst_.reset();
                blocked_ = true;
                mobile_search_pitch_ = 0.;
                mobile_search_pitch_time_ = -INFINITY;
            }
            buff_feedback_.update(stampSeconds(s->header.stamp), s->yaw_degrees, s->pitch_degrees);
            mobile_feedback_.update(stampSeconds(s->header.stamp), s->yaw_degrees, s->pitch_degrees);
            big_feedback_.measured(stampSeconds(s->header.stamp), s->yaw_degrees, s->pitch_degrees);
            state_.put(s);
            if (mission_fields_["stage"]=="BUFF" && rune_shots_.active(steadyTime()) &&
                s->ammo_remaining<rune_shot_ammo_) {
                rune_shots_.cancel();
                tick();
            }
        });

        solution_sub_ = create_subscription<Solution>("/aim/solution", latest, [this](Solution::ConstSharedPtr s) {

            if (solution_.message && stampSeconds(s->header.stamp) < stampSeconds(solution_.message->header.stamp))
                return;
            solution_.put(s);

            if (phase_ == "ENGAGE" || phase_ == "ACQUIRE" || phase_ == "EVADE" || phase_ == "REPOSITION" ||
                last_command_.if_shoot || phase_ == "REACQUIRE" || last_mode_ == "REFILL_DEFENSE" ||
                last_mode_ == "STATIONARY_DEFENSE" || last_mode_ == "SEARCH_REFILL_THREAT")
                tick();
        });

        buff_sub_ = create_subscription<campus_aim_interfaces::msg::BuffSolution>(
            "/buff/solution", latest, [this](campus_aim_interfaces::msg::BuffSolution::ConstSharedPtr s) {
                if(stampSeconds(s->header.stamp)<buff_match_start_stamp_ ||
                   (s->fire && stampSeconds(s->observation_stamp)<buff_match_start_stamp_))return;
                if (buff_solution_.message &&
                    stampSeconds(s->header.stamp) < stampSeconds(buff_solution_.message->header.stamp))
                    return;
                buff_solution_.put(s);
                if (mission_fields_["stage"] == "BUFF")
                    tick();
            });

        buff_status_sub_=create_subscription<campus_aim_interfaces::msg::BuffStatus>(
            "/buff/status",latest,[this](campus_aim_interfaces::msg::BuffStatus::ConstSharedPtr b) {
                if(!state_.message || stampSeconds(b->header.stamp)<buff_match_start_stamp_ || !sourceFresh(b->header.stamp,.3) ||
                   b->header.frame_id!=state_.message->header.frame_id || b->state_session!=session(*state_.message) ||
                   b->frame_sequence<buff_sequence_min_ || b->frame_sequence>state_.message->frame_sequence)return;
                big_successes_.observe(b->request_id,b->opportunities,b->activated);
            });

        speed_sub_ = create_subscription<Speed>("/target_speed_player_" + std::to_string(player), latest,
                                                [this](Speed::ConstSharedPtr s) { speed_.put(s); });

        follow_sub_ = create_subscription<Text>("/navigation/follow_status", cached, [this](Text::ConstSharedPtr m) {
            auto f = fields(m->data);
            phase_ = f["combat"];
            ready_ = f["combat_ready"] == "true";
            goal_ = identifier(f, "goal");
            navigation_target_ = identifier(f, "combat_target");
            if (cancelled_goal_ && goal_ > *cancelled_goal_)
                cancelled_goal_.reset();
            const auto& mode = f["stage"];
            const bool travelling = mode == "FOLLOWING" || mode == "NAV_RECOVERY" || mode == "CLEARANCE_RECOVERY" ||
                                    mode == "STATIONARY_MARGIN_RECOVERY" || mode == "ENEMY_MARGIN_RECOVERY" || mode == "ARRIVED" || mode == "SETTLING" ||
                                    mode == "LOCAL_BLOCKED" || mode == "WAIT_OBSTACLE";
            const bool combat = mode.rfind("COMBAT_", 0) == 0;
            blocked_ = cancelled_goal_.has_value() || !(travelling || combat);
            follow_.put(m);
            if (last_command_.if_shoot)
                tick();
        });

        mission_sub_ = create_subscription<Text>("/mission/status", cached, [this](Text::ConstSharedPtr m) {
            auto next = fields(m->data);
            if (mobileStage(next["stage"]) && !mobileStage(mission_fields_["stage"])) {
                mobile_search_pitch_ = 0.;
                mobile_search_pitch_time_ = -INFINITY;
                aim_hold_.reset();
                contact_.reset();
                burst_.reset();
            }

            if (next["stage"] != mission_fields_["stage"]) {
                buff_solution_.clear();
                aim_hold_.reset();
                contact_.reset();
                burst_.reset();
            }
            mission_fields_ = std::move(next);
            mission_.put(m);
        });

        cancel_sub_ = create_subscription<std_msgs::msg::Bool>("/planning/cancel", latest,
                                                               [this](std_msgs::msg::Bool::ConstSharedPtr m) {
                                                                   if (m->data) {
                                                                       cancelled_goal_ = goal_;
                                                                       blocked_ = true;
                                                                   }
                                                               });
        last_tick_ = steadyTime();

        timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { tick(); });
        RCLCPP_INFO(get_logger(), "C++ gimbal owner: atomic /aim/solution; output=%s", output_->get_topic_name());
    }

    void stop() {
        last_command_.if_shoot = false;
        output_->publish(last_command_);
    }

private:
    bool sourceFresh(const builtin_interfaces::msg::Time& stamp, double limit) const {
        double age = now().seconds() - stampSeconds(stamp);
        return std::isfinite(age) && age >= -.1 && age <= limit;
    }

    std::optional<double> objectiveBearing(const State& s, double* distance = nullptr) const {
        if (!mission_.recent(.5))
            return {};
        const bool sentry_search=campus::sentrySearchStage(mission_fields_.at("stage"),s.enemy_healths,
            mission_fields_.count("ignored_enemy_one") && mission_fields_.at("ignored_enemy_one")=="1") &&
            mission_fields_.count("sentry_search_known") && mission_fields_.at("sentry_search_known")=="1";
        const auto known = mission_fields_.find("target_known");
        if (!sentry_search && (known == mission_fields_.end() || known->second != "1"))
            return {};
        double a = matrix_[0], b = matrix_[1], c = matrix_[2], d = matrix_[3], e = matrix_[4], f = matrix_[5],
               det = a * e - b * d;
        double x = -s.position.y / 3., z = s.position.x / 3.;
        double dx = number(mission_fields_, sentry_search ? "sentry_search_x_m" : "target_x_m") - (a * x + b * z + c),
               dy = number(mission_fields_, sentry_search ? "sentry_search_y_m" : "target_y_m") - (d * x + e * z + f);
        if (distance)
            *distance = std::hypot(dx, dy);
        if (std::hypot(dx, dy) <= .05)
            return {};
        return std::atan2((e * dx - b * dy) / det, (-d * dx + a * dy) / det) * 180 / M_PI;
    }

    std::optional<double> travel(Command& command, const State& s, double dt, std::string& source) {
        const bool speed_fresh = speed_.recent(.15) && sourceFresh(speed_.message->header.stamp, .15);
        auto [yaw, origin] =
            campus::motionHeading(s.twist.linear.x, s.twist.linear.y, speed_fresh ? speed_.message->twist.linear.x : 0.,
                                  speed_fresh ? speed_.message->twist.linear.y : 0.);
        const auto stage = mission_fields_["stage"];

        if ((stage == "BUFF" || stage == "RUNE_RETURN" || stage=="SENTRY_ADVANCE") && speed_fresh &&
            std::hypot(speed_.message->twist.linear.x, speed_.message->twist.linear.y) >= .03) {
            yaw = std::atan2(speed_.message->twist.linear.x, speed_.message->twist.linear.y) * 180 / M_PI;
            origin = "COMMAND";
        }
        if (!yaw && (stage == "OUTPOST" || stage == "BASE" || stage == "ROBOT" || stage == "CLEAR_ROBOTS")) {
            yaw = objectiveBearing(s);
            if (yaw)
                origin = "OBJECTIVE";
        }
        source = origin;
        if (yaw) {
            auto v = heading_.update(s.yaw_degrees, s.pitch_degrees, *yaw, dt);
            command.yaw = v.yaw;
            command.pitch = v.pitch;
        }

        return yaw;
    }

    void scanning(Command& command, const State& s, double dt, std::string& mode) {
        command.yaw = scan_.update(s.yaw_degrees, dt, scan_rate_);
        mode = "SCAN";
        double distance = INFINITY;
        auto bearing = objectiveBearing(s, &distance);
        auto stage = mission_fields_["stage"];
        if (bearing && distance < 4.7 && (stage == "OUTPOST" || stage == "BASE")) {
            command.yaw = campus::slewAngle(last_command_.yaw, *bearing + 25. * std::sin(steadyTime() * .8), dt, 90.);
            scan_.reset();
            mode = "SEARCH_OBJECTIVE";
        }
    }

    void waitingSearch(Command& command, const State& s, double dt, std::string& mode) {
        auto bearing = objectiveBearing(s);
        if (bearing)
            command.yaw = campus::searchBearing(last_command_.yaw, s.yaw_degrees, *bearing, dt);
        else
            command.yaw = scan_.update(s.yaw_degrees, dt, scan_rate_);
        mode = "SEARCH_NAVIGATION_WAIT";
    }

    std::string solutionState(const State& s) const {
        if (!solution_.message)
            return "WAIT_PACKETS";
        const auto& a = *solution_.message;

        const bool mobile = (a.number >= 1 && a.number <= 5) || a.number == 7;
        double source_limit = a.number == 6 ? outpost_source_age_ : solution_timeout_;
        double observation_limit =
            a.number == 6 ? outpost_source_age_ + outpost_coast_
                          : solution_timeout_ + (mobile && a.reason != "WAIT_TRACK_CONFIRM" ? mobile_coast_ : 0.);
        if (!sourceFresh(a.header.stamp, source_limit) || !sourceFresh(a.observation_stamp, observation_limit))
            return "SOURCE_EXPIRED";
        if (!solution_.recent(.12))
            return "RECEIPT_EXPIRED";
        if (a.header.frame_id != s.header.frame_id || a.state_session != session(s))
            return "SESSION_MISMATCH";

        if (!a.valid || !a.track_id || a.slot < 0 || !std::isfinite(a.yaw) || !std::isfinite(a.pitch) ||
            std::abs(a.pitch) > 60)
            return "SOLVER_INVALID";
        return "READY";
    }

    bool holdRecentAim(Command& command, const State& s, bool defense) const {
        if (!aim_hold_ || steadyTime() - aim_hold_time_ > campus::kReacquireHold ||
            aim_hold_->header.frame_id != s.header.frame_id || aim_hold_->state_session != session(s) ||
            aim_hold_->frame_sequence > s.frame_sequence)
            return false;
        const auto stage = campus::armorSelectionStage(mission_fields_.at("stage"), defense);
        if (strict_ && !campus::missionArmorAlive(stage, aim_hold_->number, s.enemy_healths,
                                                  mission_fields_.count("ignored_enemy_one") &&
                                                      mission_fields_.at("ignored_enemy_one") == "1", field_order_))
            return false;
        command.yaw = aim_hold_->yaw;
        command.pitch = aim_hold_->pitch;
        command.if_shoot = false;
        return true;
    }

    bool reacquire(Command& command, const State& s, double time, double dt) const {
        if (!contact_ || cancelled_goal_ || time - contact_time_ > campus::kReacquireTimeout ||
            contact_->header.frame_id != s.header.frame_id || contact_->state_session != session(s) ||
            contact_->frame_sequence > s.frame_sequence || s.is_weakened || s.ammo_remaining <= 0)
            return false;
        if (strict_ && (!mission_.recent(.5) ||
                        !campus::missionArmorAlive(mission_fields_.at("stage"), contact_->number, s.enemy_healths,
                                                   mission_fields_.count("ignored_enemy_one") &&
                                                       mission_fields_.at("ignored_enemy_one") == "1", field_order_)))
            return false;
        auto view = campus::localReacquireAngles(contact_->yaw, contact_->pitch, time - contact_time_);
        command.yaw = campus::slewAngle(last_command_.yaw, view.yaw, dt, 45.);
        command.pitch = view.pitch;
        command.if_shoot = false;
        return true;
    }

    bool structurePositionReady(const State& s) const {
        auto stage = mission_fields_.find("stage");
        if (!strict_ || stage == mission_fields_.end() || (stage->second != "OUTPOST" && stage->second != "BASE"))
            return true;
        campus::StandoffBand band;
        bool base = stage->second == "BASE";
        band.minimum = number(mission_fields_, base ? "base_min_range_m" : "outpost_min_range_m", base ? 1.8 : 2.8);
        band.preferred =
            number(mission_fields_, base ? "base_preferred_range_m" : "outpost_preferred_range_m", base ? 2.5 : 3.5);
        band.maximum = number(mission_fields_, base ? "base_max_range_m" : "outpost_max_range_m", base ? 3.2 : 4.2);
        double distance = INFINITY;
        return ready_ && objectiveBearing(s, &distance) && band.contains(distance);
    }

    bool robotHighlandReady(int number) const {
        auto here=mission_fields_.find("robot_highland_here");
        bool known=strict_ && here!=mission_fields_.end() && here->second!="-1";
        return campus::robotHighlandFireAllowed(number,known,
            known && mission_.recent(.5) && here->second=="1");
    }

    bool anchorPositionReady(const State& s) const {
        auto highland=mission_fields_.find("robot_highland_required");
        if(highland!=mission_fields_.end() && highland->second=="1" &&
           (!mission_.recent(.5) || !follow_.recent(.3) || !ready_ || identifier(mission_fields_,"goal")!=goal_))
            return false;
        auto active = mission_fields_.find("sentry_anchor_active");
        if (active == mission_fields_.end() || active->second != "1")
            return true;
        if (!mission_.recent(.5) || mission_fields_.find("sentry_anchor_known") == mission_fields_.end() ||
            mission_fields_.at("sentry_anchor_known") != "1")
            return false;
        double x = -s.position.y / 3., z = s.position.x / 3.;
        const auto& a = matrix_;
        return campus::anchorContains(
            {a[0] * x + a[1] * z + a[2], a[3] * x + a[4] * z + a[5]},
            {number(mission_fields_, "sentry_anchor_x_m"), number(mission_fields_, "sentry_anchor_y_m")},
            number(mission_fields_, "sentry_anchor_radius_m"));
    }

    bool mobileAlignmentReady(const Solution& a, const State& s) const {
        if (!((a.number >= 1 && a.number <= 5) || a.number == 7)) return true;
        auto feedback = mobile_feedback_enabled_
            ? mobile_feedback_.predict(now().seconds(),mobile_feedback_delay_) : campus::MobileGimbalEstimate{};
        if (std::abs(feedback.stamp-stampSeconds(s.header.stamp)) > 1e-6) feedback.valid = false;
        const double yaw = feedback.valid ? feedback.yaw : s.yaw_degrees;
        const double pitch = feedback.valid ? feedback.pitch : s.pitch_degrees;
        const double yaw_bound = feedback.valid ? feedback.yaw_bound : 0.;
        const double pitch_bound = feedback.valid ? feedback.pitch_bound : 0.;
        return a.mobile_window_valid && campus::mobileExecutionAligned(
            {a.mobile_yaw_limit_deg-yaw_bound, a.mobile_pitch_limit_deg-pitch_bound}, a.fire_yaw, a.fire_pitch,
            a.yaw, a.pitch, yaw, pitch,
            now().seconds() - stampSeconds(a.calculation_stamp));
    }

    bool continueBurst(Command& command, const State& s, double time) const {
        if (!anchorPositionReady(s))
            return false;
        if (!burst_ || !robotHighlandReady(burst_->number) || cancelled_goal_ || blocked_ ||
            !(phase_ == "ENGAGE" || phase_ == "REACQUIRE" || phase_ == "EVADE") ||
            !campus::burstWithinLease(time - burst_time_, now().seconds() - stampSeconds(burst_->observation_stamp),
                                      burst_hold_) ||
            burst_->header.frame_id != s.header.frame_id || burst_->state_session != session(s) ||
            burst_->frame_sequence > s.frame_sequence || s.is_weakened || s.ammo_remaining <= 0 ||
            std::hypot(s.twist.linear.x, s.twist.linear.y) / 3. > .03 ||
            std::hypot(campus::angleDifference(burst_->yaw, s.yaw_degrees), burst_->pitch - s.pitch_degrees) > 1.)
            return false;
        if (strict_ && (!mission_.recent(.5) || mission_fields_.at("stage") == "REFILL" ||
                        !campus::missionArmorAlive(mission_fields_.at("stage"), burst_->number, s.enemy_healths,
                                                   mission_fields_.count("ignored_enemy_one") &&
                                                       mission_fields_.at("ignored_enemy_one") == "1", field_order_)))
            return false;

        if (burst_->number == 6 || burst_->number == 8 || !solution_.recent(.12) || !solution_.message)
            return false;
        const auto& a = *solution_.message;
        if (a.track_id != burst_->track_id || a.slot != burst_->slot || a.number != burst_->number ||
            solutionState(s) != "READY" || !a.fire || a.reason != "ALLOW_PREDICTED" ||
            !mobileAlignmentReady(a, s))
            return false;
        // Only the solver's current, uncertainty-checked prediction can bridge a gap.
        // Reusing a frozen aim angle can shoot beside a moving/spinning armor plate.
        command.yaw = a.yaw;
        command.pitch = a.pitch;
        command.if_shoot = true;
        return true;
    }
    void aim(Command& command, const State& s, std::string& mode, std::string& reason, std::string& solution_state,
             uint32_t& target, bool defense = false, bool angle_only = false) {
        if (!anchorPositionReady(s)) {
            mode = "WAIT_VALID_AIM";
            reason = "RETURN_SENTRY_ANCHOR";
            return;
        }

        if (!structurePositionReady(s)) {
            reason = mission_fields_["stage"] == "BASE" ? "POSITION_BASE_RANGE" : "POSITION_OUTPOST_RANGE";
            mode = "WAIT_STRUCTURE_POSITION";
            return;
        }
        solution_state = solutionState(s);
        if (solution_state != "READY") {
            reason = "WAIT_SOLUTION_" + solution_state;
            mode = "WAIT_VALID_AIM";
            if (holdRecentAim(command, s, defense)) {
                mode = "HOLD_AIM_NO_FIRE";
                target = aim_hold_->track_id;
            }
            return;
        }
        const auto& a = *solution_.message;
        target = a.track_id;
        if (!robotHighlandReady(a.number)) {
            mode="WAIT_VALID_AIM";reason="ROBOT_FOUR_REQUIRES_HIGHLAND";return;
        }
        const auto stage = campus::armorSelectionStage(mission_fields_["stage"], defense);
        bool allowed = !strict_ || campus::missionArmorAlive(stage, a.number, s.enemy_healths,
                                                             mission_fields_["ignored_enemy_one"] == "1", field_order_);
        if (!allowed || (a.number == 8 && s.enemy_healths[5] > 0)) {
            reason = "MISSION_PRIORITY";
            return;
        }

        if (mobileStage(mission_fields_["stage"]) && !campus::tentativeMobilePitchAllowed(a.reason, a.pitch)) {
            reason = "TENTATIVE_HIGH_PITCH";
            mode = "WAIT_VALID_AIM";
            return;
        }
        command.yaw = a.yaw;
        command.pitch = a.pitch;
        if (mobileStage(mission_fields_["stage"]) && a.reason != "WAIT_TRACK_CONFIRM") {
            mobile_search_pitch_ = a.pitch;
            mobile_search_pitch_time_ = steadyTime() - (now().seconds() - stampSeconds(a.observation_stamp));
        }
        aim_hold_ = a;
        aim_hold_time_ = steadyTime();

        if (a.reason != "TEMP_LOST" && a.reason != "NO_FRESH_OBSERVATION" && a.reason != "ALLOW_PREDICTED" &&
            a.reason != "PREDICTION_UNCERTAIN") {

            if (!contact_ || stampSeconds(a.observation_stamp) > stampSeconds(contact_->observation_stamp))
                contact_time_ = steadyTime() - (now().seconds() - stampSeconds(a.observation_stamp));
            contact_ = a;
        }
        mode = defense ? (mission_fields_["stage"] == "REFILL" ? "REFILL_DEFENSE" : "STATIONARY_DEFENSE")
                       : (phase_ == "EVADE" ? "EVADE" : "ENGAGE");
        double speed = std::hypot(s.twist.linear.x, s.twist.linear.y) / 3.;
        if (angle_only) {
            mode = "NAVIGATION_AIM";
            reason = "NAVIGATION_NO_FIRE";
        } else if (!defense && phase_ == "REPOSITION") {
            mode = "REPOSITION_AIM";
            reason = "REPOSITION_NO_FIRE";
        } else if (s.ammo_remaining <= 0)
            reason = "NO_AMMO";
        else if (s.is_weakened)
            reason = "WEAKENED";
        else if (!std::isfinite(speed) || (defense && speed > .03))
            reason = "WAIT_CHASSIS_STOP";
        else if (!a.fire)
            reason = a.reason;
        else if (!mobileAlignmentReady(a, s))
            reason = "WAIT_MOBILE_EXECUTION_ALIGNMENT";
        else {
            command.if_shoot = true;
            reason = "ALLOW";
            burst_ = a;
            burst_time_ = steadyTime();
        }
    }

    //综合判断当前机器人处于什么状态
    void tick() {
        double time = steadyTime(), dt = std::clamp(time - last_tick_, 0., .05);
        last_tick_ = time;
        Command command = last_command_;
        command.if_shoot = false;
        double state_age_limit = strict_ && mission_.recent(.5) && mission_fields_["stage"] == "BUFF" ? .28 : .2;
        bool fresh = state_.message && campus::freshSample(steadyTime() - state_.received,
                                                           now().seconds() - stampSeconds(state_.message->header.stamp),
                                                           state_age_limit, state_age_limit);
        const State* s = fresh ? state_.message.get() : nullptr;
        bool healthy = s && s->health > 0 && s->remaining_seconds > 0 && std::isfinite(s->remaining_seconds) &&
                       std::isfinite(s->yaw_degrees) && std::isfinite(s->pitch_degrees);

        const bool rune_hold = strict_ && mission_.recent(.5) && mission_fields_["stage"]=="BUFF";
        if (healthy && !rune_hold) {
            command.yaw = s->yaw_degrees;
            command.pitch = s->pitch_degrees;
        }

        std::string mode = "HOLD", reason = "NO_TARGET", solution_state = "NOT_REQUESTED", heading_source = "NONE";
        std::optional<double> desired;
        uint32_t target = 0;
        if (count_publishers(output_->get_topic_name()) > 1)
            mode = reason = "MULTIPLE_ANGLE_PUBLISHERS";
        else if (!healthy)
            reason = s ? "DEAD_OR_INVALID_STATE" : "STALE_STATE";
        else if (!follow_.recent(.2))
            reason = "STALE_FOLLOW_STATUS";
        else if (strict_ && mission_.recent(.5) &&
                 campus::stationaryDefenseAllowed(fields(follow_.message->data)["stage"], mission_fields_["stage"],
                                                  cancelled_goal_.has_value(), s->ammo_remaining, s->is_weakened,
                                                  std::hypot(s->twist.linear.x, s->twist.linear.y) / 3.,
                                                  after_arrival_)) {
            aim(command, *s, mode, reason, solution_state, target, true);
            if (solution_state != "READY" && mode != "HOLD_AIM_NO_FIRE") {
                if (mission_fields_["stage"] == "REFILL") {
                    command.yaw = scan_.update(s->yaw_degrees, dt, scan_rate_);
                    mode = "SEARCH_REFILL_THREAT";
                    reason = "REFILL_ROUTE_BLOCKED_SEARCH";
                } else {
                    waitingSearch(command, *s, dt, mode);
                    reason = "WAIT_DEFENSE_" + solution_state;
                }
            }
        } else if (strict_ && mission_.recent(.5) && mission_fields_["stage"] == "WAIT_CLEAR_ROBOTS_POSITION" &&
                   fields(follow_.message->data)["stage"] == "WAIT_GOAL" && !cancelled_goal_ &&
                   s->ammo_remaining > 0 && !s->is_weakened &&
                   std::hypot(s->twist.linear.x, s->twist.linear.y) / 3. <= .03) {
            command.yaw = scan_.update(s->yaw_degrees, dt, scan_rate_);
            mode = "SEARCH_ROBOT_POSITION";
            reason = "WAIT_ROBOT_POSITION";
        } else if (blocked_) {
            if (strict_ && mission_.recent(.5) && phase_ == "NAVIGATE" &&
                campus::navigationSearchAllowed(fields(follow_.message->data)["stage"], mission_fields_["stage"],
                                                cancelled_goal_.has_value(), s->ammo_remaining, s->is_weakened)) {
                waitingSearch(command, *s, dt, mode);
                reason = "WAIT_SAFE_ROUTE";
            } else
                reason = mission_fields_["stage"] == "REFILL" ? "REFILL_ROUTE_BLOCKED" : "NAVIGATION_INHIBITED";
        } else if (strict_ && !mission_.recent(.5))
            reason = "WAIT_MISSION";
        else if (strict_ && identifier(mission_fields_, "goal") != goal_ &&
                 (mission_fields_["stage"] == "REFILL" || phase_ == "NAVIGATE")) {
            reason = "WAIT_MISSION_PATH";
            if (campus::navigationSearchAllowed("WAIT_MISSION_PATH", mission_fields_["stage"],
                                                cancelled_goal_.has_value(), s->ammo_remaining, s->is_weakened))
                waitingSearch(command, *s, dt, mode);
        } else if (strict_ && (mission_fields_["stage"] == "REFILL" || s->ammo_remaining <= 0 || s->is_weakened)) {
            if (mission_fields_["stage"] == "REFILL") {
                desired = travel(command, *s, dt, heading_source);
                mode = "RETURN_SPAWN_REFILL";
            }
            reason = "RETURN_SPAWN_REFILL";
        } else if(strict_ && mission_fields_["stage"]=="SENTRY_ADVANCE") {
            desired=travel(command,*s,dt,heading_source);
            mode="SENTRY_ADVANCE_TRAVEL";
            reason="MOVE_TO_HIGHLAND_FRONTIER";
        } else if (strict_ && mission_fields_["stage"] == "RUNE_RETURN") {
            desired = travel(command, *s, dt, heading_source);
            mode = "RUNE_RETURN_TRAVEL";
            reason = "RETURN_BY_DESCENT_ROUTE";
        } else if (strict_ && mission_fields_["stage"] == "BUFF") {
            mode = "RUNE_WAIT";
            reason = "WAIT_RUNE_ARRIVAL";
            auto follow = fields(follow_.message->data);
            if (follow["stage"] != "ARRIVED" || identifier(mission_fields_, "goal") != goal_) {
                desired = travel(command, *s, dt, heading_source);
                mode = "RUNE_TRAVEL";
            } else if (buff_solution_.recent(.08) && sourceFresh(buff_solution_.message->header.stamp, .12)) {
                const auto& b = *buff_solution_.message;
                if (b.goal_id == goal_ && b.header.frame_id == s->header.frame_id && b.state_session == session(*s) &&
                    b.frame_sequence >= buff_sequence_min_ && b.frame_sequence <= s->frame_sequence && b.valid &&
                    std::isfinite(b.yaw) && std::isfinite(b.pitch) && std::abs(b.pitch) <= 60) {
                    double intercept_yaw=b.yaw,intercept_pitch=b.pitch;
                    double solution_age=now().seconds()-stampSeconds(b.header.stamp);
                    const campus_buff::RuneAimTrajectory path{b.trajectory_valid,b.yaw,b.pitch,
                        b.yaw_rate,b.pitch_rate,b.yaw_acceleration,b.pitch_acceleration};
                    const bool current_intercept=!b.big_rune || path.evaluate(solution_age,intercept_yaw,intercept_pitch);
                    command.yaw = intercept_yaw;
                    command.pitch = intercept_pitch;
                    mode = "RUNE_AIM";
                    reason = b.reason;
                    const bool final_precision=b.big_rune && big_successes_.count()>=2;
                    const bool acquiring=b.big_rune && (b.reason=="WAIT_TARGET_FEEDBACK" ||
                        b.reason=="WAIT_BIG_ROTATION_MODEL" || b.reason=="ACQUIRE_PENDING_NO_FIRE");
                    if(final_request_!=b.request_id) {final_request_=b.request_id;final_trim_.reset();}
                    if(final_precision) {
                        command.yaw=std::remainder(intercept_yaw+final_trim_.yaw(),360.);
                        command.pitch=intercept_pitch+final_trim_.pitch();
                    }
                    if(b.big_rune)
                        campus_buff::continuousRuneCommand(last_command_.yaw,last_command_.pitch,command.yaw,command.pitch,dt,acquiring);
                    auto barrel=big_feedback_.fireEstimate(now().seconds(),b.command_latency,command.yaw,command.pitch,b.big_rune);
                    if(final_precision) {
                        if(barrel.valid && big_feedback_.estimateSource()==1)
                            final_trim_.observe(big_feedback_.measurementStamp(),command.yaw,command.pitch,barrel.yaw,barrel.pitch);
                        else final_trim_.decay(dt);
                        command.yaw=std::remainder(intercept_yaw+final_trim_.yaw(),360.);
                        command.pitch=intercept_pitch+final_trim_.pitch();
                        campus_buff::continuousRuneCommand(last_command_.yaw,last_command_.pitch,command.yaw,command.pitch,dt,acquiring);
                        barrel=big_feedback_.fireEstimate(now().seconds(),b.command_latency,command.yaw,command.pitch,true);
                    }
                    const double alignment_limit=b.big_rune
                        ? campus_buff::finalRuneAlignmentLimit(final_precision,b.horizon,
                            stampSeconds(b.header.stamp)-stampSeconds(b.observation_stamp),b.command_latency,s->projectile_speed)
                        : .80;
                    bool allow = current_intercept && barrel.valid && b.fire && b.shot_id != 0 && b.request_id != 0 &&
                                 sourceFresh(b.observation_stamp, .28) &&
                                 std::hypot(s->twist.linear.x, s->twist.linear.y) / 3. < .03 &&
                                 std::hypot(campus::angleDifference(intercept_yaw, barrel.yaw), intercept_pitch - barrel.pitch) < alignment_limit;
                    if(b.big_rune && b.fire && !allow)
                        reason=!current_intercept?"BIG_WAIT_CURRENT_INTERCEPT":
                            (barrel.valid?"BIG_WAIT_BARREL_ALIGNMENT":"BIG_WAIT_MEASURED_TRACK_"+std::to_string(big_feedback_.rejection()));
                    if (allow && rune_shots_.consume(b.shot_id, steadyTime())) {
                        rune_shot_ammo_ = s->ammo_remaining;
                        rune_shot_sent_ = b.shot_id;
                        rune_shot_goal_ = b.goal_id;
                        rune_shot_request_ = b.request_id;
                        rune_shot_yaw_ = command.yaw;
                        rune_shot_pitch_ = command.pitch;
                        rune_shot_observation_ = b.observation_stamp;
                        RCLCPP_INFO(get_logger(), "RUNE_SINGLE_SHOT id=%lu request=%lu final_precision=%d error_deg=%.4f limit_deg=%.4f trim_yaw=%.4f trim_pitch=%.4f solution_age=%.4f",
                            b.shot_id,b.request_id,final_precision,
                            std::hypot(campus::angleDifference(intercept_yaw,barrel.yaw),intercept_pitch-barrel.pitch),alignment_limit,
                            final_trim_.yaw(),final_trim_.pitch(),solution_age);
                    } else if (allow && b.shot_id == rune_shots_.last()) {
                        std_msgs::msg::UInt64 ack;
                        ack.data = b.shot_id;
                        rune_shot_pub_->publish(ack);
                    }

                    command.if_shoot = rune_shots_.active(steadyTime()) && b.goal_id == rune_shot_goal_ &&
                                       b.request_id == rune_shot_request_ && sourceFresh(rune_shot_observation_, .28) &&
                                       std::hypot(s->twist.linear.x, s->twist.linear.y) / 3. < .03;

                    if (command.if_shoot) {
                        command.yaw = rune_shot_yaw_;
                        command.pitch = rune_shot_pitch_;
                    }
                } else
                    reason = "INVALID_RUNE_SOLUTION";
            } else
                reason = "STALE_RUNE_SOLUTION";
        } else if (!structurePositionReady(*s)) {
            desired = travel(command, *s, dt, heading_source);
            mode = "TRAVEL_HEADING";
            reason = mission_fields_["stage"] == "BASE" ? "POSITION_BASE_RANGE" : "POSITION_OUTPOST_RANGE";
        } else {
            bool engage = (!after_arrival_ || ready_) &&
                          ((after_arrival_ && ready_) || phase_ == "ENGAGE" || phase_ == "ACQUIRE" ||
                           phase_ == "REACQUIRE" || phase_ == "EVADE" || phase_ == "REPOSITION");
            if (engage) {
                aim(command, *s, mode, reason, solution_state, target);
            } else if (scan_enabled_)
                scanning(command, *s, dt, mode);
            else {
                desired = travel(command, *s, dt, heading_source);
                if (desired)
                    mode = "TRAVEL_HEADING";
            }

            if (after_arrival_ && !ready_) {
                command.if_shoot = false;
                reason = "WAIT_ARRIVAL";
            }
        }
        const bool robots_cleared =
            s && campus::robotCleanupComplete(s->enemy_healths, mission_fields_["ignored_enemy_one"] == "1");
        const bool mobile_stage = mobileStage(mission_fields_["stage"]) || (strict_ && !robots_cleared);
        bool navigation_observation = false;
        // Navigation may advance/replan while a robot remains visible. Keep
        // tracking its plate instead of swinging to the chassis/route bearing.
        // Use the native solver during a valid active route as in manual
        // chassis mode; goal changes and navigation waits remain angle-only.
        if (healthy && follow_.recent(.2) && mission_.recent(.5) && mobile_stage && !cancelled_goal_ &&
            !after_arrival_ && s->ammo_remaining > 0 && !s->is_weakened &&
            (mode == "TRAVEL_HEADING" || mode == "SCAN" || mode == "SEARCH_ROBOT_POSITION" ||
             mode == "SEARCH_NAVIGATION_WAIT")) {
            navigation_observation = true;
            auto saved = command;
            auto saved_mode = mode, saved_reason = reason;
            const bool moving_fire = campus::navigationFireAllowed(
                fields(follow_.message->data)["stage"], identifier(mission_fields_, "goal") == goal_,
                ready_ || mission_fields_["stage"] == "OPENING_RAMP");
            aim(command, *s, mode, reason, solution_state, target, false, !moving_fire);
            if (mode == "ENGAGE")
                mode = "NAVIGATION_AIM";
            if (!moving_fire)
                command.if_shoot = false;
            if (mode != "NAVIGATION_AIM" && mode != "HOLD_AIM_NO_FIRE") {
                command = saved;
                mode = saved_mode;
                reason = saved_reason;
            }
        }

        if (healthy && follow_.recent(.2) && (phase_ == "REACQUIRE" || mobile_stage) &&
            (mode == "WAIT_VALID_AIM" || mode == "HOLD_AIM_NO_FIRE" || mode == "SEARCH_NAVIGATION_WAIT" ||
             (mobile_stage && (mode == "TRAVEL_HEADING" || mode == "SCAN" || mode == "SEARCH_ROBOT_POSITION"))) &&
            reacquire(command, *s, time, dt)) {
            bool holding = time - contact_time_ <= campus::kReacquireHold;
            mode = holding ? "HOLD_AIM_NO_FIRE" : "REACQUIRE_SEARCH";
            reason = holding ? "BRIEF_OBSERVATION_GAP" : "TARGET_REACQUIRE";
            target = contact_->track_id;
        }

        if (healthy && follow_.recent(.2) && !navigation_observation && !command.if_shoot &&
            (mode == "WAIT_VALID_AIM" || mode == "HOLD_AIM_NO_FIRE" || mode == "ENGAGE" || mode == "EVADE" ||
             mode == "REACQUIRE_SEARCH") &&
            structurePositionReady(*s) && continueBurst(command, *s, time)) {
            mode = "BURST_CONTINUE";
            reason = "PREDICTED_LOCK_BURST";
            target = burst_->track_id;
        }

        bool search_mode =
            (mode == "TRAVEL_HEADING" || mode == "SCAN" || mode == "SEARCH_OBJECTIVE" ||
             mode == "SEARCH_ROBOT_POSITION" || mode == "SEARCH_NAVIGATION_WAIT" || mode == "SEARCH_REFILL_THREAT");
        bool searching = pitch_search_enabled_ && mission_fields_["stage"] == "OUTPOST" && search_mode &&
                         (!strict_ || robots_cleared);
        if (searching) {
            command.pitch = pitch_search_.update(s->pitch_degrees, dt, -45., 8., pitch_search_rate_);
        } else if (mode != "HOLD" || reason == "DEAD_OR_INVALID_STATE" || cancelled_goal_) {
            // A short transport hold must not restart each sweep upward. Keep
            // its phase, while the held command and fire remain inhibited.
            pitch_search_.reset();
        }
        const bool expired_aim_wait =
            (mode == "WAIT_VALID_AIM" || mode == "HOLD_AIM_NO_FIRE" || mode == "REACQUIRE_SEARCH") &&
            time - contact_time_ > campus::kReacquireTimeout;
        if (healthy && mobile_stage && (search_mode || expired_aim_wait)) {
            double pitch = campus::mobileSearchPitch(mobile_search_pitch_, time - mobile_search_pitch_time_);
            command.pitch = last_command_.pitch + std::clamp(pitch - last_command_.pitch, -60. * dt, 60. * dt);
        }

        const bool sentry_stage=strict_ && mission_.recent(.5) && s &&
            campus::sentrySearchStage(mission_fields_["stage"],s->enemy_healths,mission_fields_["ignored_enemy_one"]=="1");
        if(mission_.recent(.5) && !sentry_stage) sentry_search_centre_.reset();
        // A real new sentry observation may establish a new search centre.
        // Public map updates and search/replan ticks cannot drag it around.
        if(healthy && sentry_stage && contact_ && contact_->number==7 && time-contact_time_<.2 &&
           (mode=="ENGAGE" || mode=="NAVIGATION_AIM" || mode=="BURST_CONTINUE")) {
            sentry_search_centre_=contact_->yaw;sentry_search_since_=time;
        }
        if(healthy && sentry_stage && phase_=="NAVIGATE" && follow_.recent(.2) &&
           fields(follow_.message->data)["stage"]=="FOLLOWING" &&
           (mode=="SCAN" || mode=="SEARCH_NAVIGATION_WAIT")) {
            desired=travel(command,*s,dt,heading_source);
            if(desired) {mode="TRAVEL_HEADING";reason="SENTRY_POSITION_TRANSIT";}
        }
        const bool travelling_to_sentry=mode=="TRAVEL_HEADING" && follow_.recent(.2) &&
            fields(follow_.message->data)["stage"]=="FOLLOWING";
        if(healthy && sentry_stage && !travelling_to_sentry && !command.if_shoot &&
           (search_mode || mode=="REACQUIRE_SEARCH" || mode=="WAIT_VALID_AIM") &&
           mode!="HOLD_AIM_NO_FIRE") {
            if(!sentry_search_centre_) {
                auto bearing=objectiveBearing(*s);
                sentry_search_centre_=bearing.value_or(s->yaw_degrees);
                sentry_search_since_=time;
            }
            command.yaw=campus::boundedSentrySearch(last_command_.yaw,*sentry_search_centre_,time-sentry_search_since_,dt);
            mode="SEARCH_SENTRY_SECTOR";
            reason="BOUNDED_SENTRY_BEARING";
        }

        if (mode != "SCAN" && mode != "SEARCH_OBJECTIVE" && mode != "SEARCH_ROBOT_POSITION" &&
            mode != "SEARCH_NAVIGATION_WAIT" && mode != "SEARCH_REFILL_THREAT")
            scan_.reset();
        if ((mode != "TRAVEL_HEADING" && mode != "RETURN_SPAWN_REFILL" && mode != "RUNE_TRAVEL" && mode!="RUNE_RETURN_TRAVEL" && mode!="SENTRY_ADVANCE_TRAVEL") || !desired) {
            heading_.reset();
            feedback_anchor_.reset();
            no_response_ = false;
        } else if (!feedback_anchor_) {
            feedback_anchor_ = s->yaw_degrees;
            feedback_time_ = time;
        } else if (time - feedback_time_ >= 1.) {
            no_response_ = std::abs(campus::angleDifference(*desired, s->yaw_degrees)) > 20 &&
                           std::abs(campus::angleDifference(s->yaw_degrees, *feedback_anchor_)) < 2;
            feedback_anchor_ = s->yaw_degrees;
            feedback_time_ = time;
        }

        if (mode == "MULTIPLE_ANGLE_PUBLISHERS" && last_mode_ != mode)
            RCLCPP_ERROR(get_logger(), "Conflicting gimbal owners on %s", output_->get_topic_name());
        if (strict_ && mission_fields_["stage"]=="BUFF" && command.if_shoot &&
            !sourceFresh(rune_shot_observation_,.28)) {
            rune_shots_.cancel();command.if_shoot=false;reason="STALE_RUNE_AT_OUTPUT";
        }
        last_mode_ = mode;
        last_command_ = command;
        // Emit the true edge once; preserve the accepted aim during its short
        // lease, then publish false. Callback bursts cannot stack true packets.
        if (!(strict_ && mission_fields_["stage"]=="BUFF" && command.if_shoot) || rune_shots_.emit(steadyTime())) {
            if(strict_ && mission_fields_["stage"]=="BUFF" && buff_solution_.message)
                big_feedback_.command(now().seconds()+buff_solution_.message->command_latency,command.yaw,command.pitch);
            output_->publish(command);
        }
        if (rune_shot_sent_) {
            std_msgs::msg::UInt64 ack;
            ack.data = rune_shot_sent_;
            rune_shot_pub_->publish(ack);
            rune_shot_sent_ = 0;
        }

        std::ostringstream out;
        out << std::fixed << std::setprecision(2) << mode << " target=" << target
            << " fire=" << (command.if_shoot ? "True" : "False") << " yaw=" << command.yaw << " pitch=" << command.pitch
            << " fire_reason=" << reason << " solution=" << solution_state << " nav_target=" << navigation_target_
            << " goal=" << goal_ << " combat_ready=" << (ready_ ? "true" : "false") << " implementation=CPP"
            << " pitch_search=" << (searching ? "true" : "false");
        if (healthy)
            out << " measured_yaw=" << s->yaw_degrees << " measured_pitch=" << s->pitch_degrees;
        out << " mission=" << mission_fields_["stage"]
            << " follow_stage=" << (follow_.message ? fields(follow_.message->data)["stage"] : "NONE");
        if (healthy)
            out << " ammo=" << s->ammo_remaining << " health=" << s->health << " weakened=" << s->is_weakened
                << " measured_speed_mps=" << std::hypot(s->twist.linear.x, s->twist.linear.y) / 3.;
        if (solution_.message)
            out << " solver_reason=" << solution_.message->reason;
        if (mode == "REACQUIRE_SEARCH")
            out << " contact_age_s=" << time - contact_time_ << " contact_number=" << contact_->number;
        if (desired)
            out << " desired_yaw=" << *desired
                << " heading_error_deg=" << campus::angleDifference(*desired, s->yaw_degrees)
                << " heading_source=" << heading_source << " heading_feedback="
                << (no_response_
                        ? "NO_RESPONSE"
                        : (std::abs(campus::angleDifference(*desired, s->yaw_degrees)) < 5 ? "ALIGNED" : "TURNING"));
        Text message;
        message.data = out.str();
        status_->publish(message);
        std::string transition = mode + " " + reason + " " + solution_state;
        if (transition != last_transition_) {
            RCLCPP_INFO(get_logger(), "%s", message.data.c_str());
            last_transition_ = transition;
        }
    }

    campus::RuneGimbalFeedback buff_feedback_;
    campus::MobileGimbalFeedback mobile_feedback_;
    bool mobile_feedback_enabled_ = false;
    double mobile_feedback_delay_ = .04;
    campus_buff::BigRuneGimbal big_feedback_;
    double buff_match_start_stamp_=0;
    campus_buff::BigRuneSuccesses big_successes_;
    campus_buff::FinalRuneTrim final_trim_;
    uint64_t final_request_=0;
    campus::RuneSingleShot rune_shots_;
    int rune_shot_ammo_ = 0;
    uint64_t rune_shot_sent_ = 0, rune_shot_goal_ = 0, rune_shot_request_ = 0;
    double rune_shot_yaw_ = 0, rune_shot_pitch_ = 0;
    builtin_interfaces::msg::Time rune_shot_observation_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr rune_shot_pub_;
    Sample<campus_aim_interfaces::msg::BuffSolution> buff_solution_;
    rclcpp::Subscription<campus_aim_interfaces::msg::BuffSolution>::SharedPtr buff_sub_;
    rclcpp::Subscription<campus_aim_interfaces::msg::BuffStatus>::SharedPtr buff_status_sub_;
    Sample<State> state_;
    Sample<Solution> solution_;
    Sample<Speed> speed_;
    std::optional<Solution> aim_hold_;
    double aim_hold_time_ = -INFINITY;
    std::optional<Solution> contact_;
    double contact_time_ = -INFINITY;
    std::optional<Solution> burst_;
    double burst_time_ = -INFINITY, burst_hold_ = .12;
    Sample<Text> follow_, mission_;
    Fields mission_fields_{{"stage", ""}};
    bool scan_enabled_ = false, after_arrival_ = false, strict_ = false, ready_ = false, blocked_ = true,
         no_response_ = false;
    bool field_order_ = false;
    bool pitch_search_enabled_ = true;
    double mobile_search_pitch_ = 0., mobile_search_pitch_time_ = -INFINITY;
    double pitch_search_rate_ = 12.;
    double outpost_source_age_ = .28, outpost_coast_ = .1;
    double scan_rate_ = 45, solution_timeout_ = .2, mobile_coast_ = .25, last_tick_ = 0, feedback_time_ = 0;
    uint64_t buff_sequence_min_ = 0;
    uint64_t goal_ = 0;
    uint32_t navigation_target_ = 0;
    std::optional<uint64_t> cancelled_goal_;
    double sentry_search_since_=0;
    std::optional<double> sentry_search_centre_;
    std::optional<double> feedback_anchor_;
    std::vector<double> matrix_;
    std::string phase_ = "NAVIGATE", last_mode_, last_transition_;
    campus::TravelHeading heading_;
    campus::ScanHeading scan_;
    campus::PitchSearch pitch_search_;
    Command last_command_;
    rclcpp::Publisher<Command>::SharedPtr output_;
    rclcpp::Publisher<Text>::SharedPtr status_;
    rclcpp::Subscription<State>::SharedPtr state_sub_;
    rclcpp::Subscription<Solution>::SharedPtr solution_sub_;
    rclcpp::Subscription<Speed>::SharedPtr speed_sub_;
    rclcpp::Subscription<Text>::SharedPtr follow_sub_, mission_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

namespace {
volatile std::sig_atomic_t stopped = 0;

void stopSignal(int) {
    stopped = 1;
}
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, stopSignal);
    std::signal(SIGTERM, stopSignal);
    auto node = std::make_shared<CombatGimbal>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !stopped) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (rclcpp::ok())
        for (int i = 0; i < 3; ++i) {
            node->stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    rclcpp::shutdown();
    return 0;
}
