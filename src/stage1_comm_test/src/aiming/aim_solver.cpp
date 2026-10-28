#include "aiming/ballistics.hpp"
#include "aiming/fire_control.hpp"
#include "aiming/mobile_gimbal_feedback.hpp"
#include "aiming/outpost_aim.hpp"
#include "common/ros_aim_utils.hpp"
#include "aiming/gimbal_stabilizer.hpp"
#include "aiming/shooter_motion.hpp"
#include "classification/armor_identity.hpp"
#include <functional>
#include <campus_aim_interfaces/msg/track_frame.hpp>
#include <campus_aim_interfaces/msg/aim_solution.hpp>
#include <tdt_interface/msg/send_data.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>

//将 Tracker 输出的运动状态转换为预测瞄准角度，并判断射击条件
class AimSolverNode : public rclcpp::Node {
public:
    AimSolverNode() : Node("aim_solver") {
        scale_ = campus::numberParam(*this, "scene_units_per_meter", 3);
        offset_ = campus::offsetParams(*this);
        muzzle_offset_[0] = campus::numberParam(*this, "muzzle.offset_x_m", 0);
        muzzle_offset_[1] = campus::numberParam(*this, "muzzle.offset_y_m", 0);
        muzzle_offset_[2] = campus::numberParam(*this, "muzzle.offset_z_m", 0.08333333333333333);

        latency_ = campus::numberParam(*this, "prediction_time_s", 0.04);
        mobile_feedback_enabled_ = declare_parameter<bool>("command.mobile_feedback_compensation", false);
        max_age_ = campus::numberParam(*this, "target_timeout_ms", 200) / 1000;
        gravity_ = campus::numberParam(*this, "gravity_mps2", 9.81);

        default_speed_ = campus::numberParam(*this, "default_projectile_speed_mps", 23.5);
        max_horizon_ = campus::numberParam(*this, "max_prediction_time_s", 1.5);
        allow_temp_lost_ = declare_parameter<bool>("allow_temp_lost", true);

        allow_detecting_aim_ = declare_parameter<bool>("selection.allow_detecting_aim", false);

        require_number_ = declare_parameter<bool>("require_number", true);
        target_number_ = campus::intParam(*this, "target_number", 0);
        max_variance_ = campus::numberParam(*this, "max_position_variance", 0.09);

        display_ = declare_parameter<bool>("display", false);
        show_prediction_ = declare_parameter<bool>("display.show_prediction", false);
        display_every_ = std::max(1, campus::intParam(*this, "display_every_n_frames", 2));

        command_enabled_ = declare_parameter<bool>("command.enabled", false);
        shoot_enabled_ = declare_parameter<bool>("command.shoot", false);
        fire_require_number_ = declare_parameter<bool>("command.fire_require_number", true);

        player_ = campus::intParam(*this, "player", 1);

        fire_tolerance_ = campus::numberParam(*this, "command.fire_tolerance_deg", 1.0);
        fire_settle_ = campus::numberParam(*this, "command.fire_settle_time_s", 0.02);
        mobile_fire_coast_ = campus::numberParam(*this, "command.mobile_fire_coast_ms", 250.) / 1000.;

        if (!std::isfinite(mobile_fire_coast_) || mobile_fire_coast_ < 0 || mobile_fire_coast_ > .25)
            throw std::invalid_argument("command.mobile_fire_coast_ms must be 0..250");

        mobile_prediction_sigma_ = campus::numberParam(*this, "command.mobile_prediction_sigma_m", .20);

        if (!std::isfinite(mobile_prediction_sigma_) || mobile_prediction_sigma_ < .02 ||
            mobile_prediction_sigma_ > .25)
            throw std::invalid_argument("command.mobile_prediction_sigma_m must be .02.. .25");

        mobile_window_scale_ = campus::numberParam(*this, "command.mobile_plate_window_scale", 1.0);

        if (!std::isfinite(mobile_window_scale_) || mobile_window_scale_ < .5 || mobile_window_scale_ > 1.5)
            throw std::invalid_argument("command.mobile_plate_window_scale must be .5..1.5 (clamped to physical size)");

        use_plate_window_ = declare_parameter<bool>("command.fire_use_plate_window", false);
        require_current_plate_ = declare_parameter<bool>("command.fire_require_current_plate", false);

        fixed_outpost_ = declare_parameter<bool>("outpost.fixed_aim.enabled", true);
        outpost_min_rate_ = campus::numberParam(*this, "outpost.fixed_aim.min_yaw_rate_rad_s", 0.5);
        multiple_outpost_heights_ = declare_parameter<bool>("outpost.fixed_aim.multiple_heights", true);
        outpost_fire_coast_ = campus::numberParam(*this, "outpost.fire_coast_ms", 60) / 1000;
        outpost_max_phase_sigma_ = campus::numberParam(*this, "outpost.max_phase_sigma_rad", 0.12);
        outpost_max_center_sigma_ = campus::numberParam(*this, "outpost.max_center_sigma_m", 0.03);
        outpost_phase_noise_ = campus::numberParam(*this, "outpost.phase_acceleration_noise", 0.08);

        control_period_ = campus::intParam(*this, "command.control_period_ms", 10);

        if (control_period_ < 5 || control_period_ > 50 || !std::isfinite(outpost_fire_coast_) ||
            outpost_fire_coast_ < 0 || outpost_fire_coast_ > max_age_ || !std::isfinite(outpost_max_phase_sigma_) ||
            outpost_max_phase_sigma_ <= 0 || !std::isfinite(outpost_phase_noise_) || outpost_phase_noise_ < 0 ||
            !std::isfinite(outpost_max_center_sigma_) || outpost_max_center_sigma_ <= 0) {
            throw std::invalid_argument("Invalid outpost timing parameters");
        }

        outpost_age_ = campus::numberParam(*this, "outpost.target_timeout_ms", 280.) / 1000.;
        outpost_fire_tolerance_ = campus::numberParam(*this, "outpost.fire_tolerance_deg", 1.18);
        outpost_settle_ = campus::numberParam(*this, "outpost.fire_settle_time_s", 0.);
        outpost_game_alignment_ = declare_parameter<bool>("outpost.delegate_alignment_to_game", false);

        if (!std::isfinite(outpost_age_) || outpost_age_ < max_age_ || outpost_age_ > .3 ||
            !std::isfinite(outpost_fire_tolerance_) || outpost_fire_tolerance_ <= 0 || outpost_fire_tolerance_ > 1.19 ||
            !std::isfinite(outpost_settle_) || outpost_settle_ < 0 || outpost_settle_ > .1)
            throw std::invalid_argument("Invalid outpost firing limits");

        outpost_number_ = campus::intParam(*this, "outpost_number", 6);

        slot_hysteresis_ = declare_parameter<bool>("selection.slot_hysteresis", true);
        slot_switch_margin_ = campus::numberParam(*this, "selection.slot_switch_margin", 0.08);
        min_facing_ = campus::numberParam(*this, "selection.min_facing", 0.2);
        campus::GimbalStabilizerConfig stabilizer_config;
        stabilizer_config.enabled = declare_parameter<bool>("command.stabilizer.enabled", true);
        stabilizer_config.time_constant_s = campus::numberParam(*this, "command.stabilizer.time_constant_ms", 0) / 1000;
        stabilizer_config.yaw_deadband_deg = campus::numberParam(*this, "command.stabilizer.yaw_deadband_deg", 0.03);
        stabilizer_config.pitch_deadband_deg =
            campus::numberParam(*this, "command.stabilizer.pitch_deadband_deg", 0.03);
        stabilizer_config.max_yaw_rate_deg_s = campus::numberParam(*this, "command.stabilizer.max_yaw_rate_deg_s", 180);
        stabilizer_config.max_pitch_rate_deg_s =
            campus::numberParam(*this, "command.stabilizer.max_pitch_rate_deg_s", 120);
        stabilizer_ = std::make_unique<campus::GimbalStabilizer>(stabilizer_config);

        if (!std::isfinite(slot_switch_margin_) || slot_switch_margin_ < 0 || !std::isfinite(min_facing_) ||
            min_facing_ < 0 || min_facing_ > 1 || !std::isfinite(fire_tolerance_) || fire_tolerance_ <= 0) {
            throw std::invalid_argument("Invalid selection/command parameters");
        }

        if (scale_ <= 0 || latency_ < 0 || max_age_ <= 0 || gravity_ < 0 || default_speed_ <= 0 || max_horizon_ <= 0 ||
            (player_ != 1 && player_ != 2)) {
            throw std::invalid_argument("Invalid solver parameters");
        }

        double parameters[] = {scale_,       latency_,      max_age_,        gravity_,    default_speed_,
                               max_horizon_, max_variance_, fire_tolerance_, fire_settle_};

        for (double value : parameters) {
            if (!std::isfinite(value) || value < 0) {
                throw std::invalid_argument("Solver parameters must be finite and nonnegative");
            }
        }

        if (fire_tolerance_ > 1.2 || !campus::finite(offset_) || !campus::finite(muzzle_offset_) ||
            !std::isfinite(outpost_min_rate_) || outpost_min_rate_ <= 0 ||
            !campus::knownArmorIdentity(outpost_number_)) {
            throw std::invalid_argument("Invalid firing tolerance or camera/muzzle offsets");
        }

        angles_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/aim/debug_angles", campus::latestQoS());
        state_pub_ = create_publisher<std_msgs::msg::String>("/aim/state", campus::latestQoS());
        fire_state_pub_ = create_publisher<std_msgs::msg::String>("/aim/fire_state", campus::latestQoS());
        aim_point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/aim/point", campus::latestQoS());
        command_angles_pub_ =
            create_publisher<geometry_msgs::msg::Vector3Stamped>("/aim/command_angles", campus::latestQoS());
        selection_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/aim/selection", campus::latestQoS());
        residual_pub_ =
            create_publisher<geometry_msgs::msg::Vector3Stamped>("/aim/stabilizer_error", campus::latestQoS());

        if (command_enabled_) {
            command_pub_ = create_publisher<tdt_interface::msg::SendData>(
                declare_parameter<std::string>("command.output_topic",
                                               "/target_angles_player_" + std::to_string(player_)),
                campus::latestQoS());
        }

        framed_command_pub_ =
            create_publisher<geometry_msgs::msg::Vector3Stamped>("/aim/framed_command", campus::latestQoS());

        solution_pub_ = create_publisher<campus_aim_interfaces::msg::AimSolution>("/aim/solution", campus::latestQoS());

        targets_sub_ = create_subscription<campus_aim_interfaces::msg::TrackFrame>(
            "/tracker/targets", campus::latestQoS(),
            std::bind(&AimSolverNode::targetsCallback, this, std::placeholders::_1));

        feedback_sub_ = create_subscription<campus::Frame>(
            "/autoaim/frame", campus::latestQoS(),
            std::bind(&AimSolverNode::feedbackCallback, this, std::placeholders::_1));
        control_timer_ =
            create_wall_timer(std::chrono::milliseconds(control_period_), std::bind(&AimSolverNode::controlTick, this));
        watchdog_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&AimSolverNode::checkTimeout, this));

        if (display_) {
            cv::namedWindow("aim_solver", cv::WINDOW_NORMAL);
            cv::resizeWindow("aim_solver", 960, 720);
        }

        RCLCPP_INFO(get_logger(), "Ballistic solver: gravity %.2f, fallback speed %.2f m/s, command=%d shoot=%d",
                    gravity_, default_speed_, command_enabled_, shoot_enabled_);
    }

    ~AimSolverNode() override {
        if (display_) {
            cv::destroyWindow("aim_solver");
        }
    }

private:
    void publishFireState(const std::string& text) {
        std_msgs::msg::String state;
        state.data = "FIRE: " + text;
        fire_state_pub_->publish(state);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "%s", state.data.c_str());
    }

    double targetAgeLimit(const campus_aim_interfaces::msg::TrackFrame& packet) const {
        return !packet.targets.empty() && std::all_of(packet.targets.begin(), packet.targets.end(),
                                                      [this](const auto& t) { return t.number == outpost_number_; })
                   ? outpost_age_
                   : max_age_;
    }//根据目标类型确定允许的数据时效

    //接收Tracker发布的目标运动状态
    void targetsCallback(campus_aim_interfaces::msg::TrackFrame::ConstSharedPtr packet) {
        bool reset = false;
        const campus::Frame& f = packet->frame;
        bool valid = guard_.check(f, now().seconds(), targetAgeLimit(*packet), reset) && campus::validCamera(f);

        if (!valid) {
            if (reset || packet->reset || !campus::validCamera(f)) {
                cached_targets_.reset();
                invalidate(f, "INVALID_FRAME");
                last_receive_ = -1;
            }
            return;
        }

        if (feedback_blocked_ || campus::stampSeconds(f.header.stamp) < feedback_barrier_time_) {
            cached_targets_.reset();
            return;
        }

        last_receive_ = campus::steadySeconds();
        last_frame_ = f;

        if (reset || packet->reset) {
            cached_targets_.reset();
            invalidate(f, "RESET");
            return;
        }

        cached_targets_ = packet;
        process(*packet, true);
    }

    //接收当前机器人和云台的状态反馈
    void feedbackCallback(campus::Frame::ConstSharedPtr frame) {
        bool reset = false;
        bool valid = feedback_guard_.check(*frame, now().seconds(), max_age_, reset);

        if (reset || frame->state.health <= 0) {
            shooter_motion_.reset();
            mobile_feedback_.reset();
            feedback_blocked_ = true;
            feedback_barrier_time_ = now().seconds();
            cached_targets_.reset();
            invalidate(*frame, reset ? "FEEDBACK_RESET" : "DEAD");
        }

        if (valid && campus::validCamera(*frame)) {
            mobile_feedback_.update(campus::stampSeconds(frame->state.header.stamp),
                                    frame->state.yaw_degrees, frame->state.pitch_degrees);
            shooter_motion_.update(campus::stampSeconds(frame->state.header.stamp),
                {frame->state.twist.linear.x/scale_,frame->state.twist.linear.y/scale_,frame->state.twist.linear.z/scale_});
            feedback_frame_ = frame;

            if (!reset && frame->state.health > 0) {
                feedback_blocked_ = false;
            }
        }
    }

    void controlTick() {
        if (!cached_targets_) {
            return;
        }

        double age = now().seconds() - campus::stampSeconds(cached_targets_->frame.header.stamp);

        if (age > targetAgeLimit(*cached_targets_)) {
            invalidate(cached_targets_->frame, "STALE");
            cached_targets_.reset();
            last_receive_ = -1;
            return;
        }

        process(*cached_targets_, false);
    }//定时进行控制更新

    void checkTimeout() {
        if (last_receive_ > 0 &&
            campus::steadySeconds() - last_receive_ > (cached_targets_ ? targetAgeLimit(*cached_targets_) : max_age_)) {
            cached_targets_.reset();
            invalidate(last_frame_, "STALE");
            last_receive_ = -1;
        }
    }//检查跟踪数据是否超时

    static campus::TargetMotion motion(const campus_aim_interfaces::msg::Target& t) {
        campus::TargetMotion m;
        m.center = {t.center.x, t.center.y, t.center.z};
        m.velocity = {t.velocity.x, t.velocity.y, t.velocity.z};
        m.yaw = t.yaw;
        m.yaw_rate = t.yaw_rate;
        m.radius = t.radius;
        m.radius_odd = t.radius_odd;
        m.dz = t.height_difference;
        m.dz_second = t.height_difference_second;
        m.count = t.armor_count;
        return m;
    }//将 ROS 目标消息转换成内部运动模型

    void invalidate(const campus::Frame& f, const std::string& reason) {
        mobile_fire_.clear();

        publishSolution(f, f.header.stamp, 0, 0, -1, 0, 0, false, false, reason);
        fire_confirmation_.clear();
        geometry_msgs::msg::PointStamped point;
        point.header = f.header;
        point.header.frame_id = "map";
        point.point.x = point.point.y = point.point.z = std::numeric_limits<double>::quiet_NaN();
        aim_point_pub_->publish(point);
        selected_id_ = 0;
        selected_slot_ = -1;
        outpost_slot_ = -1;
        stabilizer_->reset();
        geometry_msgs::msg::Vector3Stamped angles;
        angles.header = f.header;
        angles.header.frame_id = "world_optical_angles";
        angles.vector.x = angles.vector.y = std::numeric_limits<double>::quiet_NaN();
        angles.vector.z = 0;
        angles_pub_->publish(angles);
        command_angles_pub_->publish(angles);
        residual_pub_->publish(angles);
        geometry_msgs::msg::Vector3Stamped selection;
        selection.header = f.header;
        selection.header.frame_id = "track_slot_score";
        selection.vector.x = 0;
        selection.vector.y = -1;
        selection.vector.z = std::numeric_limits<double>::quiet_NaN();
        selection_pub_->publish(selection);
        geometry_msgs::msg::Vector3Stamped revoked = angles;
        revoked.vector.z = -1;
        framed_command_pub_->publish(revoked);
        std_msgs::msg::String state;
        state.data = reason;
        state_pub_->publish(state);

        publishFireState("BLOCK " + reason);
        const campus::Frame* hold = &f;

        if (!std::isfinite(f.state.yaw_degrees) || !std::isfinite(f.state.pitch_degrees)) {
            hold = &last_frame_;
        }

        if (command_pub_ && std::isfinite(hold->state.yaw_degrees) && std::isfinite(hold->state.pitch_degrees)) {
            tdt_interface::msg::SendData command;
            command.yaw = hold->state.yaw_degrees;
            command.pitch = std::clamp(double(hold->state.pitch_degrees), -60.0, 60.0);
            command.if_shoot = false;
            command_pub_->publish(command);
        }
    }

    void process(const campus_aim_interfaces::msg::TrackFrame& packet, bool new_packet) {
        const campus::Frame& source = packet.frame;
        const campus::Frame* feedback = &source;
        if (feedback_frame_ && feedback_frame_->header.frame_id == source.header.frame_id &&
            campus::sessionKey(*feedback_frame_) == campus::sessionKey(source) &&
            campus::stampSeconds(feedback_frame_->header.stamp) >= campus::stampSeconds(source.header.stamp)) {
            feedback = feedback_frame_.get();
        }
        const campus::Frame& f = *feedback;

        double feedback_age = now().seconds() - campus::stampSeconds(f.state.header.stamp);

        if (!std::isfinite(feedback_age) || feedback_age < -.1 || feedback_age > max_age_) {
            invalidate(f, "STALE_FEEDBACK");
            return;
        }

        if (!campus::finite({f.state.twist.linear.x, f.state.twist.linear.y, f.state.twist.linear.z})) {
            invalidate(f, "INVALID_MOTION");
            return;
        }

        double age = std::max(0.0, now().seconds() - campus::stampSeconds(source.header.stamp));
        double speed = default_speed_;

        if (std::isfinite(f.state.projectile_speed) && f.state.projectile_speed > 0) {
            speed = f.state.projectile_speed / scale_;
        }

        campus::CameraGeometry camera = campus::cameraGeometry(source, scale_, offset_);
        cv::Vec3d pivot(f.state.position.x / scale_, f.state.position.y / scale_, f.state.position.z / scale_);
        cv::Vec3d self_velocity(f.state.twist.linear.x / scale_, f.state.twist.linear.y / scale_,
                                f.state.twist.linear.z / scale_);

        double feedback_offset = campus::stampSeconds(source.header.stamp) - campus::stampSeconds(f.state.header.stamp);
        pivot += self_velocity * feedback_offset;

        pivot += shooter_motion_.releaseCorrection(feedback_age+latency_);
        const campus_aim_interfaces::msg::Target* selected = nullptr;

        campus::InterceptSolution aim;
        bool fixed_outpost_aim = false;
        double best_cost = INFINITY;
        std::string selection_failure = packet.targets.empty() ? "NO_TARGET" : "TARGET_FILTERED";

        if (f.state.health > 0) {
            for (size_t target_index = 0; target_index < packet.targets.size(); ++target_index) {
                const campus_aim_interfaces::msg::Target& t = packet.targets[target_index];
                bool state_ok =
                    t.state == "TRACKING" ||
                    (allow_detecting_aim_ && t.state == "DETECTING" && campus::knownArmorIdentity(t.number)) ||
                    (allow_temp_lost_ && t.state == "TEMP_LOST" &&
                     t.last_observation_age <
                         ((t.number != 6 && t.number != 8) ? std::max(max_age_, mobile_fire_coast_) : max_age_));

                if (!state_ok) {
                    selection_failure = t.state == "DETECTING" ? "WAIT_TRACK_CONFIRM" : "NO_TRACKING_TARGET";
                    continue;
                }

                if (t.position_variance > max_variance_ || !campus::validArmorIdentity(t.number) ||
                    (require_number_ && t.number == 0) || (target_number_ && t.number != target_number_)) {
                    selection_failure = t.position_variance > max_variance_ ? "TARGET_UNCERTAIN" : "TARGET_FILTERED";
                    continue;
                }

                if (t.armor_count < 1 || t.armor_count > 4 || !std::isfinite(t.position_variance) ||
                    t.position_variance < 0 || !std::isfinite(t.last_observation_age) || t.last_observation_age < 0) {
                    continue;
                }

                int preferred_slot = -1;

                if (slot_hysteresis_ && t.track_id == selected_id_) {
                    preferred_slot = selected_slot_;
                }

                campus::TargetMotion target_motion = motion(t);
                bool use_fixed = fixed_outpost_ && t.number == outpost_number_ && t.armor_count == 3 &&
                                 std::abs(t.yaw_rate) >= outpost_min_rate_ && campus::finite(target_motion.velocity) &&
                                 cv::norm(target_motion.velocity) <= 0.05;

                campus::InterceptSolution candidate;

                if (use_fixed) {
                    std::array<bool, 4> learned = t.seen;

                    for (int slot = 0; slot < 3; ++slot) {
                        learned[slot] = learned[slot] || t.observed[slot];
                    }

                    int preferred = t.track_id == selected_id_ ? outpost_slot_ : -1;
                    candidate = campus::selectOutpostPassage(
                        target_motion, learned, preferred, multiple_outpost_heights_, t.width_m, pivot, self_velocity,
                        offset_ + muzzle_offset_, f.state.yaw_degrees, f.state.pitch_degrees, speed, age + latency_,
                        gravity_, max_horizon_);
                } else {
                    unsigned allowed_slots = 0xf;
                    if (t.number == 4) {
                        // The radius/phase model cannot authorize an unseen plate.
                        // Do not slew to it first and only then reject its fire permit.
                        allowed_slots = 0;
                        for (int slot = 0; slot < t.armor_count; ++slot)
                            if (t.observed[slot] || (t.geometry_observable && t.seen[slot]))
                                allowed_slots |= 1u << slot;
                        if (t.state == "TEMP_LOST")
                            for (int slot = 0; slot < t.armor_count; ++slot)
                                if (t.seen[slot]) allowed_slots |= 1u << slot;
                    }
                    candidate = campus::interceptMovingShooter(
                        target_motion, pivot, self_velocity, offset_ + muzzle_offset_, f.state.yaw_degrees,
                        f.state.pitch_degrees, speed, age + latency_, gravity_, max_horizon_, preferred_slot,
                        slot_switch_margin_, min_facing_, allowed_slots);
                }

                if (!candidate.ballistic.valid) {
                    selection_failure = "NO_INTERCEPT";
                    continue;
                }

                double error_yaw =
                    campus::wrap((candidate.ballistic.yaw - f.state.yaw_degrees) * CV_PI / 180) * 180 / CV_PI;
                double cost = std::abs(error_yaw) + 0.5 * std::abs(candidate.ballistic.pitch - f.state.pitch_degrees);

                if (t.track_id == selected_id_) {
                    cost -= 2;
                }

                if (cost < best_cost) {
                    selected = &t;
                    aim = candidate;
                    fixed_outpost_aim = use_fixed;
                    best_cost = cost;
                }
            }
        }

        if (now().seconds() - campus::stampSeconds(source.header.stamp) > targetAgeLimit(packet)) {
            invalidate(f, "STALE_AFTER_SOLVE");
            return;
        }

        if (!selected) {
            invalidate(f, f.state.health <= 0 ? "DEAD" : selection_failure);
        } else {
            if (selected->track_id != selected_id_) {
                fire_confirmation_.clear();
                outpost_slot_ = -1;
            }
            selected_id_ = selected->track_id;
            selected_slot_ = aim.slot;
            if (fixed_outpost_aim) {
                outpost_slot_ = aim.slot;
            }

            campus::GimbalCommand stabilized =
                stabilizer_->update(aim.ballistic.yaw, aim.ballistic.pitch, f.state.yaw_degrees, f.state.pitch_degrees,
                                    campus::steadySeconds());

            if (!stabilized.valid) {
                invalidate(f, "INVALID_COMMAND");
                return;
            }
            geometry_msgs::msg::Vector3Stamped angles;
            angles.header = f.header;
            angles.header.frame_id = "world_optical_angles";
            angles.vector.x = aim.ballistic.yaw;
            angles.vector.y = aim.ballistic.pitch;
            angles.vector.z = 1;
            angles_pub_->publish(angles);
            geometry_msgs::msg::PointStamped point;
            point.header = f.header;
            point.header.frame_id = "map";
            point.point.x = aim.point[0];
            point.point.y = aim.point[1];
            point.point.z = aim.point[2];
            aim_point_pub_->publish(point);
            geometry_msgs::msg::Vector3Stamped command_angles = angles;
            command_angles.vector.x = stabilized.yaw;
            command_angles.vector.y = stabilized.pitch;
            command_angles_pub_->publish(command_angles);
            geometry_msgs::msg::Vector3Stamped selection = angles;
            selection.header.frame_id = "track_slot_score";
            selection.vector.x = selected_id_;
            selection.vector.y = selected_slot_;
            selection.vector.z = aim.score;
            selection_pub_->publish(selection);
            geometry_msgs::msg::Vector3Stamped residual = angles;
            residual.header.frame_id = "command_residual_deg";
            residual.vector.x = stabilized.yaw_residual;
            residual.vector.y = stabilized.pitch_residual;
            residual_pub_->publish(residual);
            std_msgs::msg::String state;
            state.data = selected->state + " T" + std::to_string(selected_id_) + " #" +
                         std::to_string(selected->number) + " slot=" + std::to_string(selected_slot_);
            std::string aim_mode = " FOLLOW_PLATE";

            if (fixed_outpost_aim) {
                aim_mode = multiple_outpost_heights_ ? " OUTPOST_MULTI" : " OUTPOST_FIXED";
            }

            state.data += aim_mode;
            state_pub_->publish(state);
            const bool is_outpost = selected->number == outpost_number_;
            const bool mobile_plate=selected->number>=1 && selected->number<=7 && selected->number!=6;
            auto feedback_estimate = mobile_plate && mobile_feedback_enabled_
                ? mobile_feedback_.predict(now().seconds(), latency_) : campus::MobileGimbalEstimate{};
            if (std::abs(feedback_estimate.stamp-campus::stampSeconds(f.state.header.stamp)) > 1e-6)
                feedback_estimate.valid = false;
            const double feedback_yaw = feedback_estimate.valid ? feedback_estimate.yaw : f.state.yaw_degrees;
            const double feedback_pitch = feedback_estimate.valid ? feedback_estimate.pitch : f.state.pitch_degrees;
            const double yaw_bound = feedback_estimate.valid ? feedback_estimate.yaw_bound : 0.;
            const double pitch_bound = feedback_estimate.valid ? feedback_estimate.pitch_bound : 0.;
            const double alignment_limit = is_outpost ? outpost_fire_tolerance_ : fire_tolerance_;
            const bool frontal_mobile=campus::immediateFrontalFire(selected->number,selected->state=="TRACKING",
                selected->observed[aim.slot],age+selected->last_observation_age,cv::norm(aim.point-aim.origin),aim.facing);
            const double confirmation_time = is_outpost ? outpost_settle_ : (frontal_mobile ? 0. : fire_settle_);
            const double observation_limit = is_outpost ? outpost_age_ + outpost_fire_coast_ : max_age_;
            double observation_age = age + selected->last_observation_age;
            double phase_sigma = std::sqrt(
                campus::predictedPhaseVariance(selected->yaw_variance, selected->yaw_rate_variance,
                                               selected->yaw_yaw_rate_covariance, aim.horizon, outpost_phase_noise_));
            bool coast_fire = selected->number == outpost_number_ && allow_temp_lost_ &&
                              selected->state == "TEMP_LOST" && selected->last_observation_age <= outpost_fire_coast_ &&
                              observation_age <= observation_limit && selected->geometry_observable &&
                              selected->seen[0] && selected->seen[1] && selected->seen[2] &&
                              phase_sigma <= outpost_max_phase_sigma_ &&
                              selected->position_variance <= outpost_max_center_sigma_ * outpost_max_center_sigma_;

            if (new_packet && selected->state == "TRACKING" && selected->observed[aim.slot] && selected->number != 6 &&
                selected->number != 8 && campus::knownArmorIdentity(selected->number) && command_enabled_ &&
                shoot_enabled_ && !f.state.is_weakened && f.state.ammo_remaining > 0)
                mobile_fire_.observedTrack(selected_id_, aim.slot, campus::steadySeconds());

            double plate_variance = selected->state_covariance_valid
                                        ? campus::predictedPlateVariance(motion(*selected), aim.slot, aim.horizon,
                                                                         selected->state_covariance, aim.origin)
                                        : INFINITY;

            bool mobile_confident = selected->state_covariance_valid && campus::mobileCoastConfident(
                campus::predictedPlateAxisVariances(motion(*selected), aim.slot, aim.horizon,
                    selected->state_covariance, aim.origin), mobile_prediction_sigma_, aim.facing,
                selected->width_m, selected->height_m, selected->tilt, aim.horizon, selected->last_observation_age);
            const bool mobile_lease =
                !fixed_outpost_aim && allow_temp_lost_ && selected->state == "TEMP_LOST" && selected->number != 6 &&
                selected->number != 8 && campus::knownArmorIdentity(selected->number) && selected->seen[aim.slot] &&
                mobile_fire_.allows(selected_id_, aim.slot, campus::steadySeconds(), selected->last_observation_age,
                                    observation_age, mobile_fire_coast_);
            const bool mobile_coast = mobile_lease && mobile_confident;
            std::string fire_reason;
            bool any_observed = false;

            for (int slot = 0; slot < selected->armor_count; ++slot) {
                if (selected->observed[slot]) {
                    any_observed = true;
                }
            }

            bool game_alignment = is_outpost && !fixed_outpost_aim && outpost_game_alignment_ &&
                                  selected->armor_count == 3 && selected->geometry_observable && selected->seen[0] &&
                                  selected->seen[1] && selected->seen[2] && std::isfinite(phase_sigma) &&
                                  phase_sigma <= outpost_max_phase_sigma_ &&
                                  selected->position_variance <= outpost_max_center_sigma_ * outpost_max_center_sigma_;
            double raw_yaw_error = campus::wrapDegrees(aim.ballistic.yaw - feedback_yaw);
            double raw_error = std::hypot(raw_yaw_error, aim.ballistic.pitch - feedback_pitch);
            double command_error = std::hypot(campus::wrapDegrees(stabilized.yaw - feedback_yaw),
                                              stabilized.pitch - feedback_pitch);
            double lag_error = std::hypot(stabilized.yaw_residual, stabilized.pitch_residual);

            if (!command_enabled_) {
                fire_reason = "COMMAND_DISABLED";
            } else if (!shoot_enabled_) {
                fire_reason = "SHOOT_DISABLED";
            } else if (f.state.is_weakened) {
                fire_reason = "WEAKENED";
            } else if (f.state.ammo_remaining <= 0) {
                fire_reason = "NO_AMMO";
            } else if (selected->state != "TRACKING" && !coast_fire && !mobile_coast) {
                fire_reason = selected->state == "DETECTING"
                                  ? "WAIT_TRACK_CONFIRM"
                                  : (mobile_lease && !mobile_confident ? "PREDICTION_UNCERTAIN" : "TEMP_LOST");
            } else if ((fire_require_number_ && !campus::knownArmorIdentity(selected->number))) {
                fire_reason = "UNKNOWN_NUMBER";
            } else if ((!mobile_coast &&
                        observation_age >
                            (is_outpost ? outpost_age_ + (coast_fire ? outpost_fire_coast_ : 0.) : max_age_)) ||
                       (!any_observed && !coast_fire && !mobile_coast)) {
                fire_reason = "NO_FRESH_OBSERVATION";
            } else if (!mobile_coast &&
                       !campus::firePlateObserved(selected->observed[aim.slot], any_observed || coast_fire,
                                                  selected->geometry_observable && selected->seen[aim.slot],
                                                  require_current_plate_ || selected->number == 0)) {
                fire_reason = "PLATE_NOT_OBSERVED";
            } else if (!(game_alignment
                             ? campus::gimbalCommandSettled(stabilized, alignment_limit)
                             : campus::gimbalAligned(aim.ballistic.yaw, aim.ballistic.pitch, stabilized,
                                                     feedback_yaw, feedback_pitch, alignment_limit))) {
                fire_reason = "WAIT_ALIGNMENT";
            } else if (fixed_outpost_aim && !selected->geometry_observable) {
                fire_reason = "WAIT_GEOMETRY";
            } else if (fixed_outpost_aim &&
                       selected->position_variance > outpost_max_center_sigma_ * outpost_max_center_sigma_) {
                fire_reason = "WAIT_CENTER_CONFIDENCE";
            } else if (fixed_outpost_aim && phase_sigma > outpost_max_phase_sigma_) {
                fire_reason = "WAIT_PHASE_CONFIDENCE";
            } else if (fixed_outpost_aim &&
                       !campus::insideFireWindow(raw_yaw_error, aim.ballistic.pitch - feedback_pitch,
                                                 cv::norm(aim.point - aim.origin), 1.0, selected->width_m,
                                                 selected->height_m, selected->tilt, fire_tolerance_)) {
                fire_reason = "WAIT_OUTPOST_ALIGNMENT";
            } else if (fixed_outpost_aim &&
                       !campus::insideOutpostPassage(motion(*selected), aim, selected->width_m, selected->height_m,
                                                     selected->tilt, phase_sigma)) {
                fire_reason = "WAIT_OUTPOST_PHASE";
            }

            if (fire_reason.empty() && (use_plate_window_ || mobile_plate)) {
                double raw_yaw_error = campus::wrapDegrees(aim.ballistic.yaw - feedback_yaw);
                double distance = cv::norm(aim.point - aim.origin);
                auto in_window = [&](double yaw, double pitch) {
                    return mobile_plate
                        ? [&] {
                            auto window = campus::mobileFireWindow(distance,aim.facing,selected->width_m,
                                selected->height_m,selected->tilt,fire_tolerance_,mobile_window_scale_);
                            window.yaw_deg -= yaw_bound;
                            window.pitch_deg -= pitch_bound;
                            return window.contains(yaw,pitch);
                          }()
                        : campus::insideFireWindow(yaw,pitch,distance,aim.facing,selected->width_m,
                              selected->height_m,selected->tilt,fire_tolerance_);
                };
                bool raw_in_window = in_window(raw_yaw_error, aim.ballistic.pitch-feedback_pitch);
                bool command_in_window = in_window(campus::wrapDegrees(stabilized.yaw-feedback_yaw),
                                                    stabilized.pitch-feedback_pitch);
                bool lag_in_window = in_window(stabilized.yaw_residual,stabilized.pitch_residual);
                if (!raw_in_window || !command_in_window || !lag_in_window) {
                    fire_reason = "PLATE_WINDOW";
                }
            }

            if (fire_reason.empty() && mobile_plate) {
                auto hits = [&](double yaw, double pitch) {
                    return campus::mobileBallisticWindow(motion(*selected), aim, selected->tilt,
                        selected->width_m, selected->height_m, offset_ + muzzle_offset_, yaw, pitch, speed, gravity_);
                };
                bool barrel_hits = hits(feedback_yaw,feedback_pitch);
                if (feedback_estimate.valid)
                    for (double y : {-yaw_bound,yaw_bound})
                        for (double p : {-pitch_bound,pitch_bound})
                            barrel_hits = barrel_hits && hits(feedback_yaw+y,feedback_pitch+p);
                if (!barrel_hits || !hits(stabilized.yaw,stabilized.pitch))
                    fire_reason = "MOBILE_BALLISTIC_WINDOW";
            }

            bool fire = fire_confirmation_.update(selected_id_, aim.slot, fire_reason.empty(), now().seconds(),
                                                  confirmation_time);

            if (fire_reason.empty()) {
                fire_reason = fire ? (mobile_coast ? "ALLOW_PREDICTED" : "ALLOW") : "WAIT_CONFIRM";
            }

            if (!command_enabled_ || !shoot_enabled_ || f.state.is_weakened || f.state.ammo_remaining <= 0)
                mobile_fire_.clear();
            std::string fire_detail = aim_mode;
            fire_detail += " slot=" + std::to_string(aim.slot);
            fire_detail += game_alignment ? " alignment=GAME_REALTIME" :
                (feedback_estimate.valid ? " alignment=TIME_ALIGNED_FEEDBACK" : " alignment=MEASURED_FEEDBACK");
            if (mobile_plate)
                fire_detail += cv::format(" feedback_age=%.0fms feedback_bound=%.3f/%.3fdeg measured_dy=%.2f",
                    feedback_age*1000,yaw_bound,pitch_bound,
                    campus::wrapDegrees(aim.ballistic.yaw-f.state.yaw_degrees));

            if (fixed_outpost_aim) {
                double front_yaw = std::atan2(selected->center.y - aim.point[1], selected->center.x - aim.point[0]);
                double phase = campus::wrap(motion(*selected).angle(aim.slot, aim.horizon) - front_yaw);
                fire_detail += cv::format(" phase=%.1f deg", phase * 180 / CV_PI);
            }

            fire_detail += cv::format(" raw=%.2f cmd=%.2f lag=%.2f limit=%.2f deg dy=%.2f dp=%.2f age=%.0fms",
                                      raw_error, command_error, lag_error, alignment_limit, raw_yaw_error,
                                      aim.ballistic.pitch - feedback_pitch, age * 1000);
            fire_detail += cv::format(" number=%d range=%.2fm", selected->number, cv::norm(aim.point-aim.origin));
            fire_detail +=
                cv::format(" obs=%.0fms phase_sigma=%.2f deg", observation_age * 1000, phase_sigma * 180 / CV_PI);
            fire_detail +=
                cv::format(" missing=%.0fms plate_sigma=%.3fm coast=%s", selected->last_observation_age * 1000,
                           std::sqrt(plate_variance), mobile_coast ? "true" : "false");

            publishFireState(fire_reason + fire_detail);
            geometry_msgs::msg::Vector3Stamped framed = command_angles;
            framed.vector.z = fire ? 1 : 0;
            framed_command_pub_->publish(framed);

            auto observation_stamp = source.header.stamp;
            int64_t observed_ns = rclcpp::Time(source.header.stamp).nanoseconds() -
                                  static_cast<int64_t>(selected->last_observation_age * 1e9);
            observation_stamp = rclcpp::Time(std::max<int64_t>(0, observed_ns));

            const auto mobile_window = mobile_plate
                ? campus::mobileFireWindow(cv::norm(aim.point-aim.origin), aim.facing, selected->width_m,
                    selected->height_m, selected->tilt, fire_tolerance_, mobile_window_scale_)
                : campus::MobileFireWindow{};
            publishSolution(f, observation_stamp, selected_id_, selected->number, selected_slot_, stabilized.yaw,
                            stabilized.pitch, true, fire, fire_reason, mobile_window, aim.ballistic.yaw,
                            aim.ballistic.pitch);

            if (command_pub_) {
                tdt_interface::msg::SendData command;
                command.yaw = stabilized.yaw;
                command.pitch = stabilized.pitch;
                command.if_shoot = fire;
                command_pub_->publish(command);
            }
        }

        if (new_packet && display_ && ++frames_ % display_every_ == 0) {
            try {
                cv::Mat image = campus::decodeFrame(source);
                if (image.empty()) {
                    return;
                }

                cv::Matx33d k = campus::intrinsics(source);
                cv::Mat d = campus::distortion(source);
                cv::drawMarker(image, cv::Point2f(k(0, 2), k(1, 2)), {255, 255, 255}, cv::MARKER_CROSS, 24, 1);

                if (show_prediction_) {
                    for (size_t target_index = 0; target_index < packet.targets.size(); ++target_index) {
                        const campus_aim_interfaces::msg::Target& t = packet.targets[target_index];
                        campus::TargetMotion m = motion(t);

                        if (m.count < 1 || m.count > 4) {
                            continue;
                        }

                        double horizon = age + latency_;

                        if (selected && t.track_id == selected_id_) {
                            horizon = aim.horizon;
                        }

                        for (int i = 0; i < m.count; ++i) {
                            campus::drawArmor(image, camera, k, d, m.position(i, horizon), m.angle(i, horizon),
                                              t.width_m, t.height_m, {255, 255, 0},
                                              "T" + std::to_string(t.track_id) + "/" + std::to_string(i), false,
                                              t.tilt);
                        }
                    }
                }

                std::string text = "NO TARGET";

                if (selected) {
                    cv::Point2f point;

                    if (camera.project(aim.point, k, d, point)) {
                        cv::drawMarker(image, point, {0, 0, 255}, cv::MARKER_CROSS, 24, 2);
                    }
                    text = "#" + std::to_string(selected->number) + " slot=" + std::to_string(aim.slot) +
                           " raw_yaw=" + std::to_string(aim.ballistic.yaw) +
                           " pitch=" + std::to_string(aim.ballistic.pitch);
                    cv::putText(image,
                                "flight=" + std::to_string(int(aim.ballistic.flight_time * 1000)) +
                                    "ms speed=" + std::to_string(speed) + "m/s",
                                {15, 55}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1);
                }

                cv::putText(image, text, {15, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 255, 255}, 1);
                campus::showFrame("aim_solver", image);
            } catch (const std::exception& e) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Solver display: %s", e.what());
            }
        }
    }//目标选择、运动预测、弹道求解和射击判断

    campus::FrameGuard guard_;
    campus::FrameGuard feedback_guard_;
    campus_aim_interfaces::msg::TrackFrame::ConstSharedPtr cached_targets_;
    campus::Frame::ConstSharedPtr feedback_frame_;
    rclcpp::Subscription<campus::Frame>::SharedPtr feedback_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    int control_period_;
    bool feedback_blocked_ = false;
    double feedback_barrier_time_ = -1;
    bool multiple_outpost_heights_;
    double outpost_age_ = .28, outpost_fire_tolerance_ = 1.18, outpost_settle_ = 0.;
    bool outpost_game_alignment_ = false;
    double outpost_fire_coast_;
    double outpost_max_phase_sigma_;
    double outpost_phase_noise_;
    double outpost_max_center_sigma_;
    campus::FireConfirmation fire_confirmation_;
    campus::MobileFireContinuation mobile_fire_;
    campus::ShooterMotion shooter_motion_;
    campus::MobileGimbalFeedback mobile_feedback_;
    bool mobile_feedback_enabled_ = false;
    double mobile_window_scale_ = 1.0;
    double mobile_fire_coast_ = 0, mobile_prediction_sigma_ = .20;
    std::unique_ptr<campus::GimbalStabilizer> stabilizer_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr command_angles_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr selection_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr residual_pub_;
    bool slot_hysteresis_;
    double slot_switch_margin_;
    double min_facing_;
    int selected_slot_ = -1;
    rclcpp::Subscription<campus_aim_interfaces::msg::TrackFrame>::SharedPtr targets_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr angles_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fire_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr aim_point_pub_;
    rclcpp::Publisher<tdt_interface::msg::SendData>::SharedPtr command_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_;
    campus::Frame last_frame_;
    cv::Vec3d offset_;
    cv::Vec3d muzzle_offset_;
    double scale_;
    double latency_;
    double max_age_;
    double gravity_;
    double default_speed_;
    double max_horizon_;
    double max_variance_;
    double fire_tolerance_;
    double fire_settle_;
    double last_receive_ = -1;
    int target_number_;
    int display_every_;
    int player_;
    bool display_;
    bool show_prediction_;
    bool allow_temp_lost_;
    bool allow_detecting_aim_;
    bool require_number_;
    bool command_enabled_;
    bool shoot_enabled_;
    bool fire_require_number_ = true;
    void publishSolution(const campus::Frame& feedback, const builtin_interfaces::msg::Time& observed, uint32_t track,
                         int number, int slot, double yaw, double pitch, bool valid, bool fire,
                         const std::string& reason, campus::MobileFireWindow window = {},
                         double fire_yaw = 0., double fire_pitch = 0.) {
        campus_aim_interfaces::msg::AimSolution message;
        message.header = feedback.state.header;
        message.observation_stamp = observed;
        message.state_session = campus::sessionKey(feedback.state);
        message.frame_sequence = feedback.state.frame_sequence;
        message.track_id = track;
        message.number = number;
        message.slot = slot;
        message.yaw = yaw;
        message.pitch = pitch;
        message.valid = valid;
        message.fire = fire;
        message.reason = reason;
        message.calculation_stamp = now();
        message.mobile_window_valid = window.yaw_deg > 0 && window.pitch_deg > 0;
        message.mobile_yaw_limit_deg = window.yaw_deg;
        message.mobile_pitch_limit_deg = window.pitch_deg;
        message.fire_yaw = fire_yaw;
        message.fire_pitch = fire_pitch;

        solution_pub_->publish(message);
    }
    rclcpp::Publisher<campus_aim_interfaces::msg::AimSolution>::SharedPtr solution_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr framed_command_pub_;
    bool use_plate_window_;
    bool require_current_plate_;
    bool fixed_outpost_;
    double outpost_min_rate_;
    int outpost_number_;
    int outpost_slot_ = -1;
    uint64_t frames_ = 0;
    uint32_t selected_id_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<AimSolverNode>());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "aim_solver: %s\n", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
