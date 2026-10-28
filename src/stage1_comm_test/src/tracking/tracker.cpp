#include "tracking/reprojection.hpp"
#include "common/ros_aim_utils.hpp"
#include "aiming/gimbal_stabilizer.hpp"

#include <functional>
#include <campus_aim_interfaces/msg/armor_frame.hpp>
#include <campus_aim_interfaces/msg/track_frame.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/string.hpp>

class ArmorTrackerNode : public rclcpp::Node {
  public:
    ArmorTrackerNode() : Node("armor_tracker") {
        campus::TrackerConfig c;
        c.use_pixel_update = declare_parameter<bool>("measurement.use_pixels", true);
        c.pixel_sigma = campus::numberParam(*this, "measurement.pixel_sigma_px", 2.0);
        c.pixel_gate = campus::numberParam(*this, "measurement.pixel_gate", 40.0);

        if (!std::isfinite(c.pixel_sigma) || c.pixel_sigma <= 0 || !std::isfinite(c.pixel_gate) || c.pixel_gate <= 0) {
            throw std::invalid_argument("Invalid pixel measurement parameters");
        }

        c.radius = campus::numberParam(*this, "marker_radius_m", 0.26);
        c.outpost_radius = campus::numberParam(*this, "outpost_radius_m", 0.28);
        c.min_radius = campus::numberParam(*this, "min_radius_m", 0.12);
        c.max_radius = campus::numberParam(*this, "max_radius_m", 0.4);
        c.initial_radius_sigma = campus::numberParam(*this, "initial_radius_sigma_m", 0.08);
        c.initial_height_sigma = campus::numberParam(*this, "initial_height_sigma_m", 0.20);
        c.max_height_difference = campus::numberParam(*this, "max_height_difference_m", 0.50);
        c.match_distance = campus::numberParam(*this, "max_match_distance_m", 0.6);

        c.match_yaw = campus::numberParam(*this, "max_match_yaw_rad", 1.2);
        c.mahalanobis_gate = campus::numberParam(*this, "mahalanobis_gate", 25);

        c.q_xy = campus::numberParam(*this, "q_xy", 0.5);
        c.q_z = campus::numberParam(*this, "q_z", 0.1);
        c.q_yaw = campus::numberParam(*this, "q_yaw", 0.5);
        c.q_outpost_yaw = campus::numberParam(*this, "q_outpost_yaw", 0.08);
        c.q_radius = campus::numberParam(*this, "q_radius", 0.00001);

        c.lost_timeout = campus::numberParam(*this, "lost_timeout_s", 0.35);
        c.min_confirm_time = campus::numberParam(*this, "min_confirm_time_s", 0.05);
        c.confirm_frames = campus::intParam(*this, "tracking_confirm_frames", 5);
        c.field_confirm_frames = campus::intParam(*this, "field_robot_confirm_frames", 0);
        c.field_confirm_time = campus::numberParam(*this, "field_robot_min_confirm_time_s", .03);

        if (c.field_confirm_frames < 0 || c.field_confirm_frames > 5 || !std::isfinite(c.field_confirm_time) ||
            c.field_confirm_time < .02 || c.field_confirm_time > .15)
            throw std::invalid_argument("Invalid field robot confirmation parameters");

        c.outpost_number = campus::intParam(*this, "outpost_number", 6);
        c.forced_count = campus::intParam(*this, "armor_count", 0);
        c.max_tracks = campus::intParam(*this, "max_tracks", 1);
        c.track_unknown = declare_parameter<bool>("track_unknown", false);
        c.stationary_outpost = declare_parameter<bool>("stationary_outpost", true);

        c.outpost_center_gain_scale = campus::numberParam(*this, "outpost.center_gain_scale", 1.0);
        c.outpost_center_warmup_s = campus::numberParam(*this, "outpost.center_warmup_s", 1.5);
        c.outpost_center_max_sigma_m = campus::numberParam(*this, "outpost.center_max_sigma_m", 0.03);
        c.outpost_center_max_residual_m = campus::numberParam(*this, "outpost.center_max_residual_m", 0.08);
        c.outpost_center_q = campus::numberParam(*this, "outpost.center_q", 1e-5);

        if (c.forced_count < 0 || c.forced_count > 4 || c.confirm_frames < 1 || c.max_tracks < 1 || c.min_radius <= 0 ||
            c.max_radius <= c.min_radius || c.lost_timeout <= 0 || c.initial_radius_sigma <= 0 ||
            c.initial_height_sigma <= 0 || c.max_height_difference <= 0) {
            throw std::invalid_argument("Invalid tracker parameters");
        }

        if (!std::isfinite(c.outpost_center_gain_scale) || c.outpost_center_gain_scale <= 0 ||
            c.outpost_center_gain_scale > 1 || !std::isfinite(c.outpost_center_warmup_s) ||
            c.outpost_center_warmup_s < 0 || !std::isfinite(c.outpost_center_max_sigma_m) ||
            c.outpost_center_max_sigma_m <= 0 || !std::isfinite(c.outpost_center_max_residual_m) ||
            c.outpost_center_max_residual_m <= 0 || !std::isfinite(c.outpost_center_q) || c.outpost_center_q < 0) {
            throw std::invalid_argument("Invalid outpost center parameters");
        }

        tracker_ = std::make_unique<campus::MultiTargetTracker>(c);

        scale_ = campus::numberParam(*this, "scene_units_per_meter", 3);
        offset_ = campus::offsetParams(*this);
        horizon_ = campus::numberParam(*this, "prediction_time_s", 0.15);
        target_number_ = campus::intParam(*this, "target_number", 0);
        mobile_only_ = declare_parameter<bool>("target.mobile_only", false);
        acquire_detecting_ = declare_parameter<bool>("target.acquire_detecting", false);

        strict_order_ = declare_parameter<bool>("mission.strict_order", false);
        field_order_ = declare_parameter<bool>("mission.field_robot_order", false);
        if (strict_order_) {
            mission_stage_ = "WAIT_MISSION";
            rclcpp::QoS cached(1);
            cached.reliable().transient_local();
            mission_sub_ = create_subscription<std_msgs::msg::String>(
                "/mission/status", cached, [this](std_msgs::msg::String::ConstSharedPtr m) {
                    mission_stage_ = campus::trackerMissionStage(m->data);
                    sentry_last_ = m->data.find(" sentry_last=1") != std::string::npos;
                    ignore_one_ = m->data.find(" ignored_enemy_one=1") != std::string::npos;
                });
        }

        display_ = declare_parameter<bool>("display", true);
        show_future_ = declare_parameter<bool>("display.show_future", false);
        show_measurements_ = declare_parameter<bool>("display.show_measurements", true);
        publish_image_ = declare_parameter<bool>("debug.publish_image", false);
        window_ = declare_parameter<std::string>("display.window", "reprojection");
        display_every_ = std::max(1, campus::intParam(*this, "display_every_n_frames", 2));
        max_age_ = campus::numberParam(*this, "max_frame_age_ms", 200) / 1000;

        if (scale_ <= 0 || horizon_ < 0 || max_age_ <= 0) {
            throw std::invalid_argument("Invalid tracker frame parameters");
        }

        track_pub_ = create_publisher<campus_aim_interfaces::msg::TrackFrame>("/tracker/targets", campus::latestQoS());
        markers_pub_ =
            create_publisher<geometry_msgs::msg::PoseArray>("/tracker/predicted_markers", campus::latestQoS());
        center_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/tracker/rigid_center", campus::latestQoS());
        center_velocity_pub_ =
            create_publisher<geometry_msgs::msg::Vector3Stamped>("/tracker/rigid_velocity", campus::latestQoS());
        point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/tracker/filtered_point", campus::latestQoS());
        velocity_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/tracker/velocity", campus::latestQoS());
        state_pub_ = create_publisher<std_msgs::msg::String>("/tracker/state", campus::latestQoS());

        fire_state_sub_ = create_subscription<std_msgs::msg::String>(
            "/aim/fire_state", campus::latestQoS(),
            std::bind(&ArmorTrackerNode::fireStateCallback, this, std::placeholders::_1));
        aim_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/aim/point", campus::latestQoS(),
            std::bind(&ArmorTrackerNode::aimPointCallback, this, std::placeholders::_1));
        if (publish_image_) {
            image_pub_ = create_publisher<sensor_msgs::msg::Image>("/tracker/reprojection", campus::latestQoS());
        }

        observations_sub_ = create_subscription<campus_aim_interfaces::msg::ArmorFrame>(
            "/armor/observations", campus::latestQoS(),
            std::bind(&ArmorTrackerNode::observationsCallback, this, std::placeholders::_1));

        watchdog_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&ArmorTrackerNode::checkTimeout, this));
        if (display_) {
            cv::namedWindow(window_, cv::WINDOW_NORMAL);
            cv::resizeWindow(window_, 960, 720);
        }

        RCLCPP_INFO(get_logger(), "Body lock, fixed plate slots, 4/3 plate reprojection; max_tracks=%d", c.max_tracks);
    }

    ~ArmorTrackerNode() override {
        if (display_) {
            cv::destroyWindow(window_);
        }
    }

  private:
    //接收瞄准解算器计算的瞄准点
    void aimPointCallback(geometry_msgs::msg::PointStamped::ConstSharedPtr point) {
        aim_point_ = {point->point.x, point->point.y, point->point.z};
        aim_point_stamp_ = campus::stampSeconds(point->header.stamp);
    }

    //接收瞄准解算器的射击状态
    void fireStateCallback(std_msgs::msg::String::ConstSharedPtr state) {
        fire_state_ = state->data;
        fire_state_time_ = campus::steadySeconds();
    }

    //计算预测装甲板和真实检测角点之间的重投影误差
    static double measurementFitError(const campus::RigidTarget& target, const campus_aim_interfaces::msg::Armor& armor,
                                      const campus::CameraGeometry& camera, const cv::Matx33d& k,
                                      const cv::Mat& distortion) {
        if (armor.number > 0 && armor.number != target.number) {
            return INFINITY;
        }

        cv::Vec3d measured(armor.position.x, armor.position.y, armor.position.z);
        int nearest_slot = -1;
        double nearest_distance = 0.5;

        for (int slot = 0; slot < target.count; ++slot) {
            if (!target.observed[slot]) {
                continue;
            }

            double distance = cv::norm(measured - target.position(slot));
            if (distance < nearest_distance) {
                nearest_distance = distance;
                nearest_slot = slot;
            }
        }

        if (nearest_slot < 0) {
            return INFINITY;
        }

        std::array<cv::Vec3d, 4> corners = campus::armorCorners(target.position(nearest_slot), target.yaw(nearest_slot),
                                                                target.width, target.height, target.tilt);
        double squared_error = 0;
        for (int corner = 0; corner < 4; ++corner) {
            cv::Point2f projected;

            if (!camera.project(corners[corner], k, distortion, projected)) {
                return INFINITY;
            }

            double dx = projected.x - armor.image_points[corner].x;
            double dy = projected.y - armor.image_points[corner].y;
            squared_error += dx * dx + dy * dy;
        }

        return std::sqrt(squared_error / 4);
    }

    //接收装甲板检测器发布的观测结果
    void observationsCallback(campus_aim_interfaces::msg::ArmorFrame::ConstSharedPtr packet) {
        process(*packet);
    }

    //检查数据是否断流，超时就清除旧跟踪状态
    void checkTimeout() {
        if (display_) {
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q') {
                rclcpp::shutdown();
                return;
            }
        }

        if (last_receive_ <= 0 || campus::steadySeconds() - last_receive_ <= max_age_) {
            return;
        }

        tracker_->reset();
        selector_.locked_id = 0;
        publishState("LOST");
        last_receive_ = -1;

        geometry_msgs::msg::PoseArray markers;
        markers.header = last_header_;
        markers.header.frame_id = "map";
        markers_pub_->publish(markers);

        if ((display_ || publish_image_) && !last_raw_image_.empty()) {
            cv::Mat image = last_raw_image_.clone();
            cv::putText(image, "LOST: source timeout", {15, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 0, 255}, 1);
            if (image_pub_) {
                image_pub_->publish(*cv_bridge::CvImage(last_header_, "bgr8", image).toImageMsg());
            }

            if (display_) {
                campus::showFrame(window_, image);
            }
        }
    }

    //发布目标跟踪状态
    void publishState(const std::string& state) {
        std_msgs::msg::String s;
        s.data = state;
        state_pub_->publish(s);
    }

    void process(const campus_aim_interfaces::msg::ArmorFrame& packet) {
        const campus::Frame& f = packet.frame;
        bool reset = false;
        bool valid = guard_.check(f, now().seconds(), max_age_, reset) && campus::validCamera(f);

        if (packet.reset || reset || f.state.health <= 0) {
            tracker_->reset();
            selector_.locked_id = 0;
        }

        if (!valid) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "TRACK_DROP source_age_ms=%.1f reset=%d",
                                 (now().seconds() - campus::stampSeconds(f.header.stamp)) * 1000., reset);
            if (packet.reset || reset) {
                campus_aim_interfaces::msg::TrackFrame out;
                out.frame = f;
                out.reset = true;

                track_pub_->publish(out);
                publishState("LOST");
            }

            return;
        }

        last_receive_ = campus::steadySeconds();
        campus::CameraGeometry camera = campus::cameraGeometry(f, scale_, offset_);
        cv::Matx33d k = campus::intrinsics(f);
        cv::Mat d = campus::distortion(f);

        std::string selection_stage =
            campus::armorSelectionStage(mission_stage_, f.state.ammo_remaining > 0 && !f.state.is_weakened);
        if (sentry_last_ && (mission_stage_ == "CLEAR_ROBOTS" || mission_stage_ == "WAIT_CLEAR_ROBOTS_POSITION") &&
            (f.state.enemy_healths[1] > 0 || f.state.enemy_healths[2] > 0 || f.state.enemy_healths[3] > 0))
            selection_stage = "FIELD_ROBOTS";
        // Filter BEFORE allocating body candidates. Otherwise a visible
        // outpost/dead robot can occupy it forever while selection rejects it.
        if (strict_order_) {
            auto& targets = tracker_->targets;
            targets.erase(std::remove_if(targets.begin(), targets.end(),
                                         [&](const auto& t) {
                                             return !campus::missionArmorAlive(selection_stage, t.number,
                                                                               f.state.enemy_healths, ignore_one_, field_order_);
                                         }),
                          targets.end());
        }

        std::vector<campus::Observation> observations;
        for (size_t observation_index = 0; observation_index < packet.armors.size(); ++observation_index) {
            const campus_aim_interfaces::msg::Armor& o = packet.armors[observation_index];
            if (!campus::validArmorIdentity(o.number) ||
                (strict_order_ &&
                 !campus::missionArmorAlive(selection_stage, o.number, f.state.enemy_healths, ignore_one_, field_order_)) ||
                (mobile_only_ && !(o.number >= 1 && o.number <= 5) && o.number != 7) ||
                (target_number_ && o.number && o.number != target_number_)) {
                continue;
            }

            campus::Observation v;
            v.position = {o.position.x, o.position.y, o.position.z};
            v.yaw = o.yaw;
            v.number = o.number;
            v.position_variance = o.position_variance;
            v.yaw_variance = o.yaw_variance;
            v.width = o.width_m;
            v.height = o.height_m;

            v.tilt = o.tilt;
            v.camera_rotation = camera.rotation;
            v.camera_matrix = k;
            v.distortion = d;
            v.has_pixels = true;

            for (int corner = 0; corner < 4; ++corner) {
                v.image_points[corner] = {o.image_points[corner].x, o.image_points[corner].y};
                if (!std::isfinite(v.image_points[corner].x) || !std::isfinite(v.image_points[corner].y)) {
                    v.has_pixels = false;
                }
            }

            if (v.has_pixels &&
                std::abs(cv::contourArea(std::vector<cv::Point2f>(v.image_points.begin(), v.image_points.end()))) < 8) {
                v.has_pixels = false;
            }

            v.has_view = true;
            v.view_origin = camera.origin;
            v.tangential_variance = o.tangential_variance;
            v.has_alternative = o.has_alternative;
            v.alternative_yaw = o.alternative_yaw;
            v.alternative_position = {o.alternative_position.x, o.alternative_position.y, o.alternative_position.z};
            v.has_alternative =
                v.has_alternative && campus::finite(v.alternative_position) && std::isfinite(v.alternative_yaw);

            if (campus::finite(v.position) && std::isfinite(v.yaw) && std::isfinite(v.position_variance) &&
                v.position_variance > 0 && std::isfinite(v.yaw_variance) && v.yaw_variance > 0 &&
                std::isfinite(v.tangential_variance) && std::isfinite(v.tilt) && std::isfinite(v.width) &&
                v.width > 0 && std::isfinite(v.height) && v.height > 0) {
                observations.push_back(v);
            }
        }

        double time = campus::stampSeconds(f.header.stamp);
        if (f.state.health > 0) {
            tracker_->update(observations, time);
        }

        const campus::RigidTarget* selected =
            selector_.select(tracker_->targets, camera, k, d, target_number_, acquire_detecting_, selection_stage,
                             strict_order_ ? &f.state.enemy_healths : nullptr, ignore_one_, field_order_);

        campus_aim_interfaces::msg::TrackFrame out;
        out.frame = f;
        out.reset = packet.reset || reset || f.state.health <= 0;

        geometry_msgs::msg::PoseArray markers;
        markers.header = f.header;
        markers.header.frame_id = "map";
        int selected_slot = 0;
        double nearest = INFINITY;

        if (selected) {
            const campus::RigidTarget& t = *selected;
            campus_aim_interfaces::msg::Target msg;
            msg.track_id = t.id;
            msg.number = t.number;
            msg.armor_count = t.count;
            msg.state = t.state;
            msg.center.x = t.x(0);
            msg.center.y = t.x(2);

            msg.center.z = t.x(4);
            msg.velocity.x = t.x(1);
            msg.velocity.y = t.x(3);
            msg.velocity.z = t.x(5);
            msg.yaw = t.x(6);
            msg.yaw_rate = t.x(7);
            msg.radius = t.x(8);
            msg.radius_odd = t.x(9);

            msg.height_difference = t.x(10);
            msg.height_difference_second = t.x(11);
            msg.tilt = t.tilt;
            msg.width_m = t.width;
            msg.height_m = t.height;
            msg.observed = t.observed;
            msg.seen = t.seen;
            msg.position_variance = std::max({t.p(0, 0), t.p(2, 2), t.p(4, 4)});

            msg.yaw_variance = t.p(6, 6);
            msg.yaw_rate_variance = t.p(7, 7);
            msg.yaw_yaw_rate_covariance = t.p(6, 7);
            msg.state_covariance_valid = t.p.allFinite();

            for (int row = 0; row < 12; ++row)
                for (int col = 0; col < 12; ++col)
                    msg.state_covariance[row * 12 + col] = t.p(row, col);
            msg.last_observation_age = time - t.last_seen;
            msg.geometry_observable = t.geometry_observable;
            out.targets.push_back(msg);

            for (int i = 0; i < t.count; ++i) {
                cv::Vec3d p = t.position(i);
                double yaw = t.yaw(i);
                geometry_msgs::msg::Pose pose;
                pose.position.x = p[0];
                pose.position.y = p[1];
                pose.position.z = p[2];
                pose.orientation.z = std::sin(yaw / 2);
                pose.orientation.w = std::cos(yaw / 2);

                markers.poses.push_back(pose);
                cv::Vec3d direction = p - camera.origin;
                if (direction.dot(cv::Vec3d(std::cos(yaw), std::sin(yaw), 0)) > 0 && cv::norm(direction) < nearest) {
                    nearest = cv::norm(direction);
                    selected_slot = i;
                }
            }

            geometry_msgs::msg::PointStamped p;
            p.header = markers.header;
            cv::Vec3d plate = t.position(selected_slot);
            p.point.x = plate[0];
            p.point.y = plate[1];
            p.point.z = plate[2];
            point_pub_->publish(p);
            p.point.x = t.x(0);

            p.point.y = t.x(2);
            p.point.z = t.x(4);
            center_pub_->publish(p);
            geometry_msgs::msg::Vector3Stamped v;
            v.header = markers.header;
            cv::Vec3d speed = t.armorVelocity(selected_slot);
            v.vector.x = speed[0];
            v.vector.y = speed[1];

            v.vector.z = speed[2];
            velocity_pub_->publish(v);
            v.vector.x = t.x(1);
            v.vector.y = t.x(3);
            v.vector.z = t.x(5);
            center_velocity_pub_->publish(v);
        }

        track_pub_->publish(out);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "TRACK mission=%s pnp=%zu accepted=%zu bodies=%zu selected=%u state=%s source_age_ms=%.1f",
                             selection_stage.c_str(), packet.armors.size(), observations.size(),
                             tracker_->targets.size(), selected ? selected->id : 0,
                             selected ? selected->state.c_str() : "NONE",
                             (now().seconds() - campus::stampSeconds(f.header.stamp)) * 1000.);
        markers_pub_->publish(markers);

        if (selected != nullptr) {
            publishState(selected->state);
        } else if (tracker_->targets.empty()) {
            publishState("LOST");
        } else {
            publishState("DETECTING");
        }

        if ((display_ || publish_image_) && ++frames_ % display_every_ == 0) {
            render(packet, selected, camera);
        }
    }

    //在 OpenCV 窗口中显示预测装甲板和调试信息
    void render(const campus_aim_interfaces::msg::ArmorFrame& packet, const campus::RigidTarget* selected,
                const campus::CameraGeometry& camera) {
        try {
            const campus::Frame& f = packet.frame;
            cv::Mat image = campus::decodeFrame(f);
            if (image.empty()) {
                return;
            }

            last_raw_image_ = image.clone();
            last_header_ = f.header;
            cv::Matx33d k = campus::intrinsics(f);
            cv::Mat d = campus::distortion(f);
            double aim_age = campus::stampSeconds(f.header.stamp) - aim_point_stamp_;

            if (selected && campus::finite(aim_point_) && std::abs(aim_age) <= max_age_) {
                cv::Point2f pixel;
                if (camera.project(aim_point_, k, d, pixel)) {
                    cv::drawMarker(image, pixel, {0, 0, 255}, cv::MARKER_CROSS, 18, 1);
                }
            }

            std::string text = "DETECTING";
            if (tracker_->targets.empty()) {
                text = "LOST";
            }

            if (selected) {
                const campus::RigidTarget& t = *selected;
                campus::drawReprojection(image, t, camera, k, d);
                if (show_future_) {
                    campus::drawReprojection(image, t, camera, k, d, horizon_, {255, 255, 0});
                }

                if (show_measurements_) {
                    for (size_t armor_index = 0; armor_index < packet.armors.size(); ++armor_index) {
                        const campus_aim_interfaces::msg::Armor& a = packet.armors[armor_index];
                        if (a.number && a.number != t.number) {
                            continue;
                        }

                        cv::Vec3d p(a.position.x, a.position.y, a.position.z);
                        double nearest = INFINITY;
                        for (int i = 0; i < t.count; ++i) {
                            nearest = std::min(nearest, cv::norm(p - t.position(i)));
                        }

                        if (nearest > 0.5) {
                            continue;
                        }

                        const std::array<geometry_msgs::msg::Point32, 4>& points = a.image_points;
                        if (std::hypot(points[0].x - points[1].x, points[0].y - points[1].y) < 2) {
                            continue;
                        }

                        // Measured light bars only, as in the reference screenshot; no second observation
                        // rectangle.
                        cv::line(image, cv::Point2f(points[0].x, points[0].y), cv::Point2f(points[3].x, points[3].y),
                                 {0, 160, 255}, 1);
                        cv::line(image, cv::Point2f(points[1].x, points[1].y), cv::Point2f(points[2].x, points[2].y),
                                 {0, 160, 255}, 1);
                    }
                }

                text = t.state + " #" + std::to_string(t.number) + " " + campus::armorIdentityName(t.number) +
                       " plates=" + std::to_string(t.count);
                int seen_count = std::count(t.seen.begin(), t.seen.begin() + t.count, true);
                text += " seen=" + std::to_string(seen_count) + "/" + std::to_string(t.count);

                if (!t.geometry_observable) {
                    text += " (radius prior)";
                }

                double fit_sum = 0;
                int fit_count = 0;
                for (size_t index = 0; index < packet.armors.size(); ++index) {
                    double error = measurementFitError(t, packet.armors[index], camera, k, d);

                    if (std::isfinite(error)) {
                        fit_sum += error;
                        ++fit_count;
                    }
                }

                std::string geometry_text = cv::format("r=%.3f/%.3f dz=%.3f/%.3f tilt=%.1f deg", t.x(8), t.x(9),
                                                       t.x(10), t.x(11), t.tilt * 180 / CV_PI);
                if (fit_count > 0) {
                    geometry_text += cv::format(" fit=%.1f px", fit_sum / fit_count);
                } else {
                    geometry_text += " fit=n/a";
                }

                cv::putText(image, geometry_text, {15, 47}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {220, 220, 220}, 1);
            }

            cv::putText(image, text, {15, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {220, 220, 220}, 1);

            std::string fire_text = fire_state_;
            if (fire_state_time_ < 0 || campus::steadySeconds() - fire_state_time_ > max_age_) {
                fire_text = "FIRE: WAIT_SOLVER";
            }

            if (!selected || selected->state != "TRACKING") {
                fire_text = "FIRE: BLOCK " + text;
            }

            cv::Scalar fire_color(0, 200, 255);
            if (fire_text.find("FIRE: ALLOW") == 0) {
                fire_color = cv::Scalar(0, 255, 0);
            }

            size_t detail_start = fire_text.find(" raw=");
            std::string fire_title = fire_text.substr(0, detail_start);
            cv::putText(image, fire_title, {15, 69}, cv::FONT_HERSHEY_SIMPLEX, 0.45, fire_color, 1);

            if (detail_start != std::string::npos) {
                cv::putText(image, fire_text.substr(detail_start + 1), {15, 90}, cv::FONT_HERSHEY_SIMPLEX, 0.4,
                            fire_color, 1);
            }

            std::string detection_text = "DET lights=" + std::to_string(packet.light_count) +
                                         " pairs=" + std::to_string(packet.candidate_count) +
                                         " known=" + std::to_string(packet.classified_count) +
                                         " PnP=" + std::to_string(packet.armors.size());
            cv::putText(image, detection_text, {15, 112}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {220, 220, 220}, 1);
            if (image_pub_) {
                image_pub_->publish(*cv_bridge::CvImage(f.header, "bgr8", image).toImageMsg());
            }

            if (display_) {
                campus::showFrame(window_, image);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Reprojection: %s", e.what());
        }
    }

    campus::FrameGuard guard_;
    campus::LockedTargetSelector selector_;
    std::unique_ptr<campus::MultiTargetTracker> tracker_;
    rclcpp::Subscription<campus_aim_interfaces::msg::ArmorFrame>::SharedPtr observations_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fire_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr aim_point_sub_;
    rclcpp::Publisher<campus_aim_interfaces::msg::TrackFrame>::SharedPtr track_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr markers_pub_;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_, center_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_pub_, center_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_;
    cv::Mat last_raw_image_;
    std_msgs::msg::Header last_header_;
    cv::Vec3d offset_;

    std::string window_;
    std::string fire_state_;
    cv::Vec3d aim_point_{NAN, NAN, NAN};
    double aim_point_stamp_ = -1;
    double fire_state_time_ = -1;
    double scale_;
    double horizon_;
    double max_age_;

    double last_receive_ = -1;
    int display_every_;
    int target_number_;
    bool mobile_only_ = false;
    bool acquire_detecting_ = false;
    bool field_order_ = false;
    bool strict_order_ = false, ignore_one_ = false, sentry_last_ = false;
    std::string mission_stage_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;

    bool display_;
    bool show_future_;
    bool show_measurements_;
    bool publish_image_;
    uint64_t frames_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<ArmorTrackerNode>());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "tracker: %s\n", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}