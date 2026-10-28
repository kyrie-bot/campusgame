#include "detection/armor_detection.hpp"
#include "classification/number_classifier.hpp"
#include "common/ros_aim_utils.hpp"
#include "geometry/camera_motion_noise.hpp"

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <functional>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <campus_aim_interfaces/msg/armor_frame.hpp>
#include <campus_aim_interfaces/msg/track_frame.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <filesystem>
#include <chrono>

class ArmorDetectorNode : public rclcpp::Node {
  public:
    ArmorDetectorNode() : Node("armor_detector") {
        cv::setNumThreads(std::max(1, campus::intParam(*this, "opencv_threads", 2)));

        campus::DetectionConfig c;
        c.color = campus::intParam(*this, "detect_color", 0);
        if (c.color != 0 && c.color != 1) {
            throw std::invalid_argument("detect_color must be 0 red or 1 blue");
        }

        int default_brightness = 180;
        if (c.color == 0) {
            default_brightness = 120;
        }

        c.brightness = campus::intParam(*this, "binary_threshold", default_brightness);
        c.color_difference = campus::intParam(*this, "color_difference", 35);
        c.min_light_length = campus::numberParam(*this, "light.min_length", 4);
        c.min_ratio = campus::numberParam(*this, "light.min_ratio", 0.04);
        c.max_ratio = campus::numberParam(*this, "light.max_ratio", 0.55);
        c.max_tilt = campus::numberParam(*this, "light.max_angle", 45);
        c.min_length_ratio = campus::numberParam(*this, "armor.min_light_ratio", 0.5);
        c.max_angle_difference = campus::numberParam(*this, "armor.max_light_angle_diff", 18);

        c.max_center_offset = campus::numberParam(*this, "armor.max_center_offset", 0.5);
        c.min_spacing = campus::numberParam(*this, "armor.min_small_center_distance", 0.7);
        c.large_spacing = campus::numberParam(*this, "armor.min_large_center_distance", 3.3);
        c.max_spacing = campus::numberParam(*this, "armor.max_large_center_distance", 5.5);

        detector_ = std::make_unique<campus::ArmorDetectionEngine>(c);

        bool enabled = declare_parameter<bool>("number.enabled", true);//是否开启数字识别
        confidence_ = campus::numberParam(*this, "number.confidence_threshold", 0.7);
        filter_unknown_ = declare_parameter<bool>("number.filter_unknown", false);
        std::string path = declare_parameter<std::string>("number.model_path", "");

        //在确定数字识别模型的路径，并在启用识别时创建分类器
        if (path.empty()) {
            path = ament_index_cpp::get_package_share_directory("stage1_comm_test") + "/models/armor_num_cnn.onnx";
        }

        if (enabled) {
            //检查模型文件是否存在且是普通文件
            if (!std::filesystem::is_regular_file(path)) {
                throw std::runtime_error("Number model missing: " + path +
                                         "; rebuild or set number.model_path to an absolute ONNX path");
            }

            try {
                // 尝试创建分类器
                classifier_ = std::make_unique<NumberClassifier>(path);
            } catch (const std::exception& e) {
                throw std::runtime_error("Cannot run ONNX " + path + ": " + e.what() +
                                         "; export with train_cnn.py --export-only, opset 11/dynamo=False");
            }

            RCLCPP_INFO(get_logger(), "CNN: %s (20x20 grayscale; 1..5 ROBOT, 6 OUTPOST, 7 SENTRY, 8 BASE)",
                        path.c_str());
        }

        small_width_ = campus::numberParam(*this, "pnp.small_width_mm", 135) / 1000;
        large_width_ = campus::numberParam(*this, "pnp.large_width_mm", 225) / 1000;
        small_height_ = campus::numberParam(*this, "pnp.small_height_mm", 55) / 1000;
        large_height_ = campus::numberParam(*this, "pnp.large_height_mm", 55) / 1000;
        force_small_armor_ = declare_parameter<bool>("pnp.force_small_armor", false);
        max_error_ = campus::numberParam(*this, "pnp.max_reprojection_error_px", 4);
        refine_yaw_ = declare_parameter<bool>("pnp.refine_yaw", true);
        tilt_ = campus::numberParam(*this, "pnp.armor_tilt_deg", 0) * CV_PI / 180;

        outpost_tilt_ = campus::numberParam(*this, "pnp.outpost_tilt_deg", 0) * CV_PI / 180;
        image_noise_ = campus::numberParam(*this, "pnp.image_noise_px", 1.5);

        campus::CameraMotionNoiseConfig motion_config;
        motion_config.enabled = declare_parameter<bool>("pnp.motion_noise.enabled", false);
        motion_config.timing_sigma_s = campus::numberParam(*this, "pnp.motion_noise.timing_sigma_ms", 0) / 1000;
        motion_config.max_rate_rad_s = campus::numberParam(*this, "pnp.motion_noise.max_rate_rad_s", 20);
        motion_config.max_bearing_sigma_rad =
            campus::numberParam(*this, "pnp.motion_noise.max_bearing_sigma_deg", 1.72) * CV_PI / 180;

        motion_noise_ = std::make_unique<campus::CameraMotionNoise>(motion_config);

        scale_ = campus::numberParam(*this, "scene_units_per_meter", 3);
        offset_ = campus::offsetParams(*this);
        max_age_ = campus::numberParam(*this, "max_frame_age_ms", 200) / 1000;
        process_every_ = std::max(1, campus::intParam(*this, "process_every_n_frames", 1));
        display_every_ = std::max(1, campus::intParam(*this, "display_every_n_frames", 2));
        display_ = declare_parameter<bool>("display", true);

        if (small_width_ <= 0 || large_width_ <= 0 || small_height_ <= 0 || large_height_ <= 0 || scale_ <= 0 ||
            max_error_ <= 0 || max_age_ <= 0) {
            throw std::invalid_argument("Invalid PnP/frame parameters");
        }

        observations_pub_ =
            create_publisher<campus_aim_interfaces::msg::ArmorFrame>("/armor/observations", campus::latestQoS());
        poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/armor/measurements", campus::latestQoS());
        point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/armor/measurement", campus::latestQoS());
        motion_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/armor/camera_motion", campus::latestQoS());
        timing_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/armor/timing", campus::latestQoS());

        tracked_sub_ = create_subscription<campus_aim_interfaces::msg::TrackFrame>(
            "/tracker/targets", campus::latestQoS(),
            std::bind(&ArmorDetectorNode::trackedCallback, this, std::placeholders::_1));
        frame_sub_ = create_subscription<campus::Frame>(
            "/autoaim/frame", campus::latestQoS(),
            std::bind(&ArmorDetectorNode::frameCallback, this, std::placeholders::_1));

        if (display_) {
            cv::namedWindow("armor_detector", cv::WINDOW_NORMAL);
            cv::namedWindow("binary_image", cv::WINDOW_NORMAL);
        }

        const char* enemy_color = "blue";
        if (c.color == 0) {
            enemy_color = "red";
        }

        RCLCPP_INFO(get_logger(), "Enemy color: %s; visual observations -> /armor/observations", enemy_color);
    }

    ~ArmorDetectorNode() override {
        if (display_) {
            cv::destroyWindow("armor_detector");
            cv::destroyWindow("binary_image");
            if (classifier_) {
                cv::destroyWindow("number_crops");
            }
        }
    }

  private:
    static bool armorImageDistanceLess(const campus_aim_interfaces::msg::Armor& a,
                                       const campus_aim_interfaces::msg::Armor& b) {
        return a.image_center_distance < b.image_center_distance;
    }

    void frameCallback(campus::Frame::ConstSharedPtr frame) {
        process(*frame);
    }

    // 检查目标和新鲜度，不能跨目标套用旧姿态
    void trackedCallback(campus_aim_interfaces::msg::TrackFrame::ConstSharedPtr packet) {
        tracked_ = packet;
    }

    //当前检测到的这块装甲板 armor，位置上是否符合之前跟踪的前哨站装甲板
    bool matchesOutpost(const campus::ArmorDetection& armor, const campus::Frame& frame,
                        const campus::CameraGeometry& camera, const cv::Matx33d& k, const cv::Mat& distortion) const {                 
        if (!tracked_ || tracked_->reset || tracked_->frame.header.frame_id != frame.header.frame_id ||
            campus::sessionKey(tracked_->frame) != campus::sessionKey(frame)) {
            return false;
        }

        //当前帧比跟踪结果晚了多久
        double dt = campus::stampSeconds(frame.header.stamp) - campus::stampSeconds(tracked_->frame.header.stamp);
        if (dt < 0 || dt > 0.2) {
            return false;
        }

        //装甲板对角线的像素长度
        double diagonal = cv::norm(armor.points[0] - armor.points[2]);

        //从跟踪目标中筛选前哨站
        for (const campus_aim_interfaces::msg::Target& target : tracked_->targets) {
            if (target.number != 6 || target.armor_count != 3 ||
                (target.state != "TRACKING" && target.state != "TEMP_LOST")) {
                continue;
            }

            for (int slot = 0; slot < 3; ++slot) {
                if (!target.seen[slot]) {
                    continue;
                }

                double angle = target.yaw + target.yaw_rate * dt + slot * 2 * CV_PI / 3;
                //然后根据目标中心、半径和角度，算装甲板的三维位置
                cv::Vec3d position(target.center.x - target.radius * std::cos(angle),
                                   target.center.y - target.radius * std::sin(angle), target.center.z);
                if (slot == 1) {
                    position[2] += target.height_difference;
                } else if (slot == 2) {
                    position[2] += target.height_difference_second;
                }

                cv::Point2f projected;
                if (camera.project(position, k, distortion, projected) &&
                    cv::norm(projected - armor.center) < std::max(6.0, diagonal * 0.75)) {
                    return true;
                }
            }
        }

        return false;
    }

    void process(const campus::Frame& f) {
        bool reset = false;
        campus_aim_interfaces::msg::ArmorFrame packet;//packet是一份装甲板检测结果
        packet.frame = f;

        if (!guard_.check(f, now().seconds(), max_age_, reset) || !campus::validCamera(f)) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000, "DETECTION_DROP age_ms=%.1f camera_valid=%d reset=%d",
                (now().seconds() - campus::stampSeconds(f.header.stamp)) * 1000., campus::validCamera(f), reset);
            if (reset || !campus::validCamera(f)) {
                packet.reset = true;
                motion_noise_->reset();
                observations_pub_->publish(packet);//发布一个带重置标记的消息，通知下游
            }

            return;
        }

        packet.reset = reset || f.state.health <= 0;
        if (packet.reset) {
            tracked_.reset();
            motion_noise_->reset();
        }

        //根据相机角度变化估计运动不确定性
        //bearing从观察者的位置看，目标在哪个方向
        campus::CameraMotionUncertainty motion =
            motion_noise_->update(f.state.yaw_degrees, f.state.pitch_degrees, campus::stampSeconds(f.header.stamp));

        geometry_msgs::msg::Vector3Stamped motion_debug;
        motion_debug.header = f.header;
        motion_debug.header.frame_id = "world_optical_rates";

        if (motion.rate_valid) {
            motion_debug.vector.x = motion.yaw_rate;
            motion_debug.vector.y = motion.pitch_rate;
        } else {
            motion_debug.vector.x = motion_debug.vector.y = std::numeric_limits<double>::quiet_NaN();
        }

        motion_debug.vector.z = motion.bearing_variance;
        motion_pub_->publish(motion_debug);
        if (!packet.reset && ++frames_ % process_every_) {
            return;
        }

        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        cv::Mat image;
        try {
            image = campus::decodeFrame(f);

            if (image.empty() || (f.camera_info.width && f.camera_info.width != unsigned(image.cols)) ||
                (f.camera_info.height && f.camera_info.height != unsigned(image.rows))) {
                packet.reset = true;
                motion_noise_->reset();
                observations_pub_->publish(packet);//图像为空或尺寸不符，就发布重置消息并结束当前帧
                return;
            }

            campus::CameraGeometry camera = campus::cameraGeometry(f, scale_, offset_);
            cv::Matx33d k = campus::intrinsics(f);
            cv::Mat d = campus::distortion(f);

            std::vector<campus::ArmorDetection> armors;
            if (f.state.health > 0) {
                armors = detector_->detect(image);
                packet.light_count = detector_->lights().size();
            }

            packet.candidate_count = armors.size();
            std::vector<NumberResult> numbers;
            geometry_msgs::msg::PoseArray poses;
            poses.header = f.header;
            poses.header.frame_id = "map";

            for (size_t armor_index_1 = 0; armor_index_1 < armors.size(); ++armor_index_1) {
                const campus::ArmorDetection& armor = armors[armor_index_1];
                bool large = armor.large && !force_small_armor_;//只有检测认为是大装甲板，并且没有启用“强制小装甲板”，才按大装甲板处理
                NumberResult n;

                if (classifier_) {
                    n = classifier_->classify(image, armor.points, large, confidence_);
                }//如果分类器已创建，就根据角点截取数字区域并分类，得到 n.number、n.confidence 等结果

                numbers.push_back(n);
                if (campus::knownArmorIdentity(n.number)) {
                    ++packet.classified_count;
                }//统计识别出已知身份的候选数量

                if (filter_unknown_ && n.number == 0) {
                    continue;
                }

                double width = small_width_;
                double height = small_height_;
                if (large) {
                    width = large_width_;
                    height = large_height_;
                }//确定装甲板实际尺寸及姿态求解选项

                campus::PoseOptions options;
                options.refine_yaw = refine_yaw_;
                options.image_noise_px = image_noise_;
                options.tilt = tilt_;

                if (n.number == 6 || (n.number == 0 && matchesOutpost(armor, f, camera, k, d))) {
                    options.tilt = outpost_tilt_;
                }//对于前哨站，使用专门的倾角

                campus::ArmorPose p =
                    campus::solveArmorPose(armor.points, width, height, k, d, camera, max_error_, options);
                if (!p.valid) {
                    continue;
                }

                //将位置、备选位置、角点、编号、置信度、尺寸等填入 o
                campus_aim_interfaces::msg::Armor o;
                o.position.x = p.position[0];
                o.position.y = p.position[1];
                o.position.z = p.position[2];
                o.alternative_position.x = p.alternative_position[0];
                o.alternative_position.y = p.alternative_position[1];
                o.alternative_position.z = p.alternative_position[2];
                o.alternative_yaw = p.alternative_yaw;

                o.has_alternative = p.has_alternative;
                o.tilt = p.tilt;
                for (int i = 0; i < 4; ++i) {
                    o.image_points[i].x = armor.points[i].x;
                    o.image_points[i].y = armor.points[i].y;
                }

                o.yaw = p.yaw;
                o.yaw_variance = p.yaw_variance + motion.bearing_variance;
                o.number = n.number;
                o.confidence = n.confidence;
                o.large = large;
                o.width_m = width;
                o.height_m = height;
                o.reprojection_error = p.error;

                double range = cv::norm(p.position - camera.origin);
                o.position_variance = std::pow(0.01 + 0.002 * range * range + 0.003 * p.error, 2);
                o.tangential_variance = std::pow(range * (image_noise_ + p.error) / std::min(k(0, 0), k(1, 1)), 2) +
                                        range * range * motion.bearing_variance;
                o.image_center_distance = cv::norm(armor.center - cv::Point2f(k(0, 2), k(1, 2)));

                packet.armors.push_back(o);
                geometry_msgs::msg::Pose pose;
                pose.position = o.position;
                pose.orientation.z = std::sin(p.yaw / 2);
                pose.orientation.w = std::cos(p.yaw / 2);
                poses.poses.push_back(pose);
            }

            std::sort(packet.armors.begin(), packet.armors.end(), armorImageDistanceLess);
            observations_pub_->publish(packet);

            geometry_msgs::msg::Vector3Stamped timing;
            timing.header = f.header;
            timing.header.frame_id = "source_age_process_output_ms";
            timing.vector.y =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            timing.vector.z = (now().seconds() - campus::stampSeconds(f.header.stamp)) * 1000;
            timing.vector.x = timing.vector.z - timing.vector.y;
            timing_pub_->publish(timing);

            std::string identities;
            for (const auto& o : packet.armors)
                identities += "#" + std::to_string(o.number) + " ";
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "DETECTION lights=%u pairs=%u known=%u pnp=%zu numbers=[%s] input_ms=%.1f "
                                 "process_ms=%.1f output_ms=%.1f",
                                 packet.light_count, packet.candidate_count, packet.classified_count,
                                 packet.armors.size(), identities.c_str(), timing.vector.x, timing.vector.y,
                                 timing.vector.z);

            poses_pub_->publish(poses);
            if (!packet.armors.empty()) {
                geometry_msgs::msg::PointStamped p;
                p.header = poses.header;
                p.point = packet.armors.front().position;
                point_pub_->publish(p);
            }

            if (display_ && frames_ % display_every_ == 0) {
                for (size_t armor_index = 0; armor_index < armors.size(); ++armor_index) {
                    const campus::ArmorDetection& a = armors[armor_index];
                    for (int i = 0; i < 4; ++i) {
                        cv::line(image, a.points[i], a.points[(i + 1) % 4], {0, 255, 0}, 2);
                    }
                }

                cv::Mat crops = cv::Mat::zeros(100, std::max(1, int(armors.size())) * 100, CV_8UC3);
                for (size_t i = 0; i < armors.size(); ++i) {
                    const NumberResult& n = numbers[i];
                    if (!n.image.empty()) {
                        cv::Mat enlarged;
                        cv::resize(n.image, enlarged, {80, 80});
                        cv::cvtColor(enlarged, crops(cv::Rect(i * 100, 20, 80, 80)), cv::COLOR_GRAY2BGR);
                    }

                    std::string label = "#" + std::to_string(n.number) + " " + campus::armorIdentityName(n.number) +
                                        " " + std::to_string(int(n.confidence * 100)) + "%";
                    cv::putText(image, label, armors[i].points[0] - cv::Point2f(0, 8), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                {0, 255, 255}, 1);
                }

                double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
                cv::putText(image,
                            "armors=" + std::to_string(packet.armors.size()) + " process=" + std::to_string(int(ms)) +
                                "ms",
                            {15, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255, 255, 255}, 1);
                campus::showFrame("armor_detector", image);

                if (!detector_->binary().empty()) {
                    cv::imshow("binary_image", detector_->binary());
                }

                if (classifier_) {
                    cv::imshow("number_crops", crops);
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Detection failed: %s", e.what());
            motion_noise_->reset();
            packet.armors.clear();
            packet.reset = true;
            observations_pub_->publish(packet);
        }
    }

    campus::FrameGuard guard_;
    campus_aim_interfaces::msg::TrackFrame::ConstSharedPtr tracked_;
    rclcpp::Subscription<campus_aim_interfaces::msg::TrackFrame>::SharedPtr tracked_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr timing_pub_;
    std::unique_ptr<campus::CameraMotionNoise> motion_noise_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr motion_pub_;
    std::unique_ptr<campus::ArmorDetectionEngine> detector_;
    std::unique_ptr<NumberClassifier> classifier_;

    rclcpp::Subscription<campus::Frame>::SharedPtr frame_sub_;
    rclcpp::Publisher<campus_aim_interfaces::msg::ArmorFrame>::SharedPtr observations_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
    double confidence_;
    double small_width_;
    bool force_small_armor_;
    double large_width_;

    double small_height_;
    double large_height_;
    double max_error_;
    double scale_;
    double max_age_;
    double tilt_;
    double outpost_tilt_;
    double image_noise_;

    cv::Vec3d offset_;
    bool display_;
    bool filter_unknown_;
    bool refine_yaw_;
    int process_every_;
    int display_every_;
    uint64_t frames_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<ArmorDetectorNode>());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "armor_detector: %s\n", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}