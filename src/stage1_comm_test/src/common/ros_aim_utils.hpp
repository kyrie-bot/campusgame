#pragma once

#include "geometry/aim_geometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tdt_interface/msg/auto_aim_frame.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sstream>

namespace campus {
typedef tdt_interface::msg::AutoAimFrame Frame;

inline double stampSeconds(const builtin_interfaces::msg::Time& t) {
    return double(t.sec) + t.nanosec * 1e-9;
}

inline bool sameStamp(const builtin_interfaces::msg::Time& a, const builtin_interfaces::msg::Time& b) {
    return a.sec == b.sec && a.nanosec == b.nanosec;
}

inline cv::Matx33d intrinsics(const Frame& f) {
    return {f.camera_info.k[0], f.camera_info.k[1], f.camera_info.k[2], f.camera_info.k[3], f.camera_info.k[4],
            f.camera_info.k[5], f.camera_info.k[6], f.camera_info.k[7], f.camera_info.k[8]};
}

inline cv::Mat distortion(const Frame& f) {
    if (f.camera_info.d.empty()) {
        return cv::Mat();
    }

    return cv::Mat(f.camera_info.d).clone();
}

inline cv::Mat decodeFrame(const Frame& f) {
    if (f.image.data.empty() == f.compressed_image.data.empty()) {
        return cv::Mat();
    }

    if (!f.compressed_image.data.empty()) {
        return cv::imdecode(f.compressed_image.data, cv::IMREAD_COLOR);
    }

    return cv_bridge::toCvCopy(f.image, sensor_msgs::image_encodings::BGR8)->image;
}

inline rclcpp::QoS latestQoS() {
    return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

inline double numberParam(rclcpp::Node& n, const std::string& key, double value) {
    return n.declare_parameter<double>(key, value);
}

inline int intParam(rclcpp::Node& n, const std::string& key, int value) {
    return n.declare_parameter<int>(key, value);
}

inline cv::Vec3d offsetParams(rclcpp::Node& n) {
    double x = numberParam(n, "camera.offset_x_m", 0);
    double y = numberParam(n, "camera.offset_y_m", 0);
    double z = numberParam(n, "camera.offset_z_m", 0);
    return {x, y, z};
}

inline CameraGeometry cameraGeometry(const Frame& f, double scale, cv::Vec3d offset) {
    return CameraGeometry({f.state.position.x, f.state.position.y, f.state.position.z}, f.state.yaw_degrees,
                          f.state.pitch_degrees, scale, offset);
}

// CMake checks the installed message definitions. Old interfaces have no session_id.
// Ordinary overloads keep the same compatibility without template field detection.
inline std::string sessionKey(const Frame& value) {
#ifdef CAMPUS_FRAME_HAS_SESSION_ID
    std::ostringstream stream;
    stream << value.session_id;
    return stream.str();
#else
    (void)value;
    return std::string();
#endif
}

inline std::string sessionKey(const tdt_interface::msg::AutoAimState& value) {
#ifdef CAMPUS_STATE_HAS_SESSION_ID
    std::ostringstream stream;
    stream << value.session_id;
    return stream.str();
#else
    (void)value;
    return std::string();
#endif
}

class FrameGuard {
  public:
    bool check(const Frame& f, double now, double max_age, bool& reset) {
        reset = false;
        double time = stampSeconds(f.header.stamp);
        double age = now - time;

        if (time <= 0 || !std::isfinite(age) || age < -0.1) {
            reset = true;
            return false;
        }

        // A late packet is dropped; only the watchdog or a real session change resets the model.
        if (age > max_age) {
            return false;
        }

        builtin_interfaces::msg::Time image_stamp = f.image.header.stamp;
        if (f.image.data.empty()) {
            image_stamp = f.compressed_image.header.stamp;
        }

        if (!sameStamp(f.header.stamp, f.state.header.stamp) ||
            !sameStamp(f.header.stamp, f.camera_info.header.stamp) || !sameStamp(f.header.stamp, image_stamp)) {
            reset = true;
            return false;
        }

        if (!finite({f.state.position.x, f.state.position.y, f.state.position.z}) ||
            !std::isfinite(f.state.yaw_degrees) || !std::isfinite(f.state.pitch_degrees)) {
            reset = true;
            return false;
        }

        std::string session = sessionKey(f) + "/" + sessionKey(f.state) + "/" + f.header.frame_id;
        if (last_time_ > 0 && session != session_) {
            reset = true;
            last_time_ = -1;
        }

        // Backward wall time means delayed packets; increasing time with sequence reset means a new match.
        if (last_time_ > 0 && time <= last_time_) {
            return false;
        }

        // Some game restarts keep the transport sequence increasing. Use
        // the same match-clock restart signal as the planner and gimbal.
        const bool clock_restart=last_time_>0 && std::isfinite(f.state.remaining_seconds) &&
                                 f.state.remaining_seconds>remaining_seconds_+2.;
        if(clock_restart)reset=true;

        if (last_time_ > 0 && f.state.frame_sequence <= sequence_) {
            if (f.state.frame_sequence == sequence_ && !clock_restart) {
                return false;
            }

            reset = true;
        }

        if (last_time_ > 0 && time - last_time_ > 0.5) {
            reset = true;
        }

        last_time_ = time;
        sequence_ = f.state.frame_sequence;
        remaining_seconds_ = f.state.remaining_seconds;
        session_ = session;
        return true;
    }

    void clear() {
        last_time_ = -1;
    }

  private:
    double last_time_ = -1;
    double remaining_seconds_ = 0;
    uint64_t sequence_ = 0;
    std::string session_;
};

inline bool validCamera(const Frame& f) {
    for (size_t value_index = 0; value_index < f.camera_info.k.size(); ++value_index) {
        double v = f.camera_info.k[value_index];
        if (!std::isfinite(v)) {
            return false;
        }
    }

    for (size_t value_index = 0; value_index < f.camera_info.d.size(); ++value_index) {
        double v = f.camera_info.d[value_index];
        if (!std::isfinite(v)) {
            return false;
        }
    }

    return f.camera_info.k[0] > 0 && f.camera_info.k[4] > 0 &&
           (f.camera_info.d.empty() || f.camera_info.d.size() == 4 || f.camera_info.d.size() == 5 ||
            f.camera_info.d.size() == 8 || f.camera_info.d.size() == 12 || f.camera_info.d.size() == 14);
}

inline bool showFrame(const std::string& window, const cv::Mat& image) {
    cv::imshow(window, image);
    int key = cv::waitKey(1);
    if (key == 27 || key == 'q' || key == 'Q') {
        rclcpp::shutdown();
        return false;
    }

    return true;
}

} // namespace campus
