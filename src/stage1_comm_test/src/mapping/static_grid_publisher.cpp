#include <filesystem>
#include <memory>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mapping/occupancy_grid.hpp"

class StaticGridPublisher : public rclcpp::Node {
  public:
    StaticGridPublisher() : Node("static_grid_publisher") {
        std::string frame = declare_parameter<std::string>("frame_id", "arena_draft");
        bool geometry_verified = declare_parameter<bool>("geometry_verified", false);
        bool registration_verified = declare_parameter<bool>("registration_verified", false);
        bool publish_standard_map = declare_parameter<bool>("publish_standard_map", false);
        bool terrain_annotations = declare_parameter<bool>("publish_terrain_annotations", false);
        bool registered = frame == "unity_world_m" && registration_verified;
        bool draft = frame == "arena_draft" && !registration_verified;
        bool reference = frame == "arena_reference" && !registration_verified;

        bool campus = frame == "arena_campus" && !registration_verified;
        if ((!registered && !draft && !reference && !campus) || geometry_verified) {
            throw std::invalid_argument(
                "Require local draft/reference or checked registration; geometry remains unverified");
        }

        int width = declare_parameter<int>("width_cells", 280);
        int height = declare_parameter<int>("height_cells", 150);
        double resolution = declare_parameter<double>("resolution_m", 0.1);
        double origin_x = declare_parameter<double>("origin_x_m", 0.0);
        double origin_z = declare_parameter<double>("origin_z_m", 0.0);

        std::string directory = declare_parameter<std::string>("map_directory", "");
        std::filesystem::path maps = ament_index_cpp::get_package_share_directory("stage1_comm_test") + "/maps";
        if (directory.empty()) {
            directory = (maps / "arena_draft").string();
        } else if (std::filesystem::path(directory).is_relative()) {
            directory = (maps / directory).string();
        }

        if (registered) {
            checkRegistration(directory, width, height, resolution, origin_x, origin_z);
        } else if (origin_x != 0.0 || origin_z != 0.0) {
            throw std::invalid_argument("Local draft/reference map requires its original local origin");
        }

        campus::StaticOccupancyGrid raw(width, height, resolution, origin_x, origin_z);
        campus::StaticOccupancyGrid blocked(width, height, resolution, origin_x, origin_z);
        raw.loadPgm((std::filesystem::path(directory) / "occupancy.pgm").string());
        blocked.loadPgm((std::filesystem::path(directory) / "blocked.pgm").string());

        for (std::size_t i = 0; i < raw.data().size(); ++i) {
            if ((blocked.data()[i] != 0 && blocked.data()[i] != 100) ||
                (raw.data()[i] != 0 && blocked.data()[i] == 0)) {
                throw std::runtime_error("Blocked grid must contain every obstacle and unknown cell");
            }
        }

        rclcpp::QoS qos(1);
        qos.reliable().transient_local();
        raw_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/arena/map", qos);

        if (publish_standard_map) {
            standard_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", qos);
        }

        blocked_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/arena/blocked_map", qos);
        status_publisher_ = create_publisher<std_msgs::msg::String>("/arena/map_status", qos);

        nav_msgs::msg::OccupancyGrid message;
        message.header.stamp = now();
        message.header.frame_id = frame;
        message.info.map_load_time = message.header.stamp;
        message.info.resolution = static_cast<float>(resolution);
        message.info.width = static_cast<uint32_t>(width);
        message.info.height = static_cast<uint32_t>(height);
        message.info.origin.position.x = origin_x;

        message.info.origin.position.y = origin_z;
        message.info.origin.orientation.w = 1.0;

        message.data = raw.data();
        raw_publisher_->publish(message);
        if (standard_publisher_) {
            standard_publisher_->publish(message);
        }

        message.data = blocked.data();
        blocked_publisher_->publish(message);
        if (campus || terrain_annotations) {
            publishTerrainAnnotations(directory, message);
        }

        std_msgs::msg::String status;

        if (registered) {
            status.data = "REGISTERED_PRIOR: paired-point checks passed; geometry and passage clearance unverified";
        } else if (campus) {
            status.data = "SCHEMATIC_ONLY: four ramp gates and static outpost exclusion; screenshot projection is not "
                          "metric measurement";
        } else if (reference) {
            status.data = "REFERENCE: closed highland ledges; four confirmed ramp gates; nominal scale; geometry "
                          "unverified; no Unity registration";
        } else {
            status.data = "DRAFT: geometry unverified; no Unity registration; static prior only";
        }

        status_publisher_->publish(status);
        RCLCPP_INFO(get_logger(), "Published prior %d x %d, %.3f m/cell, frame=%s, origin=(%.2f, %.2f)", width, height,
                    resolution, frame.c_str(), origin_x, origin_z);
    }

  private:
    void publishTerrainAnnotations(const std::string& directory, const nav_msgs::msg::OccupancyGrid& grid) {
        cv::FileStorage file((std::filesystem::path(directory) / "manifest.json").string(), cv::FileStorage::READ);
        if (!file.isOpened()) {
            throw std::runtime_error("Missing campus vector manifest");
        }

        terrain_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/arena/terrain_markers", rclcpp::QoS(1).reliable().transient_local());

        visualization_msgs::msg::MarkerArray output;
        double width_m = grid.info.width * grid.info.resolution;
        double height_m = grid.info.height * grid.info.resolution;
        int id = 0;

        cv::FileNode edges = file["terrain_edges"];
        for (cv::FileNodeIterator it = edges.begin(); it != edges.end(); ++it) {
            visualization_msgs::msg::Marker line;
            line.header = grid.header;
            line.ns = "unverified_terrain";
            line.id = id++;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.pose.orientation.w = 1;
            line.scale.x = 0.08;
            line.color.r = 1;

            line.color.g = 0.7;
            line.color.a = 1;
            cv::FileNode vertices = (*it)["points_uv"];

            for (cv::FileNodeIterator vertex = vertices.begin(); vertex != vertices.end(); ++vertex) {
                geometry_msgs::msg::Point point;
                point.x = (*vertex)[0].real() * width_m;
                point.y = (1 - (*vertex)[1].real()) * height_m;
                point.z = 0.12;
                line.points.push_back(point);
            }

            output.markers.push_back(line);
        }

        // Remove cached frames/labels from earlier terrain marker versions.
        visualization_msgs::msg::Marker clear;
        clear.header = grid.header;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        output.markers.insert(output.markers.begin(), clear);

        for (const char* key : {"ramp_surface_triangles_xy_m", "walkable_highland_triangles_xy_m"}) {
            cv::FileNode triangles = file[key];
            if (triangles.empty())
                continue;
            if (triangles.size() % 3 != 0) {
                throw std::runtime_error("Invalid walkable terrain triangle list");
            }

            visualization_msgs::msg::Marker surface;
            surface.header = grid.header;
            surface.ns = std::string(key) == "ramp_surface_triangles_xy_m" ? "ramp_surfaces" : "walkable_highlands";
            surface.id = id++;
            surface.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            surface.pose.orientation.w = 1;
            surface.scale.x = surface.scale.y = surface.scale.z = 1;
            surface.color.r = surface.color.g = surface.color.b = 0.70;

            surface.color.a = 1;
            for (cv::FileNodeIterator vertex = triangles.begin(); vertex != triangles.end(); ++vertex) {
                geometry_msgs::msg::Point point;
                point.x = (*vertex)[0].real();
                point.y = (*vertex)[1].real();
                point.z = 0.025;
                surface.points.push_back(point);
            }

            output.markers.push_back(surface);
        }

        cv::FileNode structures = file["unresolved_structures"];
        for (cv::FileNodeIterator it = structures.begin(); it != structures.end(); ++it) {
            visualization_msgs::msg::Marker label;
            label.header = grid.header;
            label.ns = "unverified_footprints";
            label.id = id++;
            label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            label.pose.orientation.w = 1;
            label.pose.position.x = (*it)["centre_uv"][0].real() * width_m;
            label.pose.position.y = (1 - (*it)["centre_uv"][1].real()) * height_m;

            label.pose.position.z = 0.6;
            label.scale.z = 0.35;
            label.color.r = 1;
            label.color.g = 0.7;
            label.color.a = 1;
            std::string name;
            (*it)["name"] >> name;

            if ((*it)["blocked"].real() != 0) {
                label.text = name + "\nBLOCKED / SIZE UNVERIFIED";
            } else {
                label.text = name + "\nFOOTPRINT UNVERIFIED";
            }

            output.markers.push_back(label);
        }

        terrain_publisher_->publish(output);
    }

    // 避免只改 frame 名字，就把未标定的草稿当成世界地图。
    void checkRegistration(const std::string& directory, int width, int height, double resolution, double origin_x,
                           double origin_z) {
        cv::FileStorage report((std::filesystem::path(directory) / "registration.json").string(),
                               cv::FileStorage::READ);
        if (!report.isOpened()) {
            throw std::runtime_error("Missing registration report");
        }

        cv::FileNode origin = report["origin_xz_m"];
        std::string report_frame;
        report["frame_id"] >> report_frame;

        if (report_frame != "unity_world_m" || report["registration_verified"].real() != 1 ||
            report["geometry_verified"].real() != 0 || report["width_cells"].real() != width ||
            report["height_cells"].real() != height || !origin.isSeq() || origin.size() != 2 ||
            !std::isfinite(resolution) || !std::isfinite(origin_x) || !std::isfinite(origin_z) ||
            std::abs(report["resolution_m"].real() - resolution) > 1e-9 ||
            std::abs(origin[0].real() - origin_x) > 1e-9 || std::abs(origin[1].real() - origin_z) > 1e-9) {
            throw std::runtime_error("Runtime parameters disagree with registration report");
        }
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr standard_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr blocked_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr terrain_publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        std::shared_ptr<StaticGridPublisher> node = std::make_shared<StaticGridPublisher>();
        rclcpp::spin(node);
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("static_grid_publisher"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}