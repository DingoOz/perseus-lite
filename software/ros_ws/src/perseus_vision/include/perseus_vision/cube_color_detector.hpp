#pragma once

/// @file cube_color_detector.hpp
/// @brief Colour-based cube detection and 6-DoF pose estimation ROS2 node.
///
/// Detects 100 mm painted cubes (Red, Blue, Green, White) using HSV colour
/// segmentation, fits a quadrilateral to the visible face contour, and
/// estimates the face's 6-DoF pose with cv::solvePnP using the known cube-face
/// edge length. Mirrors the structure of the ArUco detector node so the same
/// downstream consumers (ObjectDetections topic, TF, RViz markers, service)
/// can be reused.

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <unordered_set>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "perseus_interfaces/msg/object_detections.hpp"
#include "perseus_interfaces/srv/detect_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace perseus_vision
{

    /// Stable colour codes used as detection IDs on the ObjectDetections topic.
    /// Order matches the existing ML cube_detector's CLASS_NAMES so consumers
    /// can switch back-ends without remapping.
    enum class CubeColor : int32_t
    {
        BLUE = 0,
        GREEN = 1,
        RED = 2,
        WHITE = 3
    };

    /// HSV inclusive range for one colour. Red wraps the hue circle so it has
    /// two ranges; all other colours leave the second range unused (h_high2<0).
    struct HsvRange
    {
        int h_low{0};
        int h_high{0};
        int s_low{0};
        int s_high{255};
        int v_low{0};
        int v_high{255};
        // Optional second hue band for colours that wrap (e.g. red).
        int h_low2{-1};
        int h_high2{-1};
    };

    /// @brief ROS2 node for detecting painted cubes by colour and estimating
    ///        their 6-DoF pose from a single visible face.
    class CubeColorDetector : public rclcpp::Node
    {
    public:
        using DetectObjects = perseus_interfaces::srv::DetectObjects;

        CubeColorDetector();

    private:
        static constexpr int kQosDepth = 10;
        static constexpr size_t kNumColors = 4;

        // Callbacks
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
        void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        // Core
        void process_image(const cv::Mat& frame, const std_msgs::msg::Header& header);

        /// Build the HSV mask for a single colour, including red's wrap-around band.
        cv::Mat build_color_mask(const cv::Mat& hsv, const HsvRange& range) const;

        /// Find candidate cube faces in a binary mask, returning their ordered
        /// corner pixels (TL, TR, BR, BL) for each accepted face.
        std::vector<std::array<cv::Point2f, 4>> extract_face_quads(
            const cv::Mat& mask) const;

        /// Project the four image-point corners onto the cube face object model
        /// via solvePnP, transform into the output frame, broadcast TF, and
        /// append the detection to the cached vectors. Returns false if the
        /// TF lookup fails.
        bool transform_and_publish_face(
            const std_msgs::msg::Header& header,
            CubeColor color,
            int instance_index,
            const std::array<cv::Point2f, 4>& image_points,
            const cv::Mat& camera_matrix,
            const cv::Mat& dist_coeffs,
            cv::Mat& annotated_frame);

        // Service
        void handle_request(
            const std::shared_ptr<DetectObjects::Request> request,
            std::shared_ptr<DetectObjects::Response> response);

        // Helpers
        static const char* color_name(CubeColor color);
        static cv::Scalar color_bgr(CubeColor color);
        static std::array<cv::Point2f, 4> order_quad_corners(
            const std::vector<cv::Point>& polygon);

        // -------------------------
        // Parameters
        // -------------------------
        double cube_size_{0.10};    // Cube edge length in metres (100 mm default).
        double axis_length_{0.05};  // Drawn axis length on the annotated image.
        double min_contour_area_{300.0};
        double max_aspect_ratio_{1.6};    // Reject elongated blobs (face should be near-square).
        double min_face_solidity_{0.85};  // contour_area / convex_hull_area; rejects spiky blobs.

        std::string camera_frame_{"camera_optical_frame"};
        std::string tf_output_frame_{"odom"};

        std::string input_img_{"/image_raw"};
        std::string output_img_{"/perseus_vision/cube_color/image"};
        std::string output_topic_{"/perseus_vision/cube_color/detections"};
        std::string output_markers_topic_{"/perseus_vision/cube_color/markers"};
        std::string camera_info_topic_{"/camera_info"};

        bool should_publish_tf_{true};
        bool should_publish_img_{true};
        bool is_compressed_io_{false};
        bool should_use_camera_info_{true};
        bool should_publish_output_{true};

        // Per-colour HSV ranges (declared as parameters with sane defaults).
        std::array<HsvRange, kNumColors> hsv_ranges_;
        std::unordered_set<int32_t> enabled_color_codes_;

        // Camera intrinsics
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;

        // TF
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

        // ROS IO
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

        rclcpp::Service<DetectObjects>::SharedPtr service_;

        rclcpp::Publisher<perseus_interfaces::msg::ObjectDetections>::SharedPtr detection_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

        // Cached detections for the service and the published topic.
        std::mutex detections_mutex_;
        rclcpp::Time latest_timestamp_{0, 0, RCL_ROS_TIME};
        std::vector<int32_t> latest_ids_;
        std::vector<geometry_msgs::msg::Pose> latest_poses_;
        std::vector<sensor_msgs::msg::RegionOfInterest> latest_regions_of_interest_;
        cv::Mat latest_frame_;

        // Camera calibration synchronization
        std::mutex camera_matrix_mutex_;
    };

}  // namespace perseus_vision
