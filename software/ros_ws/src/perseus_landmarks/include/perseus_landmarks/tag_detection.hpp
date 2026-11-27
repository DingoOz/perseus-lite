#pragma once

/// @file tag_detection.hpp
/// @brief AprilTag detection and pose estimation on one grey image (OpenCV only).
///
/// Uses OpenCV's ArUco module with its AprilTag dictionaries and AprilTag corner
/// refinement, then a square-specific PnP (IPPE_SQUARE) per tag. RoboStack's
/// apriltag_ros cannot join the robot's Pixi environment (it is built against a newer
/// ROS snapshot), and OpenCV is already there.

#include <functional>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <vector>

namespace perseus_landmarks
{
    /// @brief Pinhole camera of an already-rectified image.
    struct CameraModel
    {
        double fx{0.0};
        double fy{0.0};
        double cx{0.0};
        double cy{0.0};
        /// @brief Distortion coefficients (empty for a rectified image).
        std::vector<double> distortion;
    };

    /// @brief One tag found in an image.
    struct DetectedTag
    {
        int id{0};
        double size_m{0.0};
        /// @brief Tag pose in the camera optical frame: rotation (Rodrigues) and
        ///        translation, OpenCV convention (tag x right, y up, z towards camera).
        cv::Vec3d rvec;
        cv::Vec3d tvec;
        double reprojection_error_px{0.0};
        /// @brief Second-best IPPE solution's error over the best (>= 1). Below ~3 the
        ///        orientation is ambiguous; see TagDetection.msg.
        double ambiguity_ratio{1.0};
        double distance_m{0.0};
        /// @brief Angle between the camera-to-tag ray and the tag's normal.
        double view_angle_rad{0.0};
        /// @brief Corners in pixels (top-left, top-right, bottom-right, bottom-left).
        std::vector<cv::Point2f> corners;
    };

    /// @brief Detection settings.
    struct DetectorParams
    {
        /// @brief "16h5", "25h9", "36h10" or "36h11".
        std::string family{"36h11"};
        /// @brief Smallest tag, as a fraction of the image's larger side (perimeter
        ///        fraction in OpenCV terms). Smaller finds tags further away, slower.
        double min_marker_perimeter_rate{0.03};
        /// @brief Corner refinement: "subpix" (default: detects furthest and at the most
        ///        oblique angles), "apriltag" (most accurate corners, shorter range) or
        ///        "none". Measured trade-off in docs/source/systems/software/landmark-localisation.md.
        std::string corner_refinement{"subpix"};
    };

    /// @brief Whether a family name is supported.
    bool family_supported(const std::string& family);

    /// @brief Finds every tag in @p gray (CV_8UC1) and estimates its pose.
    /// @param size_of Edge length (black square, metres) for a tag id.
    /// @throws std::invalid_argument on an unsupported family or a non-grey image.
    std::vector<DetectedTag> detect_tags(const cv::Mat& gray, const CameraModel& camera, const DetectorParams& params,
                                         const std::function<double(int)>& size_of);

    /// @brief Renders tag @p id of @p family as a grey image with a one-cell white
    ///        border (for tests and simulation textures). @p cell_px pixels per cell.
    cv::Mat render_tag(const std::string& family, int id, int cell_px);

}
