/// @file tag_detector_node.cpp
/// @brief tag_detector: rectified grey/colour image in, perseus_interfaces/TagDetectionArray out.
///
/// Subscribes:  image_topic (sensor_msgs/Image), camera_info_topic (sensor_msgs/CameraInfo)
/// Publishes:   /landmarks/detections (every processed image, even when empty)
///              /landmarks/debug/image (annotated, with publish_debug_image)
///
/// Tag sizes come from the landmark map (landmarks_file) so a mix of sizes works; tags
/// not in the map use default_tag_size_m and are reported with known = false.

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cv_bridge/cv_bridge.hpp>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <perseus_interfaces/msg/tag_detection_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

#include "perseus_landmarks/landmark_map.hpp"
#include "perseus_landmarks/tag_detection.hpp"

namespace perseus_landmarks
{
    class TagDetectorNode : public rclcpp::Node
    {
    public:
        explicit TagDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
            : rclcpp::Node("tag_detector", options)
        {
            const auto landmarks_file = declare_parameter<std::string>("landmarks_file", "");
            params_.family = declare_parameter<std::string>("tag_family", "");
            default_size_m_ = declare_parameter<double>("default_tag_size_m", 0.0);
            params_.min_marker_perimeter_rate = declare_parameter<double>("min_marker_perimeter_rate", 0.03);
            params_.corner_refinement = declare_parameter<std::string>("corner_refinement", "subpix");
            rectified_ = declare_parameter<bool>("rectified", true);
            max_rate_hz_ = declare_parameter<double>("max_rate_hz", 5.0);
            publish_debug_image_ = declare_parameter<bool>("publish_debug_image", false);

            if (!landmarks_file.empty())
            {
                map_ = load_landmark_map(landmarks_file);
                if (params_.family.empty())
                {
                    params_.family = map_->family;
                }
                if (default_size_m_ <= 0.0)
                {
                    default_size_m_ = map_->default_tag_size_m;
                }
            }
            if (params_.family.empty())
            {
                params_.family = "36h11";
            }
            if (default_size_m_ <= 0.0)
            {
                default_size_m_ = 0.16;
            }
            if (!family_supported(params_.family))
            {
                throw std::invalid_argument("unsupported tag_family '" + params_.family + "'");
            }

            info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
                declare_parameter<std::string>("camera_info_topic", "/oak/right_rect/camera_info"), rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
                { on_camera_info(msg); });
            image_sub_ = create_subscription<sensor_msgs::msg::Image>(
                declare_parameter<std::string>("image_topic", "/oak/right_rect/image_raw"), rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg)
                { on_image(msg); });
            detections_pub_ = create_publisher<perseus_interfaces::msg::TagDetectionArray>("/landmarks/detections", 10);
            if (publish_debug_image_)
            {
                debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/landmarks/debug/image", rclcpp::SensorDataQoS());
            }
            RCLCPP_INFO(get_logger(), "tag_detector: tag%s, default size %.3f m, %zu known tags", params_.family.c_str(),
                        default_size_m_, map_ ? map_->tags.size() : 0U);
        }

    private:
        void on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
        {
            CameraModel camera;
            if (rectified_ && msg->p[0] > 0.0)
            {
                camera = {msg->p[0], msg->p[5], msg->p[2], msg->p[6], {}};
            }
            else
            {
                camera = {msg->k[0], msg->k[4], msg->k[2], msg->k[5], {msg->d.begin(), msg->d.end()}};
            }
            if (!camera_)
            {
                RCLCPP_INFO(get_logger(), "Camera: fx %.1f fy %.1f cx %.1f cy %.1f (%ux%u)", camera.fx, camera.fy, camera.cx,
                            camera.cy, msg->width, msg->height);
            }
            camera_ = camera;
        }

        void on_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
        {
            if (!camera_)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Image received but no camera_info yet");
                return;
            }
            const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
            if (max_rate_hz_ > 0.0 && last_stamp_ && (stamp - *last_stamp_).seconds() >= 0.0 &&
                (stamp - *last_stamp_).seconds() < 0.9 / max_rate_hz_)
            {
                return;
            }
            last_stamp_ = stamp;

            cv_bridge::CvImageConstPtr gray;
            try
            {
                gray = cv_bridge::toCvShare(msg, "mono8");
            }
            catch (const cv_bridge::Exception& error)
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Cannot convert '%s' to mono8: %s",
                                      msg->encoding.c_str(), error.what());
                return;
            }

            const auto tags = detect_tags(gray->image, *camera_, params_, [this](int id)
                                          { return map_ ? map_->size_of(id) : default_size_m_; });

            perseus_interfaces::msg::TagDetectionArray out;
            out.header = msg->header;
            for (const DetectedTag& tag : tags)
            {
                perseus_interfaces::msg::TagDetection detection;
                detection.id = tag.id;
                detection.family = params_.family;
                detection.size_m = tag.size_m;
                detection.pose.position.x = tag.tvec[0];
                detection.pose.position.y = tag.tvec[1];
                detection.pose.position.z = tag.tvec[2];
                cv::Matx33d r;
                cv::Rodrigues(tag.rvec, r);
                const tf2::Matrix3x3 basis(r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2, 1), r(2, 2));
                tf2::Quaternion q;
                basis.getRotation(q);
                detection.pose.orientation.x = q.x();
                detection.pose.orientation.y = q.y();
                detection.pose.orientation.z = q.z();
                detection.pose.orientation.w = q.w();
                detection.reprojection_error_px = tag.reprojection_error_px;
                detection.ambiguity_ratio = tag.ambiguity_ratio;
                detection.distance_m = tag.distance_m;
                detection.view_angle_rad = tag.view_angle_rad;
                detection.known = map_ && map_->tags.count(tag.id) > 0;
                out.detections.push_back(detection);
            }
            detections_pub_->publish(out);

            if (publish_debug_image_)
            {
                publish_debug(gray->image, tags, msg->header);
            }
        }

        void publish_debug(const cv::Mat& gray, const std::vector<DetectedTag>& tags, const std_msgs::msg::Header& header)
        {
            cv::Mat colour;
            cv::cvtColor(gray, colour, cv::COLOR_GRAY2BGR);
            const cv::Matx33d k(camera_->fx, 0, camera_->cx, 0, camera_->fy, camera_->cy, 0, 0, 1);
            for (const DetectedTag& tag : tags)
            {
                std::vector<cv::Point> polygon(tag.corners.begin(), tag.corners.end());
                cv::polylines(colour, polygon, true, cv::Scalar(0, 255, 0), 2);
                cv::drawFrameAxes(colour, k, cv::noArray(), tag.rvec, tag.tvec, static_cast<float>(tag.size_m / 2.0));
                cv::putText(colour, std::to_string(tag.id) + " " + cv::format("%.2fm", tag.distance_m), polygon[0],
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
            }
            debug_pub_->publish(*cv_bridge::CvImage(header, "bgr8", colour).toImageMsg());
        }

        DetectorParams params_;
        double default_size_m_{0.16};
        bool rectified_{true};
        double max_rate_hz_{5.0};
        bool publish_debug_image_{false};
        std::optional<LandmarkMap> map_;
        std::optional<CameraModel> camera_;
        std::optional<rclcpp::Time> last_stamp_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<perseus_interfaces::msg::TagDetectionArray>::SharedPtr detections_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
    };

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<perseus_landmarks::TagDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
