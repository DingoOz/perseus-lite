#include "perseus_vision/cube_color_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <numeric>

namespace perseus_vision
{
    namespace
    {
        constexpr std::array<CubeColor, 4> kAllColors = {
            CubeColor::BLUE, CubeColor::GREEN, CubeColor::RED, CubeColor::WHITE};

        sensor_msgs::msg::RegionOfInterest roi_from_corners(
            const std::array<cv::Point2f, 4>& corner_points)
        {
            sensor_msgs::msg::RegionOfInterest roi;
            double min_x = corner_points[0].x;
            double max_x = corner_points[0].x;
            double min_y = corner_points[0].y;
            double max_y = corner_points[0].y;

            for (const auto& pt : corner_points)
            {
                min_x = std::min(min_x, static_cast<double>(pt.x));
                max_x = std::max(max_x, static_cast<double>(pt.x));
                min_y = std::min(min_y, static_cast<double>(pt.y));
                max_y = std::max(max_y, static_cast<double>(pt.y));
            }

            roi.x_offset = static_cast<uint32_t>(std::max(0.0, std::floor(min_x)));
            roi.y_offset = static_cast<uint32_t>(std::max(0.0, std::floor(min_y)));
            roi.width = static_cast<uint32_t>(std::max(0.0, std::ceil(max_x - min_x)));
            roi.height = static_cast<uint32_t>(std::max(0.0, std::ceil(max_y - min_y)));
            roi.do_rectify = false;
            return roi;
        }

        HsvRange parse_hsv_range(const std::vector<int64_t>& v, const HsvRange& fallback)
        {
            // Accept either 6 values [h_low,h_high,s_low,s_high,v_low,v_high] or
            // 8 values appending [h_low2,h_high2] for hue-wrap colours.
            if (v.size() != 6 && v.size() != 8)
            {
                return fallback;
            }
            HsvRange r;
            r.h_low = static_cast<int>(v[0]);
            r.h_high = static_cast<int>(v[1]);
            r.s_low = static_cast<int>(v[2]);
            r.s_high = static_cast<int>(v[3]);
            r.v_low = static_cast<int>(v[4]);
            r.v_high = static_cast<int>(v[5]);
            if (v.size() == 8)
            {
                r.h_low2 = static_cast<int>(v[6]);
                r.h_high2 = static_cast<int>(v[7]);
            }
            return r;
        }
    }  // namespace

    const char* CubeColorDetector::color_name(CubeColor color)
    {
        switch (color)
        {
        case CubeColor::BLUE:
            return "blue";
        case CubeColor::GREEN:
            return "green";
        case CubeColor::RED:
            return "red";
        case CubeColor::WHITE:
            return "white";
        }
        return "unknown";
    }

    cv::Scalar CubeColorDetector::color_bgr(CubeColor color)
    {
        // BGR for OpenCV drawing.
        switch (color)
        {
        case CubeColor::BLUE:
            return cv::Scalar(255, 80, 0);
        case CubeColor::GREEN:
            return cv::Scalar(0, 220, 0);
        case CubeColor::RED:
            return cv::Scalar(0, 0, 255);
        case CubeColor::WHITE:
            return cv::Scalar(240, 240, 240);
        }
        return cv::Scalar(200, 200, 200);
    }

    CubeColorDetector::CubeColorDetector()
        : Node("cube_color_detector")
    {
        // -------------------------
        // Geometry / detector params
        // -------------------------
        cube_size_ = this->declare_parameter<double>("cube_size", 0.10);
        axis_length_ = this->declare_parameter<double>("axis_length", 0.05);
        min_contour_area_ = this->declare_parameter<double>("min_contour_area", 300.0);
        max_aspect_ratio_ = this->declare_parameter<double>("max_aspect_ratio", 1.6);
        min_face_solidity_ = this->declare_parameter<double>("min_face_solidity", 0.85);

        // -------------------------
        // Frames / topics
        // -------------------------
        camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera_optical_frame");
        tf_output_frame_ = this->declare_parameter<std::string>("tf_output_frame", "odom");
        input_img_ = this->declare_parameter<std::string>("input_img", "/image_raw");
        output_img_ = this->declare_parameter<std::string>("output_img", "/perseus_vision/cube_color/image");
        output_topic_ = this->declare_parameter<std::string>(
            "output_topic", "/perseus_vision/cube_color/detections");
        output_markers_topic_ = this->declare_parameter<std::string>(
            "output_markers_topic", "/perseus_vision/cube_color/markers");
        camera_info_topic_ = this->declare_parameter<std::string>("camera_info_topic", "/camera_info");

        should_publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
        should_publish_img_ = this->declare_parameter<bool>("publish_img", true);
        is_compressed_io_ = this->declare_parameter<bool>("compressed_io", false);
        should_use_camera_info_ = this->declare_parameter<bool>("use_camera_info", true);
        should_publish_output_ = this->declare_parameter<bool>("publish_output", true);

        // -------------------------
        // Per-colour HSV thresholds (OpenCV scale: H in [0,180], S/V in [0,255])
        // -------------------------
        // Defaults are conservative starting points for indoor light; tune via YAML.
        const HsvRange blue_default{100, 130, 90, 255, 60, 255, -1, -1};
        const HsvRange green_default{40, 85, 70, 255, 50, 255, -1, -1};
        const HsvRange red_default{0, 10, 110, 255, 70, 255, 170, 180};
        const HsvRange white_default{0, 180, 0, 45, 200, 255, -1, -1};

        hsv_ranges_[static_cast<size_t>(CubeColor::BLUE)] = parse_hsv_range(
            this->declare_parameter<std::vector<int64_t>>(
                "hsv_blue",
                {blue_default.h_low, blue_default.h_high, blue_default.s_low,
                 blue_default.s_high, blue_default.v_low, blue_default.v_high}),
            blue_default);
        hsv_ranges_[static_cast<size_t>(CubeColor::GREEN)] = parse_hsv_range(
            this->declare_parameter<std::vector<int64_t>>(
                "hsv_green",
                {green_default.h_low, green_default.h_high, green_default.s_low,
                 green_default.s_high, green_default.v_low, green_default.v_high}),
            green_default);
        hsv_ranges_[static_cast<size_t>(CubeColor::RED)] = parse_hsv_range(
            this->declare_parameter<std::vector<int64_t>>(
                "hsv_red",
                {red_default.h_low, red_default.h_high, red_default.s_low,
                 red_default.s_high, red_default.v_low, red_default.v_high,
                 red_default.h_low2, red_default.h_high2}),
            red_default);
        hsv_ranges_[static_cast<size_t>(CubeColor::WHITE)] = parse_hsv_range(
            this->declare_parameter<std::vector<int64_t>>(
                "hsv_white",
                {white_default.h_low, white_default.h_high, white_default.s_low,
                 white_default.s_high, white_default.v_low, white_default.v_high}),
            white_default);

        // Optional whitelist: e.g. enabled_colors: ["red","blue"] to drop green/white.
        std::vector<std::string> enabled_color_names = this->declare_parameter<std::vector<std::string>>(
            "enabled_colors", std::vector<std::string>{"red", "blue", "green", "white"});
        for (const auto& name : enabled_color_names)
        {
            for (CubeColor c : kAllColors)
            {
                if (name == color_name(c))
                {
                    enabled_color_codes_.insert(static_cast<int32_t>(c));
                }
            }
        }
        if (enabled_color_codes_.empty())
        {
            RCLCPP_WARN(this->get_logger(),
                        "enabled_colors parameter resolved to empty set — all colours will be skipped.");
        }

        // -------------------------
        // Camera intrinsics fallback
        // -------------------------
        std::vector<double> camera_matrix_param = this->declare_parameter<std::vector<double>>(
            "camera_matrix", {530.4, 0.0, 320.0, 0.0, 530.4, 240.0, 0.0, 0.0, 1.0});
        std::vector<double> dist_coeffs_param = this->declare_parameter<std::vector<double>>(
            "distortion_coefficients", {0.0, 0.0, 0.0, 0.0, 0.0});

        if (camera_matrix_param.size() != 9)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "camera_matrix must have 9 elements, got %zu. Using defaults.",
                         camera_matrix_param.size());
            camera_matrix_param = {530.4, 0.0, 320.0, 0.0, 530.4, 240.0, 0.0, 0.0, 1.0};
        }

        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        for (size_t i = 0; i < 9; ++i)
        {
            camera_matrix_.at<double>(i / 3, i % 3) = camera_matrix_param[i];
        }
        dist_coeffs_ = cv::Mat(dist_coeffs_param.size(), 1, CV_64F);
        for (size_t i = 0; i < dist_coeffs_param.size(); ++i)
        {
            dist_coeffs_.at<double>(i, 0) = dist_coeffs_param[i];
        }

        // -------------------------
        // TF
        // -------------------------
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // -------------------------
        // Image IO
        // -------------------------
        if (is_compressed_io_)
        {
            compressed_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                input_img_ + "/compressed", kQosDepth,
                std::bind(&CubeColorDetector::compressed_image_callback, this, std::placeholders::_1));
            compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
                output_img_ + "/compressed", kQosDepth);
        }
        else
        {
            sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                input_img_, kQosDepth,
                std::bind(&CubeColorDetector::image_callback, this, std::placeholders::_1));
            pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_img_, kQosDepth);
        }

        if (should_use_camera_info_)
        {
            camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_info_topic_, rclcpp::SensorDataQoS(),
                std::bind(&CubeColorDetector::camera_info_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribing to camera_info on %s",
                        camera_info_topic_.c_str());
        }

        service_ = this->create_service<DetectObjects>(
            "detect_cubes",
            std::bind(&CubeColorDetector::handle_request, this,
                      std::placeholders::_1, std::placeholders::_2));

        if (should_publish_output_)
        {
            detection_pub_ = this->create_publisher<perseus_interfaces::msg::ObjectDetections>(
                output_topic_, kQosDepth);
        }

        marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            output_markers_topic_, kQosDepth);

        RCLCPP_INFO(this->get_logger(),
                    "CubeColorDetector started. cube_size=%.3f m, enabled colours: %zu.",
                    cube_size_, enabled_color_codes_.size());
    }

    void CubeColorDetector::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        try
        {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        process_image(frame, msg->header);

        if (should_publish_img_)
        {
            std::lock_guard<std::mutex> lock(detections_mutex_);
            auto processed_msg = cv_bridge::CvImage(msg->header, "bgr8", latest_frame_).toImageMsg();
            pub_->publish(*processed_msg);
        }
    }

    void CubeColorDetector::compressed_image_callback(
        const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        cv::Mat frame;
        try
        {
            frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            if (frame.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image");
                return;
            }
        }
        catch (cv::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
            return;
        }

        process_image(frame, msg->header);

        if (should_publish_img_)
        {
            sensor_msgs::msg::CompressedImage compressed_msg;
            compressed_msg.header = msg->header;
            compressed_msg.format = "jpeg";
            {
                std::lock_guard<std::mutex> lock(detections_mutex_);
                std::vector<uchar> buffer;
                std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
                cv::imencode(".jpg", latest_frame_, buffer, params);
                compressed_msg.data = buffer;
            }
            compressed_pub_->publish(compressed_msg);
        }
    }

    cv::Mat CubeColorDetector::build_color_mask(const cv::Mat& hsv, const HsvRange& range) const
    {
        cv::Mat mask;
        cv::inRange(hsv,
                    cv::Scalar(range.h_low, range.s_low, range.v_low),
                    cv::Scalar(range.h_high, range.s_high, range.v_high),
                    mask);
        if (range.h_low2 >= 0 && range.h_high2 >= 0)
        {
            cv::Mat mask2;
            cv::inRange(hsv,
                        cv::Scalar(range.h_low2, range.s_low, range.v_low),
                        cv::Scalar(range.h_high2, range.s_high, range.v_high),
                        mask2);
            cv::bitwise_or(mask, mask2, mask);
        }

        // Morphological cleanup: open removes salt noise, close fills small holes.
        const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        return mask;
    }

    std::array<cv::Point2f, 4> CubeColorDetector::order_quad_corners(
        const std::vector<cv::Point>& polygon)
    {
        // Image coords: y grows downward. TL has min(x+y), BR max(x+y);
        // TR has min(y-x), BL max(y-x).
        std::array<cv::Point2f, 4> ordered{};
        double min_sum = std::numeric_limits<double>::max();
        double max_sum = -std::numeric_limits<double>::max();
        double min_diff = std::numeric_limits<double>::max();
        double max_diff = -std::numeric_limits<double>::max();
        for (const auto& p : polygon)
        {
            double s = static_cast<double>(p.x) + static_cast<double>(p.y);
            double d = static_cast<double>(p.y) - static_cast<double>(p.x);
            if (s < min_sum)
            {
                min_sum = s;
                ordered[0] = cv::Point2f(p.x, p.y);
            }  // TL
            if (s > max_sum)
            {
                max_sum = s;
                ordered[2] = cv::Point2f(p.x, p.y);
            }  // BR
            if (d < min_diff)
            {
                min_diff = d;
                ordered[1] = cv::Point2f(p.x, p.y);
            }  // TR
            if (d > max_diff)
            {
                max_diff = d;
                ordered[3] = cv::Point2f(p.x, p.y);
            }  // BL
        }
        return ordered;
    }

    std::vector<std::array<cv::Point2f, 4>> CubeColorDetector::extract_face_quads(
        const cv::Mat& mask) const
    {
        std::vector<std::array<cv::Point2f, 4>> faces;
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours)
        {
            const double area = cv::contourArea(contour);
            if (area < min_contour_area_)
            {
                continue;
            }

            std::vector<cv::Point> hull;
            cv::convexHull(contour, hull);
            const double hull_area = cv::contourArea(hull);
            if (hull_area <= 0.0)
            {
                continue;
            }
            // Solidity rejects spiky/concave blobs (e.g. lighting reflections on
            // multiple cubes blurred together) before we pay for polygon fitting.
            const double solidity = area / hull_area;
            if (solidity < min_face_solidity_)
            {
                continue;
            }

            // Polygon-approximate the convex hull. Fitting against the hull
            // (instead of the raw contour) gives a more stable 4-vertex result
            // when the cube edge is slightly noisy.
            std::vector<cv::Point> approx;
            const double perimeter = cv::arcLength(hull, true);
            cv::approxPolyDP(hull, approx, 0.04 * perimeter, true);
            if (approx.size() != 4)
            {
                continue;
            }

            // Aspect-ratio filter: a cube face viewed obliquely is a quad but
            // its bounding rect should still be roughly square.
            const cv::RotatedRect rrect = cv::minAreaRect(contour);
            const float w = std::max(rrect.size.width, rrect.size.height);
            const float h = std::min(rrect.size.width, rrect.size.height);
            if (h <= 0.0f)
            {
                continue;
            }
            const float aspect = w / h;
            if (aspect > static_cast<float>(max_aspect_ratio_))
            {
                continue;
            }

            faces.push_back(order_quad_corners(approx));
        }
        return faces;
    }

    bool CubeColorDetector::transform_and_publish_face(
        const std_msgs::msg::Header& header,
        CubeColor color,
        int instance_index,
        const std::array<cv::Point2f, 4>& image_points,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs,
        cv::Mat& annotated_frame)
    {
        // Cube face object points (face on z=0, +Z normal toward camera).
        const float half = static_cast<float>(cube_size_) / 2.0f;
        const std::vector<cv::Point3f> obj_points = {
            cv::Point3f(-half, half, 0.0f),   // TL
            cv::Point3f(half, half, 0.0f),    // TR
            cv::Point3f(half, -half, 0.0f),   // BR
            cv::Point3f(-half, -half, 0.0f),  // BL
        };
        const std::vector<cv::Point2f> img_pts(image_points.begin(), image_points.end());

        cv::Vec3d rvec, tvec;
        if (!cv::solvePnP(obj_points, img_pts, camera_matrix, dist_coeffs,
                          rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE))
        {
            return false;
        }

        try
        {
            geometry_msgs::msg::PoseStamped pose_camera;
            pose_camera.header.stamp = header.stamp;
            pose_camera.header.frame_id = camera_frame_;
            pose_camera.pose.position.x = tvec[0];
            pose_camera.pose.position.y = tvec[1];
            pose_camera.pose.position.z = tvec[2];

            cv::Mat rotation_matrix;
            cv::Rodrigues(rvec, rotation_matrix);
            tf2::Matrix3x3 tf2_rot(
                rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
                rotation_matrix.at<double>(0, 2),
                rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1),
                rotation_matrix.at<double>(1, 2),
                rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
                rotation_matrix.at<double>(2, 2));
            tf2::Quaternion quat;
            tf2_rot.getRotation(quat);
            pose_camera.pose.orientation.x = quat.x();
            pose_camera.pose.orientation.y = quat.y();
            pose_camera.pose.orientation.z = quat.z();
            pose_camera.pose.orientation.w = quat.w();

            // v4l2_camera stamps frames with kernel time while EKF/RSP stamp TFs
            // with ROS time, so per-stamp lookups always trip "extrapolation
            // into the future" — use latest-available instead. (Same workaround
            // as aruco_detector; see commit history for the underlying issue.)
            geometry_msgs::msg::PoseStamped pose_for_tf = pose_camera;
            pose_for_tf.header.stamp = rclcpp::Time(0);
            geometry_msgs::msg::PoseStamped pose_out;
            tf_buffer_->transform(pose_for_tf, pose_out, tf_output_frame_);
            pose_out.header.stamp = header.stamp;

            const int32_t color_code = static_cast<int32_t>(color);
            {
                std::lock_guard<std::mutex> lock(detections_mutex_);
                latest_ids_.push_back(color_code);
                latest_poses_.push_back(pose_out.pose);
                latest_regions_of_interest_.push_back(roi_from_corners(image_points));
            }

            if (should_publish_tf_)
            {
                geometry_msgs::msg::TransformStamped transform;
                transform.header.stamp = header.stamp;
                transform.header.frame_id = tf_output_frame_;
                transform.child_frame_id =
                    std::string("cube_") + color_name(color) + "_" + std::to_string(instance_index);
                transform.transform.translation.x = pose_out.pose.position.x;
                transform.transform.translation.y = pose_out.pose.position.y;
                transform.transform.translation.z = pose_out.pose.position.z;
                transform.transform.rotation = pose_out.pose.orientation;
                tf_broadcaster_->sendTransform(transform);
            }

            // Annotate the debug image: draw the quad and a frame-axis triad.
            const cv::Scalar bgr = color_bgr(color);
            for (size_t i = 0; i < 4; ++i)
            {
                cv::line(annotated_frame, image_points[i], image_points[(i + 1) % 4],
                         bgr, 2);
            }
            cv::drawFrameAxes(annotated_frame, camera_matrix, dist_coeffs, rvec, tvec,
                              static_cast<float>(axis_length_));
            char label[64];
            std::snprintf(label, sizeof(label), "%s #%d %.2fm",
                          color_name(color), instance_index, tvec[2]);
            cv::putText(annotated_frame, label,
                        cv::Point(static_cast<int>(image_points[0].x),
                                  static_cast<int>(image_points[0].y) - 6),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, bgr, 1, cv::LINE_AA);

            RCLCPP_DEBUG(this->get_logger(),
                         "Cube %s #%d in %s: x=%.2f y=%.2f z=%.2f",
                         color_name(color), instance_index, tf_output_frame_.c_str(),
                         pose_out.pose.position.x, pose_out.pose.position.y,
                         pose_out.pose.position.z);
            return true;
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform cube pose: %s", ex.what());
            return false;
        }
    }

    void CubeColorDetector::process_image(const cv::Mat& frame,
                                          const std_msgs::msg::Header& header)
    {
        cv::Mat annotated_frame = frame.clone();

        {
            std::lock_guard<std::mutex> lock(detections_mutex_);
            latest_ids_.clear();
            latest_poses_.clear();
            latest_regions_of_interest_.clear();
            latest_timestamp_ = header.stamp;
        }

        cv::Mat local_camera_matrix, local_dist_coeffs;
        bool has_camera_matrix = false;
        {
            std::lock_guard<std::mutex> lock(camera_matrix_mutex_);
            if (!camera_matrix_.empty())
            {
                local_camera_matrix = camera_matrix_.clone();
                local_dist_coeffs = dist_coeffs_.clone();
                has_camera_matrix = true;
            }
            else
            {
                RCLCPP_WARN_ONCE(this->get_logger(),
                                 "Camera matrix not initialised; skipping pose estimation.");
            }
        }

        if (has_camera_matrix && !enabled_color_codes_.empty())
        {
            cv::Mat hsv;
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

            for (CubeColor color : kAllColors)
            {
                if (enabled_color_codes_.count(static_cast<int32_t>(color)) == 0)
                {
                    continue;
                }
                const HsvRange& range = hsv_ranges_[static_cast<size_t>(color)];
                cv::Mat mask = build_color_mask(hsv, range);
                std::vector<std::array<cv::Point2f, 4>> faces = extract_face_quads(mask);

                int instance_index = 0;
                for (const auto& face : faces)
                {
                    if (transform_and_publish_face(header, color, instance_index,
                                                   face, local_camera_matrix,
                                                   local_dist_coeffs, annotated_frame))
                    {
                        ++instance_index;
                    }
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(detections_mutex_);
            latest_frame_ = std::move(annotated_frame);
        }

        // Per-colour instance counters for unique RViz marker IDs.
        std::array<int, kNumColors> ns_counters{};
        ns_counters.fill(0);

        visualization_msgs::msg::MarkerArray marker_array;
        {
            std::lock_guard<std::mutex> lock(detections_mutex_);
            for (size_t i = 0; i < latest_ids_.size(); ++i)
            {
                const int32_t code = latest_ids_[i];
                if (code < 0 || code >= static_cast<int32_t>(kNumColors))
                {
                    continue;
                }
                const auto color = static_cast<CubeColor>(code);
                const std::string ns = std::string("cube_color_") + color_name(color);
                const int marker_id = ns_counters[code]++;

                visualization_msgs::msg::Marker cube;
                cube.header.frame_id = tf_output_frame_;
                cube.header.stamp = header.stamp;
                cube.ns = ns;
                cube.id = marker_id;
                cube.type = visualization_msgs::msg::Marker::CUBE;
                cube.action = visualization_msgs::msg::Marker::ADD;
                cube.pose = latest_poses_[i];
                cube.scale.x = cube.scale.y = cube.scale.z = cube_size_;
                // Convert BGR Scalar back to 0..1 RGB for RViz.
                const cv::Scalar bgr = color_bgr(color);
                cube.color.b = static_cast<float>(bgr[0] / 255.0);
                cube.color.g = static_cast<float>(bgr[1] / 255.0);
                cube.color.r = static_cast<float>(bgr[2] / 255.0);
                cube.color.a = 0.85f;
                cube.lifetime = rclcpp::Duration(2, 0);
                marker_array.markers.push_back(cube);

                visualization_msgs::msg::Marker text;
                text.header.frame_id = tf_output_frame_;
                text.header.stamp = header.stamp;
                text.ns = ns + "_label";
                text.id = marker_id;
                text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                text.action = visualization_msgs::msg::Marker::ADD;
                text.pose.position = latest_poses_[i].position;
                text.pose.position.z += cube_size_;
                text.pose.orientation.w = 1.0;
                text.scale.z = 0.15;
                text.color.r = text.color.g = text.color.b = 1.0f;
                text.color.a = 1.0f;
                text.text = std::string(color_name(color)) + " #" + std::to_string(marker_id);
                text.lifetime = rclcpp::Duration(2, 0);
                marker_array.markers.push_back(text);
            }
        }
        marker_array_pub_->publish(marker_array);

        if (should_publish_output_ && detection_pub_)
        {
            perseus_interfaces::msg::ObjectDetections detection_msg;
            detection_msg.stamp = header.stamp;
            detection_msg.frame_id = tf_output_frame_;
            {
                std::lock_guard<std::mutex> lock(detections_mutex_);
                detection_msg.ids = latest_ids_;
                detection_msg.poses = latest_poses_;
                detection_msg.regions_of_interest = latest_regions_of_interest_;
            }
            detection_pub_->publish(detection_msg);
        }
    }

    void CubeColorDetector::handle_request(
        const std::shared_ptr<DetectObjects::Request> request,
        std::shared_ptr<DetectObjects::Response> response)
    {
        cv::Mat frame_to_save;
        std::vector<int32_t> ids_copy;
        size_t detection_count = 0;
        {
            std::lock_guard<std::mutex> lock(detections_mutex_);
            response->stamp = latest_timestamp_;
            response->frame_id = tf_output_frame_;
            response->ids = latest_ids_;
            response->poses = latest_poses_;
            response->regions_of_interest = latest_regions_of_interest_;
            response->message = latest_ids_.empty()
                                    ? "No cube detections are currently cached."
                                    : "Returned cached cube detections.";
            detection_count = latest_ids_.size();
            if (request->capture_image)
            {
                frame_to_save = latest_frame_.clone();
                ids_copy = latest_ids_;
            }
        }

        if (request->capture_image)
        {
            if (frame_to_save.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Capture requested but no frame available.");
            }
            else
            {
                try
                {
                    std::error_code ec;
                    std::filesystem::create_directories(request->img_save_path, ec);
                    if (ec)
                    {
                        RCLCPP_WARN(this->get_logger(), "Failed to create directory: %s",
                                    request->img_save_path.c_str());
                    }
                    else
                    {
                        const auto now = std::chrono::system_clock::now();
                        const auto epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                  now.time_since_epoch())
                                                  .count();
                        std::string ids_str;
                        for (size_t i = 0; i < ids_copy.size(); ++i)
                        {
                            if (i > 0)
                                ids_str += "_";
                            const int32_t code = ids_copy[i];
                            ids_str += (code >= 0 && code < static_cast<int32_t>(kNumColors))
                                           ? color_name(static_cast<CubeColor>(code))
                                           : "unknown";
                        }
                        if (ids_str.empty())
                            ids_str = "no_cubes";

                        const std::string filename = request->img_save_path + "/cube_color_" +
                                                     ids_str + "_" + std::to_string(epoch_ms) + ".png";
                        if (cv::imwrite(filename, frame_to_save))
                        {
                            RCLCPP_INFO(this->get_logger(), "Captured image saved to: %s",
                                        filename.c_str());
                        }
                        else
                        {
                            RCLCPP_ERROR(this->get_logger(), "Failed to write image to: %s",
                                         filename.c_str());
                        }
                    }
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Exception during image capture: %s", e.what());
                }
            }
        }

        if (detection_count > 0)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Service request: returning %zu cube detections.",
                        detection_count);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                        "Service request: no cube detections available.");
        }
    }

    void CubeColorDetector::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        // Reject all-zero K (uncalibrated v4l2_camera default) — overwriting
        // the YAML fallback with zeroes makes solvePnP collapse every cube to
        // the camera origin. Same defensive check as aruco_detector.
        if (msg->k[0] <= 0.0 || msg->k[4] <= 0.0)
        {
            RCLCPP_WARN_ONCE(this->get_logger(),
                             "Ignoring %s: fx/fy are zero (uncalibrated). "
                             "Keeping YAML fallback camera_matrix.",
                             camera_info_topic_.c_str());
            return;
        }

        std::lock_guard<std::mutex> lock(camera_matrix_mutex_);
        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        for (size_t i = 0; i < 9; ++i)
        {
            camera_matrix_.at<double>(i / 3, i % 3) = msg->k[i];
        }
        dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F);
        for (size_t i = 0; i < msg->d.size(); ++i)
        {
            dist_coeffs_.at<double>(i, 0) = msg->d[i];
        }
    }

}  // namespace perseus_vision
