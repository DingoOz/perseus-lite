/// @file landmark_localiser_node.cpp
/// @brief landmark_localiser: tag sightings in, the site -> map transform out.
///
/// slam_toolbox owns map -> odom and the EKF owns odom -> base_link (see CLAUDE.md's TF
/// contract), so tag localisation never touches either. It estimates where the SLAM map
/// sits in the site's own coordinates (the frame the landmark map, destinations and
/// operators use) and publishes that as a parent frame: site -> map. A Nav2 goal given
/// in "site" is transformed through it automatically.
///
/// Subscribes: /landmarks/detections, odom_topic (yaw-rate gate)
/// Publishes:  TF site -> map (once calibrated), /landmarks/localisation (status),
///             /landmarks/markers (tags and destinations, for RViz)
/// Services:   ~/calibrate (perseus_interfaces/srv/CalibrateFromLandmarks), ~/reset

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <deque>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <perseus_interfaces/msg/landmark_localisation.hpp>
#include <perseus_interfaces/msg/tag_detection_array.hpp>
#include <perseus_interfaces/srv/calibrate_from_landmarks.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "perseus_landmarks/landmark_map.hpp"
#include "perseus_landmarks/site_estimator.hpp"

namespace perseus_landmarks
{
    using CalibrateSrv = perseus_interfaces::srv::CalibrateFromLandmarks;

    class LandmarkLocaliser : public rclcpp::Node
    {
    public:
        explicit LandmarkLocaliser(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
            : rclcpp::Node("landmark_localiser", options)
        {
            const auto landmarks_file = declare_parameter<std::string>("landmarks_file", "");
            if (landmarks_file.empty())
            {
                throw std::invalid_argument("landmarks_file is required");
            }
            landmarks_ = load_landmark_map(landmarks_file);
            map_frame_ = declare_parameter<std::string>("map_frame", "map");
            max_range_m_ = declare_parameter<double>("max_range_m", 4.0);
            max_view_angle_rad_ = declare_parameter<double>("max_view_angle_deg", 60.0) * M_PI / 180.0;
            max_reprojection_px_ = declare_parameter<double>("max_reprojection_px", 2.0);
            min_ambiguity_ratio_ = declare_parameter<double>("min_ambiguity_ratio", 2.0);
            max_yaw_rate_ = declare_parameter<double>("max_yaw_rate_rad_s", 0.3);
            transform_wait_s_ = declare_parameter<double>("transform_wait_s", 0.5);
            calibrate_timeout_s_ = declare_parameter<double>("calibrate_timeout_s", 5.0);
            calibrate_min_detections_ = static_cast<int>(declare_parameter<int>("calibrate_min_detections", 10));

            params_.window_s = declare_parameter<double>("estimator.window_s", params_.window_s);
            params_.min_observations = static_cast<int>(declare_parameter<int>("estimator.min_observations", params_.min_observations));
            params_.max_residual_m = declare_parameter<double>("estimator.max_residual_m", params_.max_residual_m);
            params_.min_tag_separation_m = declare_parameter<double>("estimator.min_tag_separation_m", params_.min_tag_separation_m);
            params_.blend_alpha = declare_parameter<double>("estimator.blend_alpha", params_.blend_alpha);
            params_.max_jump_m = declare_parameter<double>("estimator.max_jump_m", params_.max_jump_m);
            params_.max_jump_yaw_rad = declare_parameter<double>("estimator.max_jump_yaw_deg", params_.max_jump_yaw_rad * 180.0 / M_PI) * M_PI / 180.0;
            params_.jump_confirm_s = declare_parameter<double>("estimator.jump_confirm_s", params_.jump_confirm_s);
            estimator_ = std::make_unique<SiteEstimator>(landmarks_, params_);

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            detections_sub_ = create_subscription<perseus_interfaces::msg::TagDetectionArray>(
                "/landmarks/detections", 10, [this](const perseus_interfaces::msg::TagDetectionArray::ConstSharedPtr& msg)
                { on_detections(msg); });
            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                declare_parameter<std::string>("odom_topic", "/odometry/filtered"), 10,
                [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
                { yaw_rate_ = msg->twist.twist.angular.z; });
            status_pub_ = create_publisher<perseus_interfaces::msg::LandmarkLocalisation>("/landmarks/localisation", 10);
            markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/landmarks/markers",
                                                                                  rclcpp::QoS(1).transient_local());

            calibrate_srv_ = create_service<CalibrateSrv>(
                // Deferred-response signature (header + request, no response): the
                // answer is sent later from finish_calibration(), once enough sightings
                // have arrived, without blocking the executor that delivers them.
                "~/calibrate", [this](const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<CalibrateSrv::Request> request)
                { start_calibration(header, *request); });
            reset_srv_ = create_service<std_srvs::srv::Trigger>(
                "~/reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
                {
                    estimator_->reset();
                    response->success = true;
                    response->message = "site -> map estimate cleared; not publishing until the next fix";
                    RCLCPP_INFO(get_logger(), "%s", response->message.c_str()); });

            const double tf_rate_hz = std::max(1.0, declare_parameter<double>("tf_rate_hz", 20.0));
            tf_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / tf_rate_hz), [this]()
                                          { on_tf_timer(); });
            status_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]()
                                              { publish_status(); });
            publish_markers();
            RCLCPP_INFO(get_logger(), "landmark_localiser: %zu tags, %zu destinations; publishing %s -> %s once calibrated",
                        landmarks_.tags.size(), landmarks_.destinations.size(), landmarks_.site_frame.c_str(),
                        map_frame_.c_str());
        }

    private:
        struct Calibration
        {
            std::shared_ptr<rmw_request_id_t> header;
            std::unique_ptr<SiteEstimator> estimator;
            rclcpp::Time deadline;
            int min_detections{0};
        };

        // ---- detections --------------------------------------------------------------

        void on_detections(const perseus_interfaces::msg::TagDetectionArray::ConstSharedPtr& msg)
        {
            bool any_known = false;
            for (const auto& detection : msg->detections)
            {
                any_known |= landmarks_.tags.count(detection.id) > 0;
            }
            if (!any_known)
            {
                return;
            }
            pending_.push_back(msg);
            while (pending_.size() > 30)
            {
                pending_.pop_front();
            }
            process_pending();
        }

        void process_pending()
        {
            const rclcpp::Time now_time = now();
            while (!pending_.empty())
            {
                const auto& msg = pending_.front();
                const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
                if (!tf_buffer_->canTransform(map_frame_, msg->header.frame_id, stamp))
                {
                    if ((now_time - stamp).seconds() < transform_wait_s_)
                    {
                        return;  // TF for this stamp usually lands a moment later
                    }
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No transform %s <- %s at detection stamp; dropping",
                                         map_frame_.c_str(), msg->header.frame_id.c_str());
                    pending_.pop_front();
                    continue;
                }
                tf2::Transform map_from_camera;
                try
                {
                    tf2::fromMsg(tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id, stamp).transform, map_from_camera);
                }
                catch (const tf2::TransformException& error)
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
                    pending_.pop_front();
                    continue;
                }
                ingest(*msg, map_from_camera, stamp.seconds());
                pending_.pop_front();
            }
        }

        void ingest(const perseus_interfaces::msg::TagDetectionArray& msg, const tf2::Transform& map_from_camera, double stamp_s)
        {
            // A turning robot smears the image and stresses TF timing; only look while
            // (nearly) still. Stop-and-look is what the search behaviour does anyway.
            if (std::abs(yaw_rate_) > max_yaw_rate_)
            {
                ++rejected_;
                ++rejected_turning_;
                return;
            }
            for (const auto& detection : msg.detections)
            {
                if (landmarks_.tags.count(detection.id) == 0)
                {
                    continue;
                }
                // Count each reason, so "0 usable sightings" can say why.
                const bool too_far = detection.distance_m > max_range_m_;
                const bool too_oblique = detection.view_angle_rad > max_view_angle_rad_;
                const bool poor_fit = detection.reprojection_error_px > max_reprojection_px_;
                if (too_far || too_oblique || poor_fit)
                {
                    ++rejected_;
                    rejected_range_ += too_far ? 1 : 0;
                    rejected_angle_ += too_oblique ? 1 : 0;
                    rejected_fit_ += poor_fit ? 1 : 0;
                    continue;
                }
                tf2::Transform camera_from_tag;
                tf2::fromMsg(detection.pose, camera_from_tag);
                const tf2::Transform map_from_tag = map_from_camera * camera_from_tag;
                // Facing = the tag's normal (its z axis) projected onto the ground plane.
                const tf2::Vector3 normal = map_from_tag.getBasis().getColumn(2);
                TagObservation observation;
                observation.id = detection.id;
                observation.position = {map_from_tag.getOrigin().x(), map_from_tag.getOrigin().y()};
                observation.facing = std::atan2(normal.y(), normal.x());
                // Near, head-on sightings are worth more: depth error grows with distance
                // squared, orientation error with obliqueness.
                observation.weight = std::cos(detection.view_angle_rad) / (1.0 + detection.distance_m * detection.distance_m);
                // A head-on or distant tag fits two mirror-tilted poses: keep its
                // position, drop its facing (see TagDetection.msg).
                observation.facing_weight = detection.ambiguity_ratio >= min_ambiguity_ratio_ ? observation.weight : 0.0;
                observation.stamp_s = stamp_s;
                estimator_->add(observation);
                if (calibration_)
                {
                    calibration_->estimator->add(observation);
                }
            }
            if (calibration_)
            {
                update_calibration(stamp_s);
                return;
            }
            const UpdateResult result = estimator_->update(stamp_s);
            report(result);
        }

        void report(UpdateResult result)
        {
            switch (result)
            {
            case UpdateResult::ACCEPTED:
            case UpdateResult::ACCEPTED_JUMP:
            {
                ++accepted_;
                last_fix_time_ = now();
                const bool first = !was_calibrated_;
                was_calibrated_ = true;
                const Pose2D& e = *estimator_->estimate();
                if (first || result == UpdateResult::ACCEPTED_JUMP)
                {
                    RCLCPP_INFO(get_logger(), "%s %s -> %s: x %.3f y %.3f yaw %.1f deg (residual %.3f m, %zu tags)",
                                first ? "Calibrated" : "Re-anchored after a persistent jump:", landmarks_.site_frame.c_str(),
                                map_frame_.c_str(), e.x, e.y, e.yaw * 180.0 / M_PI, estimator_->last_fix()->residual_m,
                                estimator_->last_fix()->tags.size());
                }
                break;
            }
            case UpdateResult::REJECTED_RESIDUAL:
                ++rejected_;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "Tag fix rejected: residual above %.2f m (a tag moved, or a misdetection?)",
                                     params_.max_residual_m);
                break;
            case UpdateResult::REJECTED_JUMP:
                ++rejected_;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "Tag fix disagrees with the estimate by more than the jump limits; waiting %.1f s for it to persist",
                                     params_.jump_confirm_s);
                break;
            case UpdateResult::NOTHING_TO_DO:
                break;
            }
        }

        // ---- calibration (asynchronous service) -------------------------------------

        void start_calibration(const std::shared_ptr<rmw_request_id_t>& header, const CalibrateSrv::Request& request)
        {
            if (calibration_)
            {
                CalibrateSrv::Response busy;
                busy.success = false;
                busy.message = "a calibration is already running";
                calibrate_srv_->send_response(*header, busy);
                return;
            }
            EstimatorParams params = params_;
            params.window_s = 1e9;  // collect everything for the whole calibration
            params.min_observations = request.min_detections > 0 ? request.min_detections : calibrate_min_detections_;
            calibration_ = Calibration{header, std::make_unique<SiteEstimator>(landmarks_, params),
                                       now() + rclcpp::Duration::from_seconds(request.timeout_s > 0.0 ? request.timeout_s : calibrate_timeout_s_),
                                       params.min_observations};
            RCLCPP_INFO(get_logger(), "Calibrating: collecting %d tag sightings (keep the robot still)", params.min_observations);
        }

        void update_calibration(double stamp_s)
        {
            const UpdateResult result = calibration_->estimator->update(stamp_s);
            if (result == UpdateResult::ACCEPTED)
            {
                const SiteFix fix = *calibration_->estimator->last_fix();
                estimator_->seed(fix);
                report(UpdateResult::ACCEPTED_JUMP);
                finish_calibration(true, "calibrated from " + std::to_string(fix.tags.size()) + " tag(s)", &fix);
            }
            else if (result == UpdateResult::REJECTED_RESIDUAL)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Calibration: sightings inconsistent (residual too high)");
            }
        }

        void finish_calibration(bool success, const std::string& message, const SiteFix* fix)
        {
            CalibrateSrv::Response response;
            response.success = success;
            response.message = message;
            if (fix)
            {
                response.x = fix->site_from_map.x;
                response.y = fix->site_from_map.y;
                response.yaw = fix->site_from_map.yaw;
                response.residual_m = fix->residual_m;
                response.tags_used.assign(fix->tags.begin(), fix->tags.end());
            }
            calibrate_srv_->send_response(*calibration_->header, response);
            if (!success)
            {
                RCLCPP_WARN(get_logger(), "Calibration failed: %s", message.c_str());
            }
            calibration_.reset();
        }

        /// @brief Rejections so far, by reason.
        std::string rejection_summary() const
        {
            return "rejected so far: " + std::to_string(rejected_range_) + " too far, " + std::to_string(rejected_angle_) +
                   " too oblique, " + std::to_string(rejected_fit_) + " poor corner fit, " + std::to_string(rejected_turning_) +
                   " while turning";
        }

        // ---- outputs -----------------------------------------------------------------

        void on_tf_timer()
        {
            process_pending();
            if (calibration_ && now() > calibration_->deadline)
            {
                const std::size_t usable = calibration_->estimator->window_size();
                std::string reason;
                if (static_cast<int>(usable) >= calibration_->min_detections)
                {
                    // Enough sightings but no fix: one tag whose orientation was always
                    // ambiguous (heading unobservable), or inconsistent tags.
                    reason = std::to_string(usable) +
                             " usable sightings but no consistent fix: with a single tag, every sighting's "
                             "orientation was ambiguous (view it more obliquely or closer, or put a second "
                             "tag in view)";
                }
                else
                {
                    reason = "timed out with " + std::to_string(usable) + " of " +
                             std::to_string(calibration_->min_detections) + " usable sightings; " + rejection_summary() +
                             " (is a known tag in view, within range and at most max_view_angle_deg off-normal?)";
                }
                finish_calibration(false, reason, nullptr);
            }
            const auto& estimate = estimator_->estimate();
            if (!estimate)
            {
                return;
            }
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = now();
            transform.header.frame_id = landmarks_.site_frame;
            transform.child_frame_id = map_frame_;
            transform.transform.translation.x = estimate->x;
            transform.transform.translation.y = estimate->y;
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, estimate->yaw);
            transform.transform.rotation = tf2::toMsg(q);
            tf_broadcaster_->sendTransform(transform);
        }

        void publish_status()
        {
            perseus_interfaces::msg::LandmarkLocalisation status;
            status.header.stamp = now();
            status.site_frame = landmarks_.site_frame;
            status.map_frame = map_frame_;
            status.accepted = accepted_;
            status.rejected = rejected_;
            const auto& estimate = estimator_->estimate();
            status.calibrated = estimate.has_value();
            if (estimate)
            {
                status.x = estimate->x;
                status.y = estimate->y;
                status.yaw = estimate->yaw;
            }
            if (const auto& fix = estimator_->last_fix())
            {
                status.residual_m = fix->residual_m;
                status.tags_used.assign(fix->tags.begin(), fix->tags.end());
            }
            if (last_fix_time_)
            {
                status.last_fix = *last_fix_time_;
            }
            status_pub_->publish(status);
        }

        void publish_markers()
        {
            visualization_msgs::msg::MarkerArray markers;
            int id = 0;
            auto make = [&](int type, const std::string& ns)
            {
                visualization_msgs::msg::Marker m;
                m.header.frame_id = landmarks_.site_frame;
                m.ns = ns;
                m.id = id++;
                m.type = type;
                m.action = visualization_msgs::msg::Marker::ADD;
                m.color.a = 1.0F;
                return m;
            };
            for (const auto& [tag_id, tag] : landmarks_.tags)
            {
                auto plate = make(visualization_msgs::msg::Marker::CUBE, "tags");
                plate.pose.position.x = tag.x;
                plate.pose.position.y = tag.y;
                plate.pose.position.z = tag.z;
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, tag.yaw);
                plate.pose.orientation = tf2::toMsg(q);
                plate.scale.x = 0.01;
                plate.scale.y = tag.size_m;
                plate.scale.z = tag.size_m;
                plate.color.r = 0.1F;
                plate.color.g = 0.1F;
                plate.color.b = 0.1F;
                markers.markers.push_back(plate);
                auto label = make(visualization_msgs::msg::Marker::TEXT_VIEW_FACING, "tags");
                label.pose = plate.pose;
                label.pose.position.z += tag.size_m;
                label.scale.z = 0.12;
                label.color.r = label.color.g = label.color.b = 1.0F;
                label.text = "tag " + std::to_string(tag_id);
                markers.markers.push_back(label);
            }
            for (const auto& destination : landmarks_.destinations)
            {
                auto arrow = make(visualization_msgs::msg::Marker::ARROW, "destinations");
                arrow.pose.position.x = destination.pose.x;
                arrow.pose.position.y = destination.pose.y;
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, destination.pose.yaw);
                arrow.pose.orientation = tf2::toMsg(q);
                arrow.scale.x = 0.4;
                arrow.scale.y = 0.08;
                arrow.scale.z = 0.08;
                arrow.color.r = 0.93F;
                arrow.color.g = 0.25F;
                arrow.color.b = 0.48F;
                markers.markers.push_back(arrow);
                auto label = make(visualization_msgs::msg::Marker::TEXT_VIEW_FACING, "destinations");
                label.pose = arrow.pose;
                label.pose.position.z = 0.25;
                label.scale.z = 0.14;
                label.color.r = label.color.g = label.color.b = 1.0F;
                label.text = destination.name;
                markers.markers.push_back(label);
            }
            markers_pub_->publish(markers);
        }

        LandmarkMap landmarks_;
        EstimatorParams params_;
        std::unique_ptr<SiteEstimator> estimator_;
        std::optional<Calibration> calibration_;
        std::string map_frame_;
        double max_range_m_{4.0};
        double max_view_angle_rad_{1.0};
        double max_reprojection_px_{2.0};
        double min_ambiguity_ratio_{2.0};
        double max_yaw_rate_{0.3};
        double transform_wait_s_{0.5};
        double calibrate_timeout_s_{5.0};
        int calibrate_min_detections_{10};
        double yaw_rate_{0.0};
        bool was_calibrated_{false};
        std::uint32_t accepted_{0};
        std::uint32_t rejected_{0};
        std::uint32_t rejected_range_{0};
        std::uint32_t rejected_angle_{0};
        std::uint32_t rejected_fit_{0};
        std::uint32_t rejected_turning_{0};
        std::optional<rclcpp::Time> last_fix_time_;
        std::deque<perseus_interfaces::msg::TagDetectionArray::ConstSharedPtr> pending_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Subscription<perseus_interfaces::msg::TagDetectionArray>::SharedPtr detections_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<perseus_interfaces::msg::LandmarkLocalisation>::SharedPtr status_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
        rclcpp::Service<CalibrateSrv>::SharedPtr calibrate_srv_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
        rclcpp::TimerBase::SharedPtr tf_timer_;
        rclcpp::TimerBase::SharedPtr status_timer_;
    };

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<perseus_landmarks::LandmarkLocaliser>());
    rclcpp::shutdown();
    return 0;
}
