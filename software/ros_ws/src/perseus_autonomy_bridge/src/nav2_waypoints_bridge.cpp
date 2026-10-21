#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "perseus_autonomy_bridge/goal_tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// bt services and messages
#include "perseus_interfaces/msg/navigation_data.hpp"
#include "perseus_interfaces/msg/waypoint.hpp"
#include "perseus_interfaces/srv/run_waypoints.hpp"
using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
using NavGoalTracker = perseus_autonomy_bridge::GoalTracker<GoalHandleNav::SharedPtr>;

// navigation_active stays true only while our goal is pending/active AND we
// have heard from it (send, accept or feedback) within this window.
static constexpr double kActivityTimeoutSec = 2.0;

static geometry_msgs::msg::Quaternion yaw_to_quat(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);
    return q;
}

class Nav2WaypointsBridge : public rclcpp::Node
{
public:
    Nav2WaypointsBridge()
        : Node("nav2_waypoints_bridge")
    {
        // Customizable feedback topic name
        _feedback_topic_name = this->declare_parameter<std::string>("feedback_topic", "/autonomy/navigation_info");

        _action_client = rclcpp_action::create_client<NavigateThroughPoses>(this, "/navigate_through_poses");

        // Publisher for navigation info
        _nav_info_pub = this->create_publisher<perseus_interfaces::msg::NavigationData>(
            _feedback_topic_name, 10);

        // Subscribe to the feedback topic using a generic subscription with a lambda
        // The feedback message contains the navigation data we need
        // (any client's goal, e.g. RViz, so the widget shows its progress too).
        auto generic_callback = [this](std::shared_ptr<rclcpp::SerializedMessage> msg)
        {
            std::lock_guard<std::mutex> lock(_data_mutex);
            // Store the serialized message for later deserialization
            this->_last_serialized_feedback = msg;
        };

        _feedback_sub = this->create_generic_subscription(
            "/navigate_through_poses/_action/feedback",
            "nav2_msgs/action/NavigateThroughPoses_FeedbackMessage",
            rclcpp::SensorDataQoS(),
            generic_callback);

        // Timer to publish navigation info periodically
        _nav_info_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Nav2WaypointsBridge::publish_nav_info, this));

        _srv_run = create_service<perseus_interfaces::srv::RunWaypoints>(
            "/autonomy/run_waypoints",
            std::bind(&Nav2WaypointsBridge::on_run, this, std::placeholders::_1, std::placeholders::_2));

        _srv_cancel = create_service<perseus_interfaces::srv::RunWaypoints>(
            "/autonomy/cancel_waypoints",
            std::bind(&Nav2WaypointsBridge::on_cancel, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "Ready: /autonomy/run_waypoints, /autonomy/cancel_waypoints");
        RCLCPP_INFO(get_logger(), "Feedback topic: '%s'", _feedback_topic_name.c_str());
    }

private:
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr _action_client;
    // The goal this bridge owns. Callbacks only touch it via their own token.
    NavGoalTracker _goal;

    rclcpp::Publisher<perseus_interfaces::msg::NavigationData>::SharedPtr _nav_info_pub;
    rclcpp::TimerBase::SharedPtr _nav_info_timer;
    std::shared_ptr<rclcpp::GenericSubscription> _feedback_sub;
    std::shared_ptr<rclcpp::SerializedMessage> _last_serialized_feedback;

    // Guards the feedback data and activity time below.
    std::mutex _data_mutex;
    // Current navigation feedback data
    nav2_msgs::action::NavigateThroughPoses::Feedback _current_feedback;
    // Last time our own goal showed signs of life (sent, accepted, feedback).
    rclcpp::Time _last_activity_time{0, 0, RCL_ROS_TIME};

    rclcpp::Service<perseus_interfaces::srv::RunWaypoints>::SharedPtr _srv_run;
    rclcpp::Service<perseus_interfaces::srv::RunWaypoints>::SharedPtr _srv_cancel;

    std::string _feedback_topic_name;

    void on_run(
        const std::shared_ptr<perseus_interfaces::srv::RunWaypoints::Request> req,
        std::shared_ptr<perseus_interfaces::srv::RunWaypoints::Response> resp)
    {
        try
        {
            if (!_action_client->wait_for_action_server(std::chrono::seconds(2)))
            {
                resp->success = false;
                resp->message = "Nav2 action server /navigate_through_poses not available";
                return;
            }

            if (req->waypoints.empty())
            {
                resp->success = false;
                resp->message = "No waypoints provided";
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Loading %zu waypoints from request",
                        req->waypoints.size());

            NavigateThroughPoses::Goal goal;
            goal.poses.reserve(req->waypoints.size());

            for (const auto& wp : req->waypoints)
            {
                geometry_msgs::msg::PoseStamped ps;
                ps.header.frame_id = "map";
                ps.pose.position.x = wp.x;
                ps.pose.position.y = wp.y;
                ps.pose.position.z = 0.0;
                ps.pose.orientation = yaw_to_quat(wp.yaw);
                goal.poses.push_back(ps);
            }

            // Supersede whatever we were running. A previous *pending* goal is
            // cancelled by its own response callback once Nav2 accepts it.
            const auto begin = _goal.begin_request();
            const auto token = begin.token;
            if (begin.to_cancel)
            {
                _action_client->async_cancel_goal(begin.to_cancel);
            }
            touch_activity();

            rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions opts;

            opts.goal_response_callback =
                [this, token](const GoalHandleNav::SharedPtr goal_handle)
            {
                switch (this->_goal.on_goal_response(token, goal_handle))
                {
                case NavGoalTracker::ResponseAction::Rejected:
                    RCLCPP_ERROR(this->get_logger(), "Goal rejected by Nav2");
                    break;
                case NavGoalTracker::ResponseAction::Cancel:
                    // Cancelled or superseded while the goal was in flight.
                    RCLCPP_INFO(this->get_logger(), "Goal accepted after it was cancelled/superseded; cancelling it");
                    this->_action_client->async_cancel_goal(goal_handle);
                    break;
                case NavGoalTracker::ResponseAction::Track:
                    this->touch_activity();
                    RCLCPP_INFO(this->get_logger(), "Goal accepted");
                    break;
                }
            };

            opts.result_callback =
                [this, token](const GoalHandleNav::WrappedResult& result)
            {
                // A late result for an older goal must not clear the current one.
                if (this->_goal.on_result(token))
                {
                    RCLCPP_INFO(this->get_logger(), "Goal finished (result code %d)",
                                static_cast<int>(result.code));
                }
            };

            opts.feedback_callback =
                [this, token](GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
            {
                if (!feedback || !this->_goal.is_current(token))
                    return;
                std::lock_guard<std::mutex> lock(_data_mutex);
                this->_current_feedback = *feedback;
                this->_last_activity_time = this->now();
            };

            _action_client->async_send_goal(goal, opts);

            resp->success = true;
            resp->message = "Goal sent to /navigate_through_poses";
        }
        catch (const std::exception& e)
        {
            resp->success = false;
            resp->message = e.what();
        }
    }

    void on_cancel(
        const std::shared_ptr<perseus_interfaces::srv::RunWaypoints::Request> /*req*/,
        std::shared_ptr<perseus_interfaces::srv::RunWaypoints::Response> resp)
    {
        const auto cancel = _goal.request_cancel();
        resp->success = true;
        switch (cancel.was)
        {
        case NavGoalTracker::State::Idle:
            resp->message = "No active goal";
            break;
        case NavGoalTracker::State::Pending:
            // No handle yet; the goal-response callback cancels it on acceptance.
            resp->message = "Cancel requested (goal will be cancelled as soon as Nav2 accepts it)";
            break;
        case NavGoalTracker::State::Active:
            _action_client->async_cancel_goal(cancel.to_cancel);
            resp->message = "Cancel requested";
            break;
        }
    }

    void touch_activity()
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        _last_activity_time = this->now();
    }

    void publish_nav_info()
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        // Try to deserialize the feedback if we have a serialized message
        if (_last_serialized_feedback)
        {
            try
            {
                // Deserialize the FeedbackMessage which wraps the actual Feedback
                rclcpp::Serialization<nav2_msgs::action::NavigateThroughPoses_FeedbackMessage> serialization;
                nav2_msgs::action::NavigateThroughPoses_FeedbackMessage feedback_msg;
                serialization.deserialize_message(_last_serialized_feedback.get(), &feedback_msg);

                // Extract the feedback struct
                _current_feedback = feedback_msg.feedback;
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Failed to deserialize feedback: %s", e.what());
            }
        }

        // Create and publish the navigation info message
        auto nav_info = std::make_unique<perseus_interfaces::msg::NavigationData>();
        nav_info->current_pose = _current_feedback.current_pose;
        nav_info->navigation_time = _current_feedback.navigation_time;
        nav_info->estimated_time_remaining = _current_feedback.estimated_time_remaining;
        nav_info->number_of_recoveries = _current_feedback.number_of_recoveries;
        nav_info->distance_remaining = _current_feedback.distance_remaining;
        nav_info->number_of_poses_remaining = _current_feedback.number_of_poses_remaining;

        // Active only while our goal is pending/active AND we heard from it
        // (sent, accepted or feedback) within the last kActivityTimeoutSec.
        const bool busy = _goal.busy();
        const double time_since_activity = (this->now() - _last_activity_time).seconds();
        nav_info->navigation_active = busy && (time_since_activity < kActivityTimeoutSec);
        RCLCPP_DEBUG(this->get_logger(), "Time since last goal activity: %.2f s, goal busy: %s",
                     time_since_activity, busy ? "yes" : "no");
        _nav_info_pub->publish(std::move(nav_info));
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nav2WaypointsBridge>());
    rclcpp::shutdown();
    return 0;
}
