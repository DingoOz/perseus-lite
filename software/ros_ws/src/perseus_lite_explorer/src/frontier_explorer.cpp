#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using namespace std::chrono_literals;

// Frontier: a connected cluster of free cells that border unknown space.
struct Frontier
{
    double centroid_x;
    double centroid_y;
    std::size_t size;
};

// Convert (mx, my) grid index to world coordinates using map metadata.
static void cell_to_world(const nav_msgs::msg::OccupancyGrid& map,
                          unsigned int mx, unsigned int my,
                          double& wx, double& wy)
{
    wx = map.info.origin.position.x + (mx + 0.5) * map.info.resolution;
    wy = map.info.origin.position.y + (my + 0.5) * map.info.resolution;
}

// True if the cell is free (occupancy below threshold and not unknown).
static bool is_free(int8_t v, int8_t free_threshold)
{
    return v >= 0 && v <= free_threshold;
}

class FrontierExplorer : public rclcpp::Node
{
public:
    FrontierExplorer()
        : Node("frontier_explorer"),
          _tf_buffer(this->get_clock()),
          _tf_listener(_tf_buffer)
    {
        _map_topic = declare_parameter<std::string>("map_topic", "/map");
        _global_frame = declare_parameter<std::string>("global_frame", "map");
        _robot_frame = declare_parameter<std::string>("robot_frame", "base_link");
        _planning_period = declare_parameter<double>("planning_period_sec", 3.0);
        _min_frontier_size = declare_parameter<int>("min_frontier_size_cells", 8);
        _free_threshold = declare_parameter<int>("free_threshold", 30);
        _occupied_threshold = declare_parameter<int>("occupied_threshold", 65);
        _blacklist_radius = declare_parameter<double>("blacklist_radius_m", 0.4);
        _goal_reached_radius = declare_parameter<double>("goal_reached_radius_m", 0.35);
        _distance_weight = declare_parameter<double>("distance_weight", 1.0);
        _size_weight = declare_parameter<double>("size_weight", 0.5);
        _goal_timeout_sec = declare_parameter<double>("goal_timeout_sec", 60.0);
        _start_delay_sec = declare_parameter<double>("start_delay_sec", 5.0);

        _action_client = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

        rclcpp::QoS map_qos(1);
        map_qos.transient_local().reliable();
        _map_sub = create_subscription<nav_msgs::msg::OccupancyGrid>(
            _map_topic, map_qos,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { _latest_map = std::move(msg); });

        _frontier_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/frontier_explorer/frontiers", 10);

        _start_time = now();
        _plan_timer = create_wall_timer(
            std::chrono::duration<double>(_planning_period),
            std::bind(&FrontierExplorer::on_plan_tick, this));

        RCLCPP_INFO(get_logger(),
                    "FrontierExplorer up — map=%s, period=%.1fs, min_frontier=%d cells",
                    _map_topic.c_str(), _planning_period, _min_frontier_size);
    }

private:
    // Parameters
    std::string _map_topic;
    std::string _global_frame;
    std::string _robot_frame;
    double _planning_period;
    int _min_frontier_size;
    int _free_threshold;
    int _occupied_threshold;
    double _blacklist_radius;
    double _goal_reached_radius;
    double _distance_weight;
    double _size_weight;
    double _goal_timeout_sec;
    double _start_delay_sec;

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _map_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _frontier_pub;
    rclcpp::TimerBase::SharedPtr _plan_timer;
    rclcpp_action::Client<NavigateToPose>::SharedPtr _action_client;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;

    // State
    nav_msgs::msg::OccupancyGrid::SharedPtr _latest_map;
    GoalHandleNav::SharedPtr _active_goal;
    geometry_msgs::msg::Point _current_goal_point;
    bool _has_active_goal = false;
    rclcpp::Time _goal_sent_time;
    rclcpp::Time _start_time;

    // Failed goals to avoid revisiting (world coords).
    std::vector<geometry_msgs::msg::Point> _blacklist;

    void on_plan_tick()
    {
        // Wait for SLAM to publish a map and the EKF/TF tree to settle.
        if ((now() - _start_time).seconds() < _start_delay_sec) {
            return;
        }
        if (!_latest_map) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Waiting for map on '%s'...", _map_topic.c_str());
            return;
        }

        // Time-out a stuck goal so exploration always makes progress.
        if (_has_active_goal &&
            (now() - _goal_sent_time).seconds() > _goal_timeout_sec) {
            RCLCPP_WARN(get_logger(), "Goal timed out after %.1fs — cancelling and blacklisting",
                        _goal_timeout_sec);
            if (_active_goal) {
                _action_client->async_cancel_goal(_active_goal);
            }
            _blacklist.push_back(_current_goal_point);
            _has_active_goal = false;
            _active_goal.reset();
        }

        if (_has_active_goal) {
            return;  // Let the current goal run.
        }

        geometry_msgs::msg::Point robot;
        if (!get_robot_position(robot)) {
            return;
        }

        std::vector<Frontier> frontiers = find_frontiers(*_latest_map);
        publish_frontier_markers(frontiers);

        if (frontiers.empty()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                                 "No frontiers detected — exploration may be complete.");
            return;
        }

        // Score = size_weight * size  -  distance_weight * dist
        // (bigger frontiers and closer ones are preferred)
        const Frontier* best = nullptr;
        double best_score = -std::numeric_limits<double>::infinity();
        for (const auto& f : frontiers) {
            if (is_blacklisted(f.centroid_x, f.centroid_y)) {
                continue;
            }
            const double dx = f.centroid_x - robot.x;
            const double dy = f.centroid_y - robot.y;
            const double dist = std::hypot(dx, dy);
            const double score = _size_weight * static_cast<double>(f.size)
                                 - _distance_weight * dist;
            if (score > best_score) {
                best_score = score;
                best = &f;
            }
        }

        if (!best) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                                 "All %zu frontiers blacklisted — clearing blacklist to retry.",
                                 frontiers.size());
            _blacklist.clear();
            return;
        }

        send_goal(best->centroid_x, best->centroid_y);
    }

    bool get_robot_position(geometry_msgs::msg::Point& out)
    {
        try {
            auto tf = _tf_buffer.lookupTransform(
                _global_frame, _robot_frame, tf2::TimePointZero,
                tf2::durationFromSec(0.5));
            out.x = tf.transform.translation.x;
            out.y = tf.transform.translation.y;
            out.z = 0.0;
            return true;
        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "TF lookup %s -> %s failed: %s",
                                 _global_frame.c_str(), _robot_frame.c_str(), e.what());
            return false;
        }
    }

    bool is_blacklisted(double x, double y) const
    {
        for (const auto& p : _blacklist) {
            if (std::hypot(p.x - x, p.y - y) < _blacklist_radius) {
                return true;
            }
        }
        return false;
    }

    // Flood-fill frontiers: a frontier cell is a free cell (0..free_threshold) with
    // at least one unknown (-1) 4-neighbour. Adjacent frontier cells are clustered.
    std::vector<Frontier> find_frontiers(const nav_msgs::msg::OccupancyGrid& map) const
    {
        const auto width = map.info.width;
        const auto height = map.info.height;
        const auto& data = map.data;
        if (width == 0 || height == 0) {
            return {};
        }

        const auto idx = [width](unsigned int x, unsigned int y) {
            return static_cast<std::size_t>(y) * width + x;
        };

        // Mark every cell that is on a free/unknown boundary.
        std::vector<uint8_t> is_frontier(static_cast<std::size_t>(width) * height, 0);
        for (unsigned int y = 0; y < height; ++y) {
            for (unsigned int x = 0; x < width; ++x) {
                const int8_t v = data[idx(x, y)];
                if (!is_free(v, static_cast<int8_t>(_free_threshold))) {
                    continue;
                }
                bool borders_unknown = false;
                if (x > 0 && data[idx(x - 1, y)] == -1) borders_unknown = true;
                else if (x + 1 < width && data[idx(x + 1, y)] == -1) borders_unknown = true;
                else if (y > 0 && data[idx(x, y - 1)] == -1) borders_unknown = true;
                else if (y + 1 < height && data[idx(x, y + 1)] == -1) borders_unknown = true;
                if (borders_unknown) {
                    is_frontier[idx(x, y)] = 1;
                }
            }
        }

        // BFS-cluster the marked cells.
        std::vector<uint8_t> visited(static_cast<std::size_t>(width) * height, 0);
        std::vector<Frontier> frontiers;
        for (unsigned int y = 0; y < height; ++y) {
            for (unsigned int x = 0; x < width; ++x) {
                const auto seed = idx(x, y);
                if (!is_frontier[seed] || visited[seed]) {
                    continue;
                }

                std::queue<std::pair<unsigned int, unsigned int>> q;
                q.emplace(x, y);
                visited[seed] = 1;

                double sum_x = 0.0;
                double sum_y = 0.0;
                std::size_t count = 0;
                while (!q.empty()) {
                    auto [cx, cy] = q.front();
                    q.pop();
                    double wx;
                    double wy;
                    cell_to_world(map, cx, cy, wx, wy);
                    sum_x += wx;
                    sum_y += wy;
                    ++count;

                    const int dxs[4] = {-1, 1, 0, 0};
                    const int dys[4] = {0, 0, -1, 1};
                    for (int n = 0; n < 4; ++n) {
                        const int nx = static_cast<int>(cx) + dxs[n];
                        const int ny = static_cast<int>(cy) + dys[n];
                        if (nx < 0 || ny < 0 ||
                            nx >= static_cast<int>(width) ||
                            ny >= static_cast<int>(height)) {
                            continue;
                        }
                        const auto ni = idx(static_cast<unsigned int>(nx),
                                            static_cast<unsigned int>(ny));
                        if (is_frontier[ni] && !visited[ni]) {
                            visited[ni] = 1;
                            q.emplace(static_cast<unsigned int>(nx),
                                      static_cast<unsigned int>(ny));
                        }
                    }
                }

                if (static_cast<int>(count) >= _min_frontier_size) {
                    Frontier f;
                    f.centroid_x = sum_x / static_cast<double>(count);
                    f.centroid_y = sum_y / static_cast<double>(count);
                    f.size = count;
                    frontiers.push_back(f);
                }
            }
        }
        return frontiers;
    }

    void send_goal(double x, double y)
    {
        if (!_action_client->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_WARN(get_logger(), "/navigate_to_pose action server not available");
            return;
        }

        // Aim the goal heading towards the frontier (from the robot's current position).
        geometry_msgs::msg::Point robot;
        double yaw = 0.0;
        if (get_robot_position(robot)) {
            yaw = std::atan2(y - robot.y, x - robot.x);
        }

        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = _global_frame;
        goal.pose.header.stamp = now();
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.position.z = 0.0;
        goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
        goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

        rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
        opts.goal_response_callback =
            [this](GoalHandleNav::SharedPtr handle) {
                if (!handle) {
                    RCLCPP_WARN(get_logger(), "Goal rejected by Nav2 — blacklisting");
                    _blacklist.push_back(_current_goal_point);
                    _has_active_goal = false;
                    return;
                }
                _active_goal = handle;
                RCLCPP_INFO(get_logger(), "Goal accepted (%.2f, %.2f)",
                            _current_goal_point.x, _current_goal_point.y);
            };
        opts.result_callback =
            [this](const GoalHandleNav::WrappedResult& result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(get_logger(), "Reached frontier (%.2f, %.2f)",
                                    _current_goal_point.x, _current_goal_point.y);
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_WARN(get_logger(), "Goal aborted — blacklisting");
                        _blacklist.push_back(_current_goal_point);
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_INFO(get_logger(), "Goal cancelled");
                        break;
                    default:
                        RCLCPP_WARN(get_logger(), "Goal ended with unknown code");
                        _blacklist.push_back(_current_goal_point);
                }
                _has_active_goal = false;
                _active_goal.reset();
            };

        _current_goal_point.x = x;
        _current_goal_point.y = y;
        _current_goal_point.z = 0.0;
        _has_active_goal = true;
        _goal_sent_time = now();
        _action_client->async_send_goal(goal, opts);
    }

    void publish_frontier_markers(const std::vector<Frontier>& frontiers)
    {
        visualization_msgs::msg::MarkerArray arr;
        // Clear previous markers.
        visualization_msgs::msg::Marker clear;
        clear.header.frame_id = _global_frame;
        clear.header.stamp = now();
        clear.ns = "frontiers";
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(clear);

        int id = 0;
        for (const auto& f : frontiers) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = _global_frame;
            m.header.stamp = now();
            m.ns = "frontiers";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = f.centroid_x;
            m.pose.position.y = f.centroid_y;
            m.pose.position.z = 0.1;
            m.pose.orientation.w = 1.0;
            const double scale = std::clamp(0.05 + 0.01 * static_cast<double>(f.size), 0.1, 0.6);
            m.scale.x = scale;
            m.scale.y = scale;
            m.scale.z = scale;
            const bool blocked = is_blacklisted(f.centroid_x, f.centroid_y);
            m.color.a = 0.8f;
            m.color.r = blocked ? 1.0f : 0.0f;
            m.color.g = blocked ? 0.0f : 1.0f;
            m.color.b = 0.0f;
            arr.markers.push_back(m);
        }
        _frontier_pub->publish(arr);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierExplorer>());
    rclcpp::shutdown();
    return 0;
}
