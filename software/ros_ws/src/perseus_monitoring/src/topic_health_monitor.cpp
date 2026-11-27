/// @file topic_health_monitor.cpp
/// @brief Type-agnostic health monitor for a list of topics: rate, bandwidth, staleness.
///
/// Watching a topic is a config change, not a code change: add it to `topics` and its
/// expected rate to `expected_rates_hz`. Each topic is subscribed generically (its type
/// is read from the ROS graph), so the monitor never needs the message headers.
///
/// Subscribing costs the full message bandwidth, so prefer a small companion topic as a
/// liveness proxy for a big one: /oak/stereo/camera_info instead of the depth image.
///
/// Publishes perseus_interfaces/SystemHealth on /health/topics once per period, and logs
/// every status change.

#include <chrono>
#include <memory>
#include <perseus_interfaces/msg/system_health.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <string>
#include <vector>

#include "perseus_monitoring/topic_tracker.hpp"

namespace perseus_monitoring
{
    class TopicHealthMonitor : public rclcpp::Node
    {
    public:
        explicit TopicHealthMonitor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
            : rclcpp::Node("topic_health_monitor", options)
        {
            const auto topics = declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{});
            auto rates = declare_parameter<std::vector<double>>("expected_rates_hz", std::vector<double>{});
            window_s_ = declare_parameter<double>("window_s", 2.0);
            rate_tolerance_ = declare_parameter<double>("rate_tolerance", 0.7);
            stale_timeout_s_ = declare_parameter<double>("stale_timeout_s", 2.0);
            const double publish_rate_hz = std::max(0.1, declare_parameter<double>("publish_rate_hz", 1.0));
            const auto health_topic = declare_parameter<std::string>("health_topic", "/health/topics");

            if (rates.size() != topics.size())
            {
                RCLCPP_WARN(get_logger(),
                            "expected_rates_hz has %zu entries for %zu topics; missing rates default to 0 (no rate check)",
                            rates.size(), topics.size());
                rates.resize(topics.size(), 0.0);
            }
            for (std::size_t i = 0; i < topics.size(); ++i)
            {
                watched_.push_back(std::make_unique<Watched>(get_node_topics_interface()->resolve_topic_name(topics[i]),
                                                             rates[i], window_s_));
            }

            health_pub_ = create_publisher<perseus_interfaces::msg::SystemHealth>(health_topic, 10);
            timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_hz), [this]()
                                       { on_timer(); });
            RCLCPP_INFO(get_logger(), "Watching %zu topics; health on %s", watched_.size(), health_topic.c_str());
        }

    private:
        struct Watched
        {
            Watched(std::string name, double rate, double window)
                : topic(std::move(name)),
                  expected_rate_hz(rate),
                  tracker(window)
            {
            }

            std::string topic;
            double expected_rate_hz;
            TopicTracker tracker;
            std::string type;
            rclcpp::GenericSubscription::SharedPtr subscription;
            std::optional<TopicStatus> last_status;
        };

        /// @brief Subscribes once the topic's type is known, matching the publishers'
        ///        reliability (a reliable subscriber never hears a best-effort sensor).
        void try_subscribe(Watched& watched)
        {
            const auto publishers = get_publishers_info_by_topic(watched.topic);
            if (publishers.empty())
            {
                return;
            }
            watched.type = publishers.front().topic_type();
            bool any_best_effort = false;
            for (const auto& publisher : publishers)
            {
                any_best_effort |= publisher.qos_profile().reliability() == rclcpp::ReliabilityPolicy::BestEffort;
            }
            rclcpp::QoS qos(10);
            if (any_best_effort)
            {
                qos.best_effort();
            }
            qos.durability_volatile();
            Watched* target = &watched;
            watched.subscription = create_generic_subscription(
                watched.topic, watched.type, qos, [this, target](std::shared_ptr<const rclcpp::SerializedMessage> message)
                { target->tracker.record(now().seconds(), message->size()); });
        }

        void on_timer()
        {
            const double t = now().seconds();
            perseus_interfaces::msg::SystemHealth health;
            health.header.stamp = now();
            health.all_ok = true;
            for (auto& watched_ptr : watched_)
            {
                Watched& watched = *watched_ptr;
                if (!watched.subscription)
                {
                    try_subscribe(watched);
                }
                perseus_interfaces::msg::TopicHealth entry;
                entry.topic = watched.topic;
                entry.type = watched.type;
                entry.expected_rate_hz = watched.expected_rate_hz;
                entry.rate_hz = watched.tracker.rate_hz(t);
                entry.bandwidth_bytes_per_s = watched.tracker.bandwidth(t);
                const auto age = watched.tracker.age_s(t);
                entry.age_s = age ? *age : -1.0;
                entry.publisher_count = static_cast<std::uint32_t>(count_publishers(watched.topic));
                const TopicStatus status = classify_topic(entry.publisher_count, age, entry.rate_hz,
                                                          watched.expected_rate_hz, rate_tolerance_, stale_timeout_s_);
                entry.status = static_cast<std::uint8_t>(status);
                health.all_ok = health.all_ok && status == TopicStatus::OK;
                log_change(watched, status, entry);
                health.topics.push_back(entry);
            }
            health_pub_->publish(health);
        }

        void log_change(Watched& watched, TopicStatus status, const perseus_interfaces::msg::TopicHealth& entry)
        {
            if (watched.last_status == status)
            {
                return;
            }
            static const char* const NAMES[] = {"OK", "SLOW", "STALE", "NO_PUBLISHER"};
            const char* name = NAMES[static_cast<std::size_t>(status)];
            if (status == TopicStatus::OK)
            {
                RCLCPP_INFO(get_logger(), "%s: OK (%.1f Hz)", watched.topic.c_str(), entry.rate_hz);
            }
            else if (watched.last_status || status != TopicStatus::NO_PUBLISHER)
            {
                // A topic with no publisher at startup is normal while the stack comes up:
                // only report it once it was seen and then lost.
                RCLCPP_WARN(get_logger(), "%s: %s (%.1f Hz of %.1f expected, last message %.1f s ago)",
                            watched.topic.c_str(), name, entry.rate_hz, watched.expected_rate_hz, entry.age_s);
            }
            watched.last_status = status;
        }

        std::vector<std::unique_ptr<Watched>> watched_;
        double window_s_{2.0};
        double rate_tolerance_{0.7};
        double stale_timeout_s_{2.0};
        rclcpp::Publisher<perseus_interfaces::msg::SystemHealth>::SharedPtr health_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
    };

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<perseus_monitoring::TopicHealthMonitor>());
    rclcpp::shutdown();
    return 0;
}
