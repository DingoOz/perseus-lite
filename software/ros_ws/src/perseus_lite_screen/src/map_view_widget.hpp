#pragma once

#include <QImage>
#include <QString>
#include <QTimer>
#include <QWidget>
#include <chrono>
#include <optional>

#include "map_screen_node.hpp"

namespace perseus_lite_screen
{

    class MapViewWidget : public QWidget
    {
        Q_OBJECT

    public:
        explicit MapViewWidget(MapScreenNode* node, QWidget* parent = nullptr);

    protected:
        void paintEvent(QPaintEvent* event) override;
        void keyPressEvent(QKeyEvent* event) override;

    private:
        void _maybe_rebuild_map_image();
        void _maybe_refresh_system_info();
        void _draw_map(QPainter& painter);
        void _draw_lidar(QPainter& painter, const RobotPose& robot_pose);
        void _draw_path(QPainter& painter);
        void _draw_goal(QPainter& painter);
        void _draw_robot(QPainter& painter, const RobotPose& robot_pose);
        void _draw_hud(QPainter& painter, const std::optional<RobotPose>& pose);

        MapScreenNode* _node;
        QTimer _refresh_timer;

        nav_msgs::msg::OccupancyGrid::ConstSharedPtr _cached_map;
        QImage _map_image;
        double _map_origin_x = 0.0;
        double _map_origin_y = 0.0;
        double _map_origin_yaw = 0.0;
        double _map_resolution = 0.05;

        double _pixels_per_meter = 60.0;

        // System-info HUD (IPv4 + free memory). Refreshed at most once per
        // second from inside paintEvent; cached values used on every repaint.
        QString _ipv4_text;
        QString _mem_text;
        std::optional<std::chrono::steady_clock::time_point> _system_info_last_refresh;
    };

}  // namespace perseus_lite_screen
