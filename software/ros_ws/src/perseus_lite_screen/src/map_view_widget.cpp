#include "map_view_widget.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <tf2/utils.h>

#include <QApplication>
#include <QFile>
#include <QFont>
#include <QKeyEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QPolygonF>
#include <QRegularExpression>
#include <QString>
#include <QTextStream>
#include <QTransform>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace perseus_lite_screen
{

    namespace
    {

        constexpr int REFRESH_HZ = 20;

        QColor cell_to_color(int8_t value)
        {
            if (value < 0)
            {
                return QColor(60, 60, 60);  // unknown
            }
            if (value == 0)
            {
                return QColor(220, 220, 220);  // free
            }
            const int v = std::min<int>(value, 100);
            const int intensity = 220 - v * 2;  // 100 -> 20, 1 -> 218
            return QColor(intensity, intensity, intensity);
        }

        double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
        {
            tf2::Quaternion tf_q;
            tf2::fromMsg(q, tf_q);
            return tf2::getYaw(tf_q);
        }

        // Returns the first non-loopback IPv4 address found via getifaddrs().
        // Empty string if none. Prefers interfaces named eth*/en*/wl* over
        // virtual ones (docker, br-, veth, tun, vmnet) so the displayed IP is
        // the one a human on the same network would actually reach.
        QString primary_ipv4_address()
        {
            struct ifaddrs* ifap = nullptr;
            if (getifaddrs(&ifap) != 0 || ifap == nullptr)
            {
                return {};
            }

            QString preferred;
            QString fallback;
            for (struct ifaddrs* it = ifap; it != nullptr; it = it->ifa_next)
            {
                if (it->ifa_addr == nullptr ||
                    it->ifa_addr->sa_family != AF_INET ||
                    (it->ifa_flags & IFF_UP) == 0 ||
                    (it->ifa_flags & IFF_LOOPBACK) != 0)
                {
                    continue;
                }

                auto* sin = reinterpret_cast<struct sockaddr_in*>(it->ifa_addr);
                char buf[INET_ADDRSTRLEN] = {};
                if (inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof(buf)) ==
                    nullptr)
                {
                    continue;
                }

                const QString name = QString::fromLatin1(it->ifa_name);
                const QString addr = QString::fromLatin1(buf);

                const bool is_physical =
                    name.startsWith("eth") || name.startsWith("en") ||
                    name.startsWith("wl") || name.startsWith("wlan");
                const bool is_virtual =
                    name.startsWith("docker") || name.startsWith("br-") ||
                    name.startsWith("veth") || name.startsWith("tun") ||
                    name.startsWith("vmnet") || name.startsWith("vir");

                if (is_physical && preferred.isEmpty())
                {
                    preferred = addr;
                }
                else if (!is_virtual && fallback.isEmpty())
                {
                    fallback = addr;
                }
            }
            freeifaddrs(ifap);

            return preferred.isEmpty() ? fallback : preferred;
        }

        // Reads /proc/meminfo and returns MemAvailable in kibibytes (the value
        // the kernel publishes for "memory the next process can claim without
        // swapping"). Returns -1 on failure.
        long mem_available_kib()
        {
            QFile f("/proc/meminfo");
            if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
            {
                return -1;
            }
            QTextStream s(&f);
            const QRegularExpression re(
                R"(^MemAvailable:\s+(\d+)\s+kB\s*$)");
            QString line;
            while (s.readLineInto(&line))
            {
                const auto m = re.match(line);
                if (m.hasMatch())
                {
                    return m.captured(1).toLong();
                }
            }
            return -1;
        }

        QString format_mem_free(long kib)
        {
            if (kib < 0)
            {
                return QStringLiteral("FREE -");
            }
            const double mib = kib / 1024.0;
            if (mib >= 1024.0)
            {
                return QString::asprintf("FREE %.1f GiB", mib / 1024.0);
            }
            return QString::asprintf("FREE %.0f MiB", mib);
        }

    }  // namespace

    MapViewWidget::MapViewWidget(MapScreenNode* node, QWidget* parent)
        : QWidget(parent),
          _node(node)
    {
        setWindowTitle("Perseus Lite Map");
        setAutoFillBackground(false);
        setAttribute(Qt::WA_OpaquePaintEvent, true);

        _pixels_per_meter = node->declare_parameter<double>("pixels_per_meter", 60.0);

        connect(&_refresh_timer, &QTimer::timeout, this,
                QOverload<>::of(&QWidget::update));
        _refresh_timer.start(1000 / REFRESH_HZ);
    }

    void MapViewWidget::keyPressEvent(QKeyEvent* event)
    {
        if (event->key() == Qt::Key_Escape || event->key() == Qt::Key_Q)
        {
            QApplication::quit();
            return;
        }
        if (event->key() == Qt::Key_Plus || event->key() == Qt::Key_Equal)
        {
            _pixels_per_meter *= 1.25;
        }
        else if (event->key() == Qt::Key_Minus)
        {
            _pixels_per_meter /= 1.25;
        }
        QWidget::keyPressEvent(event);
    }

    void MapViewWidget::_maybe_rebuild_map_image()
    {
        auto map = _node->latest_map();
        if (!map)
        {
            return;
        }
        if (map.get() == _cached_map.get())
        {
            return;  // same map pointer; image still valid
        }

        _cached_map = map;
        _map_origin_x = map->info.origin.position.x;
        _map_origin_y = map->info.origin.position.y;
        _map_origin_yaw = yaw_from_quaternion(map->info.origin.orientation);
        _map_resolution = map->info.resolution;

        const int w = static_cast<int>(map->info.width);
        const int h = static_cast<int>(map->info.height);
        if (w <= 0 || h <= 0)
        {
            _map_image = QImage();
            return;
        }

        QImage img(w, h, QImage::Format_ARGB32);
        // OccupancyGrid: row-major, origin row at bottom. We render with the
        // painter's negative-Y scale so we want the QImage's top row to match
        // OccupancyGrid row 0 (bottom-left in world frame). A vertical flip is
        // applied at draw time by negating the Y scale of the painter, so we
        // store the image with row 0 at bottom (matching the grid's data layout
        // when iterated row-major).
        for (int row = 0; row < h; ++row)
        {
            QRgb* dst = reinterpret_cast<QRgb*>(img.scanLine(h - 1 - row));
            const int row_off = row * w;
            for (int col = 0; col < w; ++col)
            {
                const int8_t v = map->data[row_off + col];
                const QColor c = cell_to_color(v);
                dst[col] = c.rgba();
            }
        }
        _map_image = std::move(img);
    }

    void MapViewWidget::_maybe_refresh_system_info()
    {
        const auto now = std::chrono::steady_clock::now();
        if (_system_info_last_refresh.has_value() &&
            (now - *_system_info_last_refresh) < std::chrono::seconds(1))
        {
            return;
        }
        _system_info_last_refresh = now;

        const QString ip = primary_ipv4_address();
        _ipv4_text =
            ip.isEmpty() ? QStringLiteral("IP -") : QStringLiteral("IP ") + ip;
        _mem_text = format_mem_free(mem_available_kib());
    }

    void MapViewWidget::paintEvent(QPaintEvent*)
    {
        QPainter painter(this);
        painter.fillRect(rect(), QColor(20, 20, 20));

        _maybe_refresh_system_info();

        auto pose_opt = _node->lookup_robot_pose();
        if (!pose_opt.has_value())
        {
            painter.setPen(Qt::white);
            QFont f = painter.font();
            f.setPointSize(28);
            painter.setFont(f);
            painter.drawText(rect(), Qt::AlignCenter, "Waiting for TF (map -> base_link)...");
            return;
        }

        const RobotPose& pose = *pose_opt;

        _maybe_rebuild_map_image();

        // World-frame painter transform:
        //   1) move origin to widget center (robot at center of screen)
        //   2) flip Y so world +Y is screen up; scale to pixels per meter
        //   3) translate so robot is at origin -> pan map under robot
        painter.save();
        painter.translate(width() / 2.0, height() / 2.0);
        painter.scale(_pixels_per_meter, -_pixels_per_meter);
        painter.translate(-pose.x, -pose.y);

        _draw_map(painter);
        _draw_path(painter);
        _draw_lidar(painter, pose);
        _draw_goal(painter);
        _draw_robot(painter, pose);

        painter.restore();

        _draw_hud(painter, pose_opt);
    }

    void MapViewWidget::_draw_map(QPainter& painter)
    {
        if (_map_image.isNull())
        {
            return;
        }

        painter.save();
        // Place the OccupancyGrid origin (its bottom-left corner in world coords)
        // at the painter's current origin, optionally rotated by the grid's yaw.
        painter.translate(_map_origin_x, _map_origin_y);
        if (std::abs(_map_origin_yaw) > 1e-6)
        {
            painter.rotate(_map_origin_yaw * 180.0 / M_PI);
        }
        // Each pixel of _map_image is one cell; cell side is _map_resolution m.
        const double w_meters = _map_image.width() * _map_resolution;
        const double h_meters = _map_image.height() * _map_resolution;
        const QRectF target(0.0, 0.0, w_meters, h_meters);
        painter.drawImage(target, _map_image);
        painter.restore();
    }

    void MapViewWidget::_draw_lidar(QPainter& painter, const RobotPose&)
    {
        auto scan = _node->latest_scan();
        if (!scan)
        {
            return;
        }

        QPen pen(QColor(255, 80, 80, 220));
        pen.setCosmetic(true);
        pen.setWidth(3);
        painter.setPen(pen);

        const std::string scan_frame = scan->header.frame_id;
        const rclcpp::Time stamp(scan->header.stamp);

        double angle = scan->angle_min;
        for (size_t i = 0; i < scan->ranges.size();
             ++i, angle += scan->angle_increment)
        {
            const float r = scan->ranges[i];
            if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max)
            {
                continue;
            }
            const double lx = r * std::cos(angle);
            const double ly = r * std::sin(angle);
            double mx = 0.0;
            double my = 0.0;
            if (!_node->try_transform_point(scan_frame, stamp, lx, ly, mx, my))
            {
                continue;
            }
            painter.drawPoint(QPointF(mx, my));
        }
    }

    void MapViewWidget::_draw_path(QPainter& painter)
    {
        auto path = _node->latest_path();
        if (!path || path->poses.empty())
        {
            return;
        }
        QPen pen(QColor(80, 220, 255, 230));
        pen.setCosmetic(true);
        pen.setWidth(3);
        painter.setPen(pen);

        QPolygonF poly;
        poly.reserve(static_cast<int>(path->poses.size()));
        for (const auto& ps : path->poses)
        {
            poly << QPointF(ps.pose.position.x, ps.pose.position.y);
        }
        painter.drawPolyline(poly);
    }

    void MapViewWidget::_draw_goal(QPainter& painter)
    {
        auto goal = _node->latest_goal();
        if (!goal)
        {
            return;
        }
        const double gx = goal->pose.position.x;
        const double gy = goal->pose.position.y;
        const double gyaw = yaw_from_quaternion(goal->pose.orientation);

        painter.save();
        painter.translate(gx, gy);
        painter.rotate(gyaw * 180.0 / M_PI);

        QPen pen(QColor(255, 215, 0, 240));
        pen.setCosmetic(true);
        pen.setWidth(3);
        painter.setPen(pen);
        painter.setBrush(QColor(255, 215, 0, 120));

        // Flag: pole + triangular flag, ~0.4 m
        painter.drawLine(QPointF(0.0, 0.0), QPointF(0.0, 0.4));
        QPolygonF flag;
        flag << QPointF(0.0, 0.4) << QPointF(0.25, 0.32) << QPointF(0.0, 0.24);
        painter.drawPolygon(flag);

        painter.restore();
    }

    void MapViewWidget::_draw_robot(QPainter& painter, const RobotPose& pose)
    {
        painter.save();
        painter.translate(pose.x, pose.y);
        painter.rotate(pose.yaw * 180.0 / M_PI);

        QPen border(QColor(255, 255, 255), 2);
        border.setCosmetic(true);
        painter.setPen(border);
        painter.setBrush(QColor(255, 140, 0, 230));

        // Triangle pointing along +X (robot forward) ~0.4 m long, 0.3 m wide.
        QPolygonF tri;
        tri << QPointF(0.25, 0.0) << QPointF(-0.15, 0.15) << QPointF(-0.15, -0.15);
        painter.drawPolygon(tri);
        painter.restore();
    }

    void MapViewWidget::_draw_hud(QPainter& painter,
                                  const std::optional<RobotPose>& pose)
    {
        painter.resetTransform();

        QFont font = painter.font();
        font.setPointSize(12);
        font.setBold(true);
        painter.setFont(font);

        painter.setPen(QColor(230, 230, 230));
        if (pose.has_value())
        {
            const QString status = QString::asprintf(
                "x %+.2f m   y %+.2f m   yaw %+.1f deg", pose->x, pose->y,
                pose->yaw * 180.0 / M_PI);
            painter.drawText(12, 22, status);
        }
        const QString sys_line = _ipv4_text + QStringLiteral("    ") + _mem_text;
        painter.drawText(12, 42, sys_line);

        const auto map_age =
            (_node->now() - _node->last_map_stamp()).seconds();
        const bool fresh = _cached_map && map_age < 5.0;
        const QColor dot = fresh ? QColor(80, 220, 80) : QColor(220, 160, 60);
        painter.setBrush(dot);
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QPointF(width() - 24, 18), 7, 7);
        painter.setPen(QColor(230, 230, 230));
        painter.drawText(width() - 130, 22, fresh ? "MAP OK" : "MAP STALE");

        // Scale bar — 1 m line in screen coords, bottom-right
        const double bar_px = _pixels_per_meter;
        const double bar_y = height() - 22;
        const double bar_x_end = width() - 20;
        const double bar_x_start = bar_x_end - bar_px;
        QPen bar_pen(QColor(230, 230, 230));
        bar_pen.setWidth(3);
        painter.setPen(bar_pen);
        painter.drawLine(QPointF(bar_x_start, bar_y), QPointF(bar_x_end, bar_y));
        painter.drawLine(QPointF(bar_x_start, bar_y - 5),
                         QPointF(bar_x_start, bar_y + 5));
        painter.drawLine(QPointF(bar_x_end, bar_y - 5),
                         QPointF(bar_x_end, bar_y + 5));
        painter.drawText(QPointF(bar_x_start, bar_y - 8), "1 m");
    }

}  // namespace perseus_lite_screen
