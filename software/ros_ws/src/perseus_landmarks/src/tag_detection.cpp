#include "perseus_landmarks/tag_detection.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <stdexcept>

namespace perseus_landmarks
{
    namespace
    {
        /// @brief OpenCV dictionary id and black-square size in cells (marker bits + the
        ///        black border) for each supported family.
        struct FamilyInfo
        {
            cv::aruco::PredefinedDictionaryType dictionary;
            int black_cells{0};
        };

        const std::map<std::string, FamilyInfo>& families()
        {
            static const std::map<std::string, FamilyInfo> table = {
                {"16h5", {cv::aruco::DICT_APRILTAG_16h5, 6}},
                {"25h9", {cv::aruco::DICT_APRILTAG_25h9, 7}},
                {"36h10", {cv::aruco::DICT_APRILTAG_36h10, 8}},
                {"36h11", {cv::aruco::DICT_APRILTAG_36h11, 8}},
            };
            return table;
        }

        const FamilyInfo& family_info(const std::string& family)
        {
            const auto found = families().find(family);
            if (found == families().end())
            {
                throw std::invalid_argument("unsupported tag family '" + family + "' (use 16h5, 25h9, 36h10 or 36h11)");
            }
            return found->second;
        }
    }

    bool family_supported(const std::string& family)
    {
        return families().count(family) > 0;
    }

    std::vector<DetectedTag> detect_tags(const cv::Mat& gray, const CameraModel& camera, const DetectorParams& params,
                                         const std::function<double(int)>& size_of)
    {
        if (gray.empty() || gray.type() != CV_8UC1)
        {
            throw std::invalid_argument("detect_tags needs a non-empty CV_8UC1 image");
        }
        const FamilyInfo& info = family_info(params.family);
        cv::aruco::DetectorParameters detector_params;
        if (params.corner_refinement == "apriltag")
        {
            detector_params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
        }
        else if (params.corner_refinement == "subpix")
        {
            detector_params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        }
        else if (params.corner_refinement == "none")
        {
            detector_params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
        }
        else
        {
            throw std::invalid_argument("corner_refinement must be subpix, apriltag or none");
        }
        detector_params.minMarkerPerimeterRate = params.min_marker_perimeter_rate;
        const cv::aruco::ArucoDetector detector(cv::aruco::getPredefinedDictionary(info.dictionary), detector_params);

        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        detector.detectMarkers(gray, corners, ids);

        const cv::Matx33d k(camera.fx, 0.0, camera.cx, 0.0, camera.fy, camera.cy, 0.0, 0.0, 1.0);
        const cv::Mat distortion = camera.distortion.empty() ? cv::Mat::zeros(1, 5, CV_64F) : cv::Mat(camera.distortion).clone();

        std::vector<DetectedTag> tags;
        for (std::size_t i = 0; i < ids.size(); ++i)
        {
            DetectedTag tag;
            tag.id = ids[i];
            tag.size_m = size_of(tag.id);
            tag.corners = corners[i];
            const double half = tag.size_m / 2.0;
            // OpenCV's marker frame: x right, y up, z out of the tag towards the camera;
            // corners clockwise from top-left, as detectMarkers returns them.
            const std::vector<cv::Point3d> object = {
                {-half, half, 0.0}, {half, half, 0.0}, {half, -half, 0.0}, {-half, -half, 0.0}};
            const std::vector<cv::Point2d> image(corners[i].begin(), corners[i].end());
            // Both IPPE solutions: their error ratio says whether the orientation can be
            // trusted (a square seen head-on fits two mirror-tilted poses).
            std::vector<cv::Mat> rvecs;
            std::vector<cv::Mat> tvecs;
            cv::Mat errors;
            const int solutions =
                cv::solvePnPGeneric(object, image, k, distortion, rvecs, tvecs, false, cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(),
                                    cv::noArray(), errors);
            if (solutions < 1)
            {
                continue;
            }
            errors.convertTo(errors, CV_64F);
            // Keep only solutions in which the tag faces the camera: a printed tag seen
            // from behind cannot be decoded, but IPPE happily returns that pose (seen
            // exactly on the optical axis it can even rank first).
            auto faces_camera = [](const cv::Mat& r, const cv::Mat& t)
            {
                cv::Matx33d rotation;
                cv::Rodrigues(r, rotation);
                const cv::Vec3d normal(rotation(0, 2), rotation(1, 2), rotation(2, 2));
                const cv::Vec3d translation(t.ptr<double>());
                return normal.dot(-translation) > 0.0;
            };
            std::vector<int> valid;
            for (int s = 0; s < solutions; ++s)
            {
                if (faces_camera(rvecs[static_cast<std::size_t>(s)], tvecs[static_cast<std::size_t>(s)]))
                {
                    valid.push_back(s);
                }
            }
            const bool none_valid = valid.empty();
            if (none_valid)
            {
                // Noise near the head-on degeneracy can tip both poses: the POSITION is
                // still good, so keep the best one but flag its orientation unusable.
                for (int s = 0; s < solutions; ++s)
                {
                    valid.push_back(s);
                }
            }
            std::sort(valid.begin(), valid.end(), [&](int a, int b)
                      { return errors.at<double>(a) < errors.at<double>(b); });
            const int best = valid.front();
            tag.rvec = cv::Vec3d(rvecs[static_cast<std::size_t>(best)].ptr<double>());
            tag.tvec = cv::Vec3d(tvecs[static_cast<std::size_t>(best)].ptr<double>());
            tag.reprojection_error_px = errors.at<double>(best);
            // Only one physically possible pose: the orientation is not ambiguous.
            tag.ambiguity_ratio = none_valid ? 1.0 : 1e3;
            if (!none_valid && valid.size() > 1)
            {
                tag.ambiguity_ratio =
                    std::min(1e3, errors.at<double>(valid[1]) / std::max(tag.reprojection_error_px, 1e-6));
            }
            tag.distance_m = cv::norm(tag.tvec);
            cv::Matx33d rotation;
            cv::Rodrigues(tag.rvec, rotation);
            // Tag normal (its z axis) in camera coordinates, against the ray to the tag.
            const cv::Vec3d normal(rotation(0, 2), rotation(1, 2), rotation(2, 2));
            const cv::Vec3d to_camera = -tag.tvec / std::max(tag.distance_m, 1e-9);
            tag.view_angle_rad = std::acos(std::clamp(normal.dot(to_camera), -1.0, 1.0));
            tags.push_back(tag);
        }
        return tags;
    }

    cv::Mat render_tag(const std::string& family, int id, int cell_px)
    {
        const FamilyInfo& info = family_info(family);
        cv::Mat marker;
        cv::aruco::generateImageMarker(cv::aruco::getPredefinedDictionary(info.dictionary), id,
                                       info.black_cells * cell_px, marker, 1);
        cv::Mat with_border(marker.rows + 2 * cell_px, marker.cols + 2 * cell_px, CV_8UC1, cv::Scalar(255));
        marker.copyTo(with_border(cv::Rect(cell_px, cell_px, marker.cols, marker.rows)));
        return with_border;
    }

}
