#include "perseus_landmarks/tag_detection.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using perseus_landmarks::CameraModel;
using perseus_landmarks::detect_tags;
using perseus_landmarks::DetectorParams;
using perseus_landmarks::render_tag;

namespace
{
    const CameraModel CAMERA{432.5, 432.5, 320.0, 240.0, {}};
    constexpr double SIZE = 0.16;  // black square edge, metres

    /// @brief Renders tag @p id at pose (rvec, tvec) in a 640x480 camera image, by the
    ///        exact pinhole homography of the tag's plane.
    cv::Mat render_scene(int id, const cv::Vec3d& rvec, const cv::Vec3d& tvec)
    {
        const cv::Mat texture = render_tag("36h11", id, 20);  // 10 cells x 20 px
        const double full = SIZE * 10.0 / 8.0;                // tag incl. white border
        cv::Matx33d rotation;
        cv::Rodrigues(rvec, rotation);
        const cv::Matx33d k(CAMERA.fx, 0, CAMERA.cx, 0, CAMERA.fy, CAMERA.cy, 0, 0, 1);
        // Plane points (X, Y, 0) -> image: K [r1 r2 t].
        const cv::Matx33d plane(rotation(0, 0), rotation(0, 1), tvec[0], rotation(1, 0), rotation(1, 1), tvec[1],
                                rotation(2, 0), rotation(2, 1), tvec[2]);
        // Texture pixel (u, v) -> plane: X = (u / W - 0.5) * full, Y = (0.5 - v / H) * full.
        const double w = texture.cols;
        const cv::Matx33d to_plane(full / w, 0, -full / 2, 0, -full / w, full / 2, 0, 0, 1);
        const cv::Matx33d homography = k * plane * to_plane;
        cv::Mat image(480, 640, CV_8UC1, cv::Scalar(140));
        cv::warpPerspective(texture, image, cv::Mat(homography), image.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
        return image;
    }

    double angle_between(const cv::Vec3d& a, const cv::Vec3d& b)
    {
        cv::Matx33d ra;
        cv::Matx33d rb;
        cv::Rodrigues(a, ra);
        cv::Rodrigues(b, rb);
        const cv::Matx33d difference = ra.t() * rb;
        const double trace = difference(0, 0) + difference(1, 1) + difference(2, 2);
        return std::acos(std::clamp((trace - 1.0) / 2.0, -1.0, 1.0));
    }
}

TEST(TagDetection, FindsAndLocatesATagHeadOn)
{
    // Tag facing the camera (its z towards the camera = camera -z): rotate pi about x.
    const cv::Vec3d rvec(M_PI, 0.0, 0.0);
    const cv::Vec3d tvec(0.1, -0.05, 1.5);
    const auto tags = detect_tags(render_scene(7, rvec, tvec), CAMERA, DetectorParams{}, [](int)
                                  { return SIZE; });
    ASSERT_EQ(tags.size(), 1U);
    EXPECT_EQ(tags[0].id, 7);
    // Position is reliable head-on...
    EXPECT_NEAR(tags[0].tvec[0], tvec[0], 0.01);
    EXPECT_NEAR(tags[0].tvec[1], tvec[1], 0.01);
    EXPECT_NEAR(tags[0].tvec[2], tvec[2], 0.03);
    EXPECT_LT(tags[0].reprojection_error_px, 1.0);
    // ...orientation is the ambiguous part, and the ratio is reported for gating.
    EXPECT_GE(tags[0].ambiguity_ratio, 1.0);
}

TEST(TagDetection, LocatesAnObliqueTagAndItsFacing)
{
    // Turned 30 degrees about its vertical axis, 1.5 m away: unambiguous.
    cv::Matx33d facing;
    cv::Rodrigues(cv::Vec3d(M_PI, 0.0, 0.0), facing);
    cv::Matx33d turn;
    cv::Rodrigues(cv::Vec3d(0.0, 30.0 * M_PI / 180.0, 0.0), turn);
    cv::Vec3d rvec;
    cv::Rodrigues(cv::Mat(turn * facing), rvec);
    // 0.1 m off the optical axis (an exactly centred, symmetric render is a degenerate
    // test scene), so the viewing angle is the 30 degree turn less the ray's 3.8 degrees.
    const cv::Vec3d tvec(0.1, 0.0, 1.5);
    const auto tags = detect_tags(render_scene(3, rvec, tvec), CAMERA, DetectorParams{}, [](int)
                                  { return SIZE; });
    ASSERT_EQ(tags.size(), 1U);
    EXPECT_EQ(tags[0].id, 3);
    EXPECT_NEAR(cv::norm(tags[0].tvec - tvec), 0.0, 0.03);
    EXPECT_GE(tags[0].ambiguity_ratio, 3.0);
    EXPECT_LT(angle_between(tags[0].rvec, rvec), 3.0 * M_PI / 180.0);
    EXPECT_NEAR(tags[0].view_angle_rad, (30.0 - std::atan2(0.1, 1.5) * 180.0 / M_PI) * M_PI / 180.0, 2.0 * M_PI / 180.0);
}

TEST(TagDetection, AllRefinementModesDetect)
{
    const cv::Mat image = render_scene(5, {M_PI, 0.0, 0.0}, {0.0, 0.0, 1.0});
    for (const char* mode : {"subpix", "apriltag", "none"})
    {
        DetectorParams params;
        params.corner_refinement = mode;
        EXPECT_EQ(detect_tags(image, CAMERA, params, [](int)
                              { return SIZE; })
                      .size(),
                  1U)
            << mode;
    }
    DetectorParams bad;
    bad.corner_refinement = "magic";
    EXPECT_THROW(detect_tags(image, CAMERA, bad, [](int)
                             { return SIZE; }),
                 std::invalid_argument);
}

TEST(TagDetection, UsesThePerIdSize)
{
    // The same image read with twice the size must give twice the distance.
    const cv::Vec3d rvec(M_PI, 0.0, 0.0);
    const cv::Mat image = render_scene(1, rvec, {0.0, 0.0, 1.0});
    const auto normal = detect_tags(image, CAMERA, DetectorParams{}, [](int)
                                    { return SIZE; });
    const auto doubled = detect_tags(image, CAMERA, DetectorParams{}, [](int)
                                     { return 2.0 * SIZE; });
    ASSERT_EQ(normal.size(), 1U);
    ASSERT_EQ(doubled.size(), 1U);
    EXPECT_NEAR(doubled[0].distance_m, 2.0 * normal[0].distance_m, 0.02);
}

TEST(TagDetection, EmptySceneAndBadInput)
{
    const cv::Mat blank(480, 640, CV_8UC1, cv::Scalar(128));
    EXPECT_TRUE(detect_tags(blank, CAMERA, DetectorParams{}, [](int)
                            { return SIZE; })
                    .empty());
    const cv::Mat colour(480, 640, CV_8UC3);
    EXPECT_THROW(detect_tags(colour, CAMERA, DetectorParams{}, [](int)
                             { return SIZE; }),
                 std::invalid_argument);
    DetectorParams bad;
    bad.family = "48h12";
    EXPECT_THROW(detect_tags(blank, CAMERA, bad, [](int)
                             { return SIZE; }),
                 std::invalid_argument);
}
