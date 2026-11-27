#include "perseus_landmarks/site_estimator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>

using namespace perseus_landmarks;

namespace
{
    constexpr double DEG = M_PI / 180.0;

    LandmarkMap two_tag_map()
    {
        return parse_landmark_map(R"(
site_frame: site
tags:
  - {id: 0, pose: [2.0, 0.0, 0.25, 180.0]}
  - {id: 1, pose: [0.0, 3.0, 0.25, -90.0]}
destinations:
  - {name: dock, pose: [1.0, 1.0, 90.0], tag: 1}
search_points:
  - {xy: [1.0, 0.0]}
  - {xy: [0.0, 2.0], frame: site}
)");
    }

    /// @brief What a sighting of @p tag looks like in the map, if site_from_map is truth.
    TagObservation observe(const LandmarkMap& map, int id, const Pose2D& site_from_map, double stamp, double noise_m = 0.0,
                           double noise_yaw = 0.0)
    {
        const TagLandmark& tag = map.tags.at(id);
        const Pose2D map_from_site = site_from_map.inverse();
        const Point2D position = map_from_site.apply({tag.x + noise_m, tag.y - noise_m});
        TagObservation observation;
        observation.id = id;
        observation.position = position;
        observation.facing = wrap_angle(tag.yaw + map_from_site.yaw + noise_yaw);
        observation.stamp_s = stamp;
        return observation;
    }
}

TEST(Pose2D, ComposeInverseApply)
{
    const Pose2D a{1.0, 2.0, 0.5};
    const Pose2D identity = a.compose(a.inverse());
    EXPECT_NEAR(identity.x, 0.0, 1e-12);
    EXPECT_NEAR(identity.y, 0.0, 1e-12);
    EXPECT_NEAR(identity.yaw, 0.0, 1e-12);
    const Point2D p = a.apply({1.0, 0.0});
    EXPECT_NEAR(p.x, 1.0 + std::cos(0.5), 1e-12);
    EXPECT_NEAR(wrap_angle(3.0 * M_PI), M_PI, 1e-12);
    EXPECT_NEAR(wrap_angle(-M_PI), M_PI, 1e-12);
}

TEST(LandmarkMap, ParsesAndRejects)
{
    const LandmarkMap map = two_tag_map();
    EXPECT_EQ(map.tags.size(), 2U);
    EXPECT_NEAR(map.tags.at(0).yaw, M_PI, 1e-9);
    EXPECT_NEAR(map.tags.at(0).size_m, 0.16, 1e-12);
    ASSERT_EQ(map.destinations.size(), 1U);
    EXPECT_EQ(*map.destinations[0].tag, 1);
    EXPECT_EQ(map.search_points[0].frame, "start");
    EXPECT_EQ(map.search_points[1].frame, "site");
    EXPECT_NEAR(map.size_of(42), map.default_tag_size_m, 1e-12);
    EXPECT_THROW(parse_landmark_map("tags: [{id: 0, pose: [0, 0, 0, 0]}, {id: 0, pose: [1, 0, 0, 0]}]"), std::runtime_error);
    EXPECT_THROW(parse_landmark_map("tags: [{id: 0, pose: [0, 0, 0]}]"), std::runtime_error);
    EXPECT_THROW(parse_landmark_map("destinations: [{name: a, pose: [0, 0, 0], tag: 9}]"), std::runtime_error);
    EXPECT_THROW(parse_landmark_map("search_points: [{xy: [0, 0], frame: moon}]"), std::runtime_error);
    EXPECT_THROW(parse_landmark_map("tags: [unclosed"), std::runtime_error);
}

TEST(SolveSiteFix, SingleTagRecoversTheTransform)
{
    const LandmarkMap map = two_tag_map();
    const Pose2D truth{0.7, -1.2, 0.4};
    const auto fix = solve_site_fix({observe(map, 0, truth, 0.0)}, map, 0.5);
    ASSERT_TRUE(fix);
    EXPECT_FALSE(fix->multi_tag);
    EXPECT_NEAR(fix->site_from_map.x, truth.x, 1e-9);
    EXPECT_NEAR(fix->site_from_map.y, truth.y, 1e-9);
    EXPECT_NEAR(fix->site_from_map.yaw, truth.yaw, 1e-9);
    EXPECT_NEAR(fix->residual_m, 0.0, 1e-9);
}

TEST(SolveSiteFix, TwoTagsUsePositionsNotFacings)
{
    const LandmarkMap map = two_tag_map();
    const Pose2D truth{-2.0, 0.5, -1.0};
    // Facings off by 10 degrees each: a position fit must not care.
    const auto fix = solve_site_fix({observe(map, 0, truth, 0.0, 0.0, 10 * DEG), observe(map, 1, truth, 0.0, 0.0, -10 * DEG)},
                                    map, 0.5);
    ASSERT_TRUE(fix);
    EXPECT_TRUE(fix->multi_tag);
    EXPECT_NEAR(fix->site_from_map.yaw, truth.yaw, 1e-9);
    EXPECT_NEAR(fix->site_from_map.x, truth.x, 1e-9);
    EXPECT_NEAR(fix->site_from_map.y, truth.y, 1e-9);
}

TEST(SolveSiteFix, SingleTagWithOnlyAmbiguousFacingsGivesNoFix)
{
    const LandmarkMap map = two_tag_map();
    TagObservation ambiguous = observe(map, 0, {0.5, 0.5, 0.2}, 0.0);
    ambiguous.facing_weight = 0.0;
    EXPECT_FALSE(solve_site_fix({ambiguous, ambiguous}, map, 0.5));
    // Two tags need no facing at all.
    TagObservation other = observe(map, 1, {0.5, 0.5, 0.2}, 0.0);
    other.facing_weight = 0.0;
    const auto fix = solve_site_fix({ambiguous, other}, map, 0.5);
    ASSERT_TRUE(fix);
    EXPECT_NEAR(fix->site_from_map.yaw, 0.2, 1e-9);
}

TEST(SolveSiteFix, IgnoresUnknownTags)
{
    const LandmarkMap map = two_tag_map();
    TagObservation unknown;
    unknown.id = 99;
    EXPECT_FALSE(solve_site_fix({unknown}, map, 0.5));
}

TEST(SiteEstimator, NeedsEnoughObservationsThenAccepts)
{
    const LandmarkMap map = two_tag_map();
    EstimatorParams params;
    params.min_observations = 3;
    SiteEstimator estimator(map, params);
    const Pose2D truth{1.0, 2.0, 0.3};
    estimator.add(observe(map, 0, truth, 0.0));
    EXPECT_EQ(estimator.update(0.0), UpdateResult::NOTHING_TO_DO);
    estimator.add(observe(map, 0, truth, 0.1));
    estimator.add(observe(map, 0, truth, 0.2));
    EXPECT_EQ(estimator.update(0.2), UpdateResult::ACCEPTED);
    ASSERT_TRUE(estimator.estimate());
    EXPECT_NEAR(estimator.estimate()->x, 1.0, 1e-9);
}

TEST(SiteEstimator, AveragesNoise)
{
    const LandmarkMap map = two_tag_map();
    EstimatorParams params;
    params.min_observations = 20;
    params.window_s = 10.0;
    SiteEstimator estimator(map, params);
    const Pose2D truth{0.0, 0.0, 0.0};
    std::mt19937 generator(3);
    std::normal_distribution<double> metres(0.0, 0.03);
    std::normal_distribution<double> radians(0.0, 4.0 * DEG);
    for (int i = 0; i < 40; ++i)
    {
        estimator.add(observe(map, 0, truth, 0.1 * i, metres(generator), radians(generator)));
    }
    ASSERT_EQ(estimator.update(4.0), UpdateResult::ACCEPTED);
    EXPECT_NEAR(estimator.estimate()->yaw, 0.0, 2.0 * DEG);
}

TEST(SiteEstimator, RejectsASingleJumpButAcceptsAPersistentOne)
{
    const LandmarkMap map = two_tag_map();
    EstimatorParams params;
    params.min_observations = 1;
    params.window_s = 0.5;
    params.jump_confirm_s = 2.0;
    SiteEstimator estimator(map, params);
    const Pose2D first{0.0, 0.0, 0.0};
    estimator.add(observe(map, 0, first, 0.0));
    ASSERT_EQ(estimator.update(0.0), UpdateResult::ACCEPTED);
    const Pose2D moved{1.0, 0.0, 0.0};
    estimator.add(observe(map, 0, moved, 1.0));
    EXPECT_EQ(estimator.update(1.0), UpdateResult::REJECTED_JUMP);
    EXPECT_NEAR(estimator.estimate()->x, 0.0, 1e-9);
    for (double t = 1.5; t < 3.0; t += 0.5)
    {
        estimator.add(observe(map, 0, moved, t));
        EXPECT_EQ(estimator.update(t), UpdateResult::REJECTED_JUMP);
    }
    estimator.add(observe(map, 0, moved, 3.1));
    EXPECT_EQ(estimator.update(3.1), UpdateResult::ACCEPTED_JUMP);
    EXPECT_NEAR(estimator.estimate()->x, 1.0, 1e-9);
}

TEST(SiteEstimator, BlendsSmallCorrections)
{
    const LandmarkMap map = two_tag_map();
    EstimatorParams params;
    params.min_observations = 1;
    params.window_s = 0.1;
    params.blend_alpha = 0.5;
    SiteEstimator estimator(map, params);
    estimator.add(observe(map, 0, {0.0, 0.0, 0.0}, 0.0));
    estimator.update(0.0);
    estimator.add(observe(map, 0, {0.1, 0.0, 0.0}, 1.0));
    EXPECT_EQ(estimator.update(1.0), UpdateResult::ACCEPTED);
    EXPECT_NEAR(estimator.estimate()->x, 0.05, 1e-6);
}

TEST(SiteEstimator, RejectsInconsistentTags)
{
    // Two tags whose sightings cannot both be right (one has moved 1 m).
    const LandmarkMap map = two_tag_map();
    EstimatorParams params;
    params.min_observations = 2;
    SiteEstimator estimator(map, params);
    TagObservation moved = observe(map, 1, {}, 0.0);
    moved.position.x += 1.0;
    estimator.add(observe(map, 0, {}, 0.0));
    estimator.add(moved);
    EXPECT_EQ(estimator.update(0.0), UpdateResult::REJECTED_RESIDUAL);
    EXPECT_FALSE(estimator.estimate());
    estimator.reset();
    EXPECT_EQ(estimator.window_size(), 0U);
}
