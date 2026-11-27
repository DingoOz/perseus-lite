#pragma once

/// @file site_estimator.hpp
/// @brief Estimates the site -> map transform from sightings of tags at known places.
///
/// Each sighting gives a tag's position and facing direction in the SLAM map frame;
/// the landmark map gives the same tag in the site frame. One transform maps the first
/// onto the second.
///
/// - One distinct tag in the window: the yaw comes from the tag's facing direction,
///   the translation from its position. A single tag's orientation from PnP is the
///   weak part: a square seen nearly head-on fits two mirror-tilted poses, so only
///   sightings flagged unambiguous contribute a facing (facing_weight > 0), and they
///   are averaged. With no such sighting a single tag gives no fix at all.
/// - Two or more distinct tags at least min_tag_separation_m apart: a weighted 2D
///   rigid fit of positions only (closed-form Kabsch). Positions are far more accurate
///   than single-tag orientation, so this is preferred whenever possible.
///
/// Fixes are gated (residual) and blended into the current estimate. A fix that
/// disagrees by more than the jump limits is only accepted after it has persisted for
/// jump_confirm_s, so one spurious sighting cannot yank the site frame, but a real
/// change (SLAM drift, a relocated tag) eventually wins.

#include <deque>
#include <optional>
#include <vector>

#include "perseus_landmarks/landmark_map.hpp"
#include "perseus_landmarks/pose2d.hpp"

namespace perseus_landmarks
{
    /// @brief One sighting of a known tag, already expressed in the map frame.
    struct TagObservation
    {
        int id{0};
        /// @brief Tag centre in the map frame, metres.
        Point2D position;
        /// @brief Tag's facing direction (normal projected onto the ground plane) in the
        ///        map frame, radians.
        double facing{0.0};
        /// @brief Relative trust in the position (> 0).
        double weight{1.0};
        /// @brief Relative trust in the facing; 0 when the tag's orientation was
        ///        ambiguous (the position is still used).
        double facing_weight{1.0};
        double stamp_s{0.0};
    };

    /// @brief A solved transform and how well it fits.
    struct SiteFix
    {
        /// @brief site_from_map: p_site = apply(p_map).
        Pose2D site_from_map;
        /// @brief RMS distance between known tag positions and the fitted observed ones.
        double residual_m{0.0};
        std::vector<int> tags;
        /// @brief Whether the yaw came from a multi-tag position fit.
        bool multi_tag{false};
    };

    /// @brief Solves site_from_map from a set of observations of known tags.
    /// @return nullopt if no observation refers to a tag in @p landmarks, or if there is
    ///         only one tag and none of its sightings has a usable facing.
    std::optional<SiteFix> solve_site_fix(const std::vector<TagObservation>& observations, const LandmarkMap& landmarks,
                                          double min_tag_separation_m);

    /// @brief Tunables of SiteEstimator.
    struct EstimatorParams
    {
        /// @brief Observations older than this (relative to the newest) are dropped.
        double window_s{2.0};
        /// @brief Sightings needed in the window before solving.
        int min_observations{5};
        /// @brief Fixes with a larger residual are rejected.
        double max_residual_m{0.15};
        /// @brief Two tags closer than this cannot define a yaw by their positions.
        double min_tag_separation_m{0.5};
        /// @brief Weight of a new fix when blending into the estimate (0..1].
        double blend_alpha{0.3};
        /// @brief Disagreements beyond these are jumps.
        double max_jump_m{0.3};
        double max_jump_yaw_rad{0.15};
        /// @brief A jump must persist this long before it replaces the estimate.
        double jump_confirm_s{3.0};
    };

    /// @brief What SiteEstimator::update() did.
    enum class UpdateResult
    {
        NOTHING_TO_DO,
        ACCEPTED,
        ACCEPTED_JUMP,
        REJECTED_RESIDUAL,
        REJECTED_JUMP,
    };

    /// @brief See the file comment.
    class SiteEstimator
    {
    public:
        SiteEstimator(LandmarkMap landmarks, EstimatorParams params);

        /// @brief Adds a sighting. Unknown tag ids are ignored.
        void add(const TagObservation& observation);

        /// @brief Solves on the current window and folds the result into the estimate.
        UpdateResult update(double now_s);

        /// @brief Drops the estimate and every observation: the next accepted fix
        ///        replaces rather than blends (used for a fresh calibration).
        void reset();

        /// @brief Replaces the estimate with @p fix (e.g. from a separate calibration
        ///        estimator), keeping the observation window.
        void seed(const SiteFix& fix);

        /// @brief Observations currently in the window.
        std::size_t window_size() const;

        const std::optional<Pose2D>& estimate() const;
        /// @brief The last fix that was accepted (its residual and tags).
        const std::optional<SiteFix>& last_fix() const;
        const LandmarkMap& landmarks() const;

    private:
        LandmarkMap landmarks_;
        EstimatorParams params_;
        std::deque<TagObservation> window_;
        std::optional<Pose2D> estimate_;
        std::optional<SiteFix> last_fix_;
        std::optional<double> disagreeing_since_s_;
    };

}
