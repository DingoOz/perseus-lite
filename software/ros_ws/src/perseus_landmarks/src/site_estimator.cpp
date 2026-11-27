#include "perseus_landmarks/site_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <map>

namespace perseus_landmarks
{
    namespace
    {
        /// @brief Per-tag weighted mean of its sightings.
        struct TagMean
        {
            Point2D map;
            Point2D site;
            double facing_map{0.0};
            double facing_site{0.0};
            double weight{0.0};
            double facing_weight{0.0};
        };
    }

    std::optional<SiteFix> solve_site_fix(const std::vector<TagObservation>& observations, const LandmarkMap& landmarks,
                                          double min_tag_separation_m)
    {
        // 1. Average each tag's sightings (positions arithmetically, facings on the circle).
        std::map<int, TagMean> means;
        std::map<int, std::pair<double, double>> facing_vectors;
        for (const TagObservation& observation : observations)
        {
            const auto known = landmarks.tags.find(observation.id);
            if (known == landmarks.tags.end() || observation.weight <= 0.0)
            {
                continue;
            }
            TagMean& mean = means[observation.id];
            mean.map.x += observation.weight * observation.position.x;
            mean.map.y += observation.weight * observation.position.y;
            mean.weight += observation.weight;
            mean.site = {known->second.x, known->second.y};
            mean.facing_site = known->second.yaw;
            auto& vector = facing_vectors[observation.id];
            vector.first += observation.facing_weight * std::cos(observation.facing);
            vector.second += observation.facing_weight * std::sin(observation.facing);
            mean.facing_weight += observation.facing_weight;
        }
        if (means.empty())
        {
            return std::nullopt;
        }
        double total_weight = 0.0;
        for (auto& [id, mean] : means)
        {
            mean.map.x /= mean.weight;
            mean.map.y /= mean.weight;
            mean.facing_map = std::atan2(facing_vectors[id].second, facing_vectors[id].first);
            total_weight += mean.weight;
        }

        // 2. Weighted centroids in both frames.
        Point2D centroid_map;
        Point2D centroid_site;
        for (const auto& [id, mean] : means)
        {
            const double w = mean.weight / total_weight;
            centroid_map.x += w * mean.map.x;
            centroid_map.y += w * mean.map.y;
            centroid_site.x += w * mean.site.x;
            centroid_site.y += w * mean.site.y;
        }

        // 3. Yaw: from the position fit when the tags are spread out enough to define it,
        //    otherwise from the tags' facing directions.
        double cross = 0.0;
        double dot = 0.0;
        double spread = 0.0;
        for (const auto& [id, mean] : means)
        {
            const double w = mean.weight / total_weight;
            const double mx = mean.map.x - centroid_map.x;
            const double my = mean.map.y - centroid_map.y;
            const double sx = mean.site.x - centroid_site.x;
            const double sy = mean.site.y - centroid_site.y;
            dot += w * (mx * sx + my * sy);
            cross += w * (mx * sy - my * sx);
            spread = std::max(spread, std::hypot(sx, sy));
        }
        SiteFix fix;
        // The farthest tag from the centroid is at least half the largest pairwise distance.
        fix.multi_tag = means.size() >= 2 && 2.0 * spread >= min_tag_separation_m;
        double yaw = 0.0;
        if (fix.multi_tag)
        {
            yaw = std::atan2(cross, dot);
        }
        else
        {
            double sum_cos = 0.0;
            double sum_sin = 0.0;
            double facing_total = 0.0;
            for (const auto& [id, mean] : means)
            {
                if (mean.facing_weight <= 0.0)
                {
                    continue;
                }
                const double difference = wrap_angle(mean.facing_site - mean.facing_map);
                sum_cos += mean.facing_weight * std::cos(difference);
                sum_sin += mean.facing_weight * std::sin(difference);
                facing_total += mean.facing_weight;
            }
            if (facing_total <= 0.0)
            {
                return std::nullopt;  // position only: the yaw is unobservable
            }
            yaw = std::atan2(sum_sin, sum_cos);
        }

        // 4. Translation maps the rotated map centroid onto the site centroid.
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        fix.site_from_map = {centroid_site.x - (c * centroid_map.x - s * centroid_map.y),
                             centroid_site.y - (s * centroid_map.x + c * centroid_map.y), wrap_angle(yaw)};

        // 5. Residual over tags.
        double squared = 0.0;
        for (const auto& [id, mean] : means)
        {
            const Point2D predicted = fix.site_from_map.apply(mean.map);
            squared += (mean.weight / total_weight) *
                       ((predicted.x - mean.site.x) * (predicted.x - mean.site.x) +
                        (predicted.y - mean.site.y) * (predicted.y - mean.site.y));
            fix.tags.push_back(id);
        }
        fix.residual_m = std::sqrt(squared);
        return fix;
    }

    SiteEstimator::SiteEstimator(LandmarkMap landmarks, EstimatorParams params)
        : landmarks_(std::move(landmarks)),
          params_(params)
    {
    }

    void SiteEstimator::add(const TagObservation& observation)
    {
        if (landmarks_.tags.find(observation.id) == landmarks_.tags.end())
        {
            return;
        }
        window_.push_back(observation);
    }

    UpdateResult SiteEstimator::update(double now_s)
    {
        while (!window_.empty() && now_s - window_.front().stamp_s > params_.window_s)
        {
            window_.pop_front();
        }
        if (static_cast<int>(window_.size()) < std::max(1, params_.min_observations))
        {
            return UpdateResult::NOTHING_TO_DO;
        }
        const auto fix = solve_site_fix({window_.begin(), window_.end()}, landmarks_, params_.min_tag_separation_m);
        if (!fix)
        {
            return UpdateResult::NOTHING_TO_DO;
        }
        if (fix->residual_m > params_.max_residual_m)
        {
            return UpdateResult::REJECTED_RESIDUAL;
        }
        if (!estimate_)
        {
            estimate_ = fix->site_from_map;
            last_fix_ = fix;
            disagreeing_since_s_.reset();
            return UpdateResult::ACCEPTED;
        }

        // Compare where the two transforms put the robot's surroundings: evaluate both at
        // the observed tags' map centroid, which is where the fix is best constrained.
        Point2D centroid;
        for (const TagObservation& observation : window_)
        {
            centroid.x += observation.position.x / static_cast<double>(window_.size());
            centroid.y += observation.position.y / static_cast<double>(window_.size());
        }
        const Point2D by_estimate = estimate_->apply(centroid);
        const Point2D by_fix = fix->site_from_map.apply(centroid);
        const double jump_m = std::hypot(by_fix.x - by_estimate.x, by_fix.y - by_estimate.y);
        const double jump_yaw = std::abs(wrap_angle(fix->site_from_map.yaw - estimate_->yaw));
        if (jump_m > params_.max_jump_m || jump_yaw > params_.max_jump_yaw_rad)
        {
            if (!disagreeing_since_s_)
            {
                disagreeing_since_s_ = now_s;
            }
            if (now_s - *disagreeing_since_s_ < params_.jump_confirm_s)
            {
                return UpdateResult::REJECTED_JUMP;
            }
            estimate_ = fix->site_from_map;
            last_fix_ = fix;
            disagreeing_since_s_.reset();
            return UpdateResult::ACCEPTED_JUMP;
        }
        disagreeing_since_s_.reset();

        // Blend: translation linearly (evaluated at the centroid, so blending does not
        // swing the origin around), yaw on the circle.
        const double alpha = std::clamp(params_.blend_alpha, 0.0, 1.0);
        const double yaw = wrap_angle(estimate_->yaw + alpha * wrap_angle(fix->site_from_map.yaw - estimate_->yaw));
        const Point2D target{by_estimate.x + alpha * (by_fix.x - by_estimate.x),
                             by_estimate.y + alpha * (by_fix.y - by_estimate.y)};
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        estimate_ = Pose2D{target.x - (c * centroid.x - s * centroid.y), target.y - (s * centroid.x + c * centroid.y), yaw};
        last_fix_ = fix;
        return UpdateResult::ACCEPTED;
    }

    void SiteEstimator::reset()
    {
        window_.clear();
        estimate_.reset();
        last_fix_.reset();
        disagreeing_since_s_.reset();
    }

    void SiteEstimator::seed(const SiteFix& fix)
    {
        estimate_ = fix.site_from_map;
        last_fix_ = fix;
        disagreeing_since_s_.reset();
    }

    std::size_t SiteEstimator::window_size() const
    {
        return window_.size();
    }

    const std::optional<Pose2D>& SiteEstimator::estimate() const
    {
        return estimate_;
    }

    const std::optional<SiteFix>& SiteEstimator::last_fix() const
    {
        return last_fix_;
    }

    const LandmarkMap& SiteEstimator::landmarks() const
    {
        return landmarks_;
    }

}
