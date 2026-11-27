#include "perseus_monitoring/slip_detector.hpp"

#include <algorithm>
#include <cmath>

namespace perseus_monitoring
{
    SlipDetector::SlipDetector(SlipDetectorConfig config)
        : config_(config)
    {
    }

    void SlipDetector::reset()
    {
        samples_.clear();
        command_active_ = false;
        command_sign_ = 0.0;
        ratio_ = 0.0;
        evaluating_ = false;
    }

    void SlipDetector::update(double commanded, double measured, double now_s)
    {
        commanded_ = commanded;
        measured_ = measured;
        if (std::abs(commanded) < config_.min_command)
        {
            reset();
            return;
        }
        const double sign = commanded > 0.0 ? 1.0 : -1.0;
        if (!command_active_ || sign != command_sign_)
        {
            // A new command, or a reversal: the robot must decelerate through zero
            // first, which is not slip. Start the grace period again.
            samples_.clear();
            command_active_ = true;
            command_sign_ = sign;
            active_since_s_ = now_s;
        }
        if (now_s - active_since_s_ < config_.grace_s)
        {
            evaluating_ = false;
            ratio_ = 0.0;
            return;
        }

        // Clamped so one wild sample (a wheel-odometry spike) cannot dominate the mean.
        const double shortfall = std::clamp(1.0 - measured / commanded, -1.0, 2.0);
        samples_.push_back({now_s, shortfall});
        while (!samples_.empty() && now_s - samples_.front().time_s > config_.window_s)
        {
            samples_.pop_front();
        }
        // Need the window at least half full before judging.
        const double span = samples_.back().time_s - samples_.front().time_s;
        evaluating_ = samples_.size() >= 3 && span >= 0.5 * config_.window_s;
        if (!evaluating_)
        {
            ratio_ = 0.0;
            return;
        }
        double sum = 0.0;
        for (const Sample& sample : samples_)
        {
            sum += sample.ratio;
        }
        ratio_ = sum / static_cast<double>(samples_.size());
    }

    bool SlipDetector::evaluating() const
    {
        return evaluating_;
    }

    bool SlipDetector::fault() const
    {
        return evaluating_ && ratio_ >= config_.fault_ratio;
    }

    double SlipDetector::ratio() const
    {
        return ratio_;
    }

    double SlipDetector::commanded() const
    {
        return commanded_;
    }

    double SlipDetector::measured() const
    {
        return measured_;
    }

}
