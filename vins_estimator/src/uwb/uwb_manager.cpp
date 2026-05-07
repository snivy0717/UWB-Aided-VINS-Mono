#include "uwb_manager.h"

#include <cmath>
#include <limits>

void UWBManager::setMaxInterval(double max_interval)
{
    std::lock_guard<std::mutex> lock(mutex_);
    max_interval_ = max_interval > 0.0 ? max_interval : 0.05;
}

void UWBManager::setInterpolationMaxGap(double max_gap)
{
    std::lock_guard<std::mutex> lock(mutex_);
    interpolation_max_gap_ = max_gap > 0.0 ? max_gap : 0.2;
}

void UWBManager::setBufferDuration(double buffer_duration)
{
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_duration_ = buffer_duration > 0.0 ? buffer_duration : 5.0;
}

void UWBManager::clear()
{
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.clear();
}

void UWBManager::addMeasurement(const UWBMeasurement &measurement)
{
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.push_back(measurement);
    pruneOldMeasurements(measurement.timestamp);
}

std::vector<UWBMeasurement> UWBManager::getMeasurementsNear(double timestamp) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::map<int, UWBMeasurement> nearest_by_anchor;
    std::map<int, double> nearest_dt_by_anchor;

    for (const auto &measurement : buffer_)
    {
        const double dt = std::fabs(measurement.timestamp - timestamp);
        if (dt > max_interval_)
            continue;

        auto it = nearest_dt_by_anchor.find(measurement.anchor_id);
        if (it == nearest_dt_by_anchor.end() || dt < it->second)
        {
            nearest_dt_by_anchor[measurement.anchor_id] = dt;
            nearest_by_anchor[measurement.anchor_id] = measurement;
        }
    }

    std::vector<UWBMeasurement> result;
    result.reserve(nearest_by_anchor.size());
    for (const auto &item : nearest_by_anchor)
        result.push_back(item.second);

    return result;
}

bool UWBManager::getInterpolatedMeasurementsAt(double timestamp, std::vector<UWBMeasurement> &measurements) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    measurements.clear();

    std::map<int, UWBMeasurement> direct_by_anchor;
    std::map<int, double> direct_dt_by_anchor;
    std::map<int, UWBMeasurement> before_by_anchor;
    std::map<int, UWBMeasurement> after_by_anchor;

    for (const auto &measurement : buffer_)
    {
        const int anchor_id = measurement.anchor_id;
        const double dt = std::fabs(measurement.timestamp - timestamp);
        auto direct_it = direct_dt_by_anchor.find(anchor_id);
        if (dt <= max_interval_ && (direct_it == direct_dt_by_anchor.end() || dt < direct_it->second))
        {
            direct_dt_by_anchor[anchor_id] = dt;
            direct_by_anchor[anchor_id] = measurement;
        }

        if (measurement.timestamp <= timestamp)
        {
            auto before_it = before_by_anchor.find(anchor_id);
            if (before_it == before_by_anchor.end() || measurement.timestamp > before_it->second.timestamp)
                before_by_anchor[anchor_id] = measurement;
        }
        if (measurement.timestamp >= timestamp)
        {
            auto after_it = after_by_anchor.find(anchor_id);
            if (after_it == after_by_anchor.end() || measurement.timestamp < after_it->second.timestamp)
                after_by_anchor[anchor_id] = measurement;
        }
    }

    for (const auto &item : direct_by_anchor)
    {
        UWBMeasurement aligned = item.second;
        aligned.timestamp = timestamp;
        measurements.push_back(aligned);
    }

    for (const auto &before_item : before_by_anchor)
    {
        const int anchor_id = before_item.first;
        if (direct_by_anchor.find(anchor_id) != direct_by_anchor.end())
            continue;

        auto after_it = after_by_anchor.find(anchor_id);
        if (after_it == after_by_anchor.end())
            continue;

        const UWBMeasurement &before = before_item.second;
        const UWBMeasurement &after = after_it->second;
        const double time_gap = after.timestamp - before.timestamp;
        if (time_gap <= 0.0 || time_gap > interpolation_max_gap_)
            continue;

        UWBMeasurement interpolated;
        interpolated.timestamp = timestamp;
        interpolated.anchor_id = anchor_id;
        interpolated.range = before.range + (after.range - before.range) * (timestamp - before.timestamp) / time_gap;
        measurements.push_back(interpolated);
    }

    return !measurements.empty();
}

size_t UWBManager::size() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.size();
}

bool UWBManager::getNearestMeasurement(double timestamp, UWBMeasurement &measurement, double &dt) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (buffer_.empty())
        return false;

    double nearest_dt = std::numeric_limits<double>::max();
    UWBMeasurement nearest_measurement;
    for (const auto &item : buffer_)
    {
        const double item_dt = std::fabs(item.timestamp - timestamp);
        if (item_dt < nearest_dt)
        {
            nearest_dt = item_dt;
            nearest_measurement = item;
        }
    }

    measurement = nearest_measurement;
    dt = nearest_dt;
    return true;
}

void UWBManager::pruneOldMeasurements(double latest_timestamp)
{
    const double oldest_timestamp = latest_timestamp - buffer_duration_;
    while (!buffer_.empty() && buffer_.front().timestamp < oldest_timestamp)
        buffer_.pop_front();
}
