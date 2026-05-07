#include "uwb_manager.h"

#include <cmath>
#include <limits>

void UWBManager::setMaxInterval(double max_interval)
{
    std::lock_guard<std::mutex> lock(mutex_);
    max_interval_ = max_interval > 0.0 ? max_interval : 0.05;
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
