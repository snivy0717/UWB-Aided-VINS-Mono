#include "uwb_triplet_manager.h"

#include <algorithm>
#include <cmath>

void UWBTripletManager::setMinRange(double min_range)
{
    std::lock_guard<std::mutex> lock(mutex_);
    uwb_min_range_ = min_range > 0.0 ? min_range : 0.2;
}

void UWBTripletManager::setMeanFilterWindowSize(int window_size)
{
    std::lock_guard<std::mutex> lock(mutex_);
    mean_filter_window_size_ = window_size > 0 ? window_size : 4;
    while (static_cast<int>(mean_window_.size()) > mean_filter_window_size_)
    {
        window_sum_ -= mean_window_.front();
        mean_window_.pop_front();
    }
}

void UWBTripletManager::setInterpolationWindowSize(int window_size)
{
    std::lock_guard<std::mutex> lock(mutex_);
    interp_window_size_ = window_size > 0 ? std::min(window_size, 4) : 4;
    while (static_cast<int>(uwb_buffer_.size()) > interp_window_size_)
        uwb_buffer_.pop_front();
}

void UWBTripletManager::clear()
{
    std::lock_guard<std::mutex> lock(mutex_);
    pending_triplets_.clear();
    mean_window_.clear();
    window_sum_.setZero();
    uwb_buffer_.clear();
    first_time_initialized_ = false;
    first_time_ = 0.0;
}

bool UWBTripletManager::addRangeMeasurement(const UWBMeasurement &measurement,
                                            UWBTriplet *accepted_triplet,
                                            UWBTriplet *filtered_triplet)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (measurement.anchor_id < 0 || measurement.anchor_id > 2)
        return false;

    PendingTriplet &pending = pending_triplets_[measurement.timestamp];
    pending.ranges[measurement.anchor_id] = measurement.range;
    pending.has_anchor[measurement.anchor_id] = true;
    pruneOldPartials(measurement.timestamp);

    if (!isComplete(pending))
        return false;

    UWBTriplet raw_triplet;
    raw_triplet.timestamp = measurement.timestamp;
    raw_triplet.ranges << pending.ranges[0], pending.ranges[1], pending.ranges[2];
    pending_triplets_.erase(measurement.timestamp);

    if (!isValidTriplet(raw_triplet.ranges))
        return false;

    UWBTriplet filtered = raw_triplet;
    filtered.ranges = meanFilter(raw_triplet.ranges);
    inputUWB(filtered);

    if (accepted_triplet)
        *accepted_triplet = raw_triplet;
    if (filtered_triplet)
        *filtered_triplet = filtered;
    return true;
}

bool UWBTripletManager::processUWBAt(double vio_time, UWBTriplet &aligned_triplet) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!first_time_initialized_ || static_cast<int>(uwb_buffer_.size()) != interp_window_size_ ||
        interp_window_size_ != 4)
        return false;

    const double query_time = vio_time - first_time_;
    std::array<Eigen::Vector2d, 4> d0;
    std::array<Eigen::Vector2d, 4> d1;
    std::array<Eigen::Vector2d, 4> d2;

    for (int i = 0; i < 4; ++i)
    {
        d0[i] = Eigen::Vector2d(uwb_buffer_[i].timestamp, uwb_buffer_[i].ranges[0]);
        d1[i] = Eigen::Vector2d(uwb_buffer_[i].timestamp, uwb_buffer_[i].ranges[1]);
        d2[i] = Eigen::Vector2d(uwb_buffer_[i].timestamp, uwb_buffer_[i].ranges[2]);
    }

    double range0 = 0.0;
    double range1 = 0.0;
    double range2 = 0.0;
    if (!interpolateCubic(d0, query_time, range0) ||
        !interpolateCubic(d1, query_time, range1) ||
        !interpolateCubic(d2, query_time, range2))
    {
        return false;
    }

    aligned_triplet.timestamp = vio_time;
    aligned_triplet.ranges << range0, range1, range2;
    return isValidTriplet(aligned_triplet.ranges);
}

size_t UWBTripletManager::tripletBufferSize() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return uwb_buffer_.size();
}

size_t UWBTripletManager::partialBufferSize() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_triplets_.size();
}

bool UWBTripletManager::isComplete(const PendingTriplet &pending) const
{
    return pending.has_anchor[0] && pending.has_anchor[1] && pending.has_anchor[2];
}

bool UWBTripletManager::isValidTriplet(const Eigen::Vector3d &ranges) const
{
    for (int i = 0; i < 3; ++i)
    {
        if (!std::isfinite(ranges[i]) || ranges[i] <= uwb_min_range_)
            return false;
    }
    return true;
}

Eigen::Vector3d UWBTripletManager::meanFilter(const Eigen::Vector3d &ranges)
{
    mean_window_.push_back(ranges);
    window_sum_ += ranges;

    while (static_cast<int>(mean_window_.size()) > mean_filter_window_size_)
    {
        window_sum_ -= mean_window_.front();
        mean_window_.pop_front();
    }

    return window_sum_ / static_cast<double>(mean_window_.size());
}

void UWBTripletManager::inputUWB(const UWBTriplet &triplet)
{
    if (!first_time_initialized_)
    {
        first_time_ = triplet.timestamp;
        first_time_initialized_ = true;
    }

    UWBTriplet relative_triplet = triplet;
    relative_triplet.timestamp = triplet.timestamp - first_time_;
    uwb_buffer_.push_back(relative_triplet);

    while (static_cast<int>(uwb_buffer_.size()) > interp_window_size_)
        uwb_buffer_.pop_front();
}

bool UWBTripletManager::interpolateCubic(const std::array<Eigen::Vector2d, 4> &samples,
                                         double query_time,
                                         double &value) const
{
    Eigen::Matrix4d align_matrix;
    Eigen::Vector4d align_vector;

    for (int i = 0; i < 4; ++i)
    {
        const double t = samples[i][0];
        align_matrix(i, 0) = t * t * t;
        align_matrix(i, 1) = t * t;
        align_matrix(i, 2) = t;
        align_matrix(i, 3) = 1.0;
        align_vector[i] = samples[i][1];
    }

    if (std::fabs(align_matrix.determinant()) < 1e-12)
        return false;

    const Eigen::Vector4d para = align_matrix.inverse() * align_vector;
    value = para[0] * query_time * query_time * query_time +
            para[1] * query_time * query_time +
            para[2] * query_time +
            para[3];

    return std::isfinite(value);
}

void UWBTripletManager::pruneOldPartials(double latest_timestamp)
{
    const double oldest_timestamp = latest_timestamp - 1.0;
    auto it = pending_triplets_.begin();
    while (it != pending_triplets_.end())
    {
        if (it->first < oldest_timestamp)
            it = pending_triplets_.erase(it);
        else
            ++it;
    }
}
