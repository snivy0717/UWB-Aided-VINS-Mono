#pragma once

#include "uwb_manager.h"

#include <array>
#include <deque>
#include <map>
#include <mutex>

#include <eigen3/Eigen/Dense>

struct UWBTriplet
{
    double timestamp = 0.0;
    Eigen::Vector3d ranges = Eigen::Vector3d::Zero();
};

class UWBTripletManager
{
  public:
    void setMinRange(double min_range);
    void setMeanFilterWindowSize(int window_size);
    void setInterpolationWindowSize(int window_size);
    void clear();

    bool addRangeMeasurement(const UWBMeasurement &measurement,
                             UWBTriplet *accepted_triplet = nullptr,
                             UWBTriplet *filtered_triplet = nullptr);
    bool processUWBAt(double vio_time, UWBTriplet &aligned_triplet) const;
    size_t tripletBufferSize() const;
    size_t partialBufferSize() const;

  private:
    struct PendingTriplet
    {
        std::array<double, 3> ranges{{0.0, 0.0, 0.0}};
        std::array<bool, 3> has_anchor{{false, false, false}};
    };

    bool isComplete(const PendingTriplet &pending) const;
    bool isValidTriplet(const Eigen::Vector3d &ranges) const;
    Eigen::Vector3d meanFilter(const Eigen::Vector3d &ranges);
    void inputUWB(const UWBTriplet &triplet);
    bool interpolateCubic(const std::array<Eigen::Vector2d, 4> &samples,
                          double query_time,
                          double &value) const;
    void pruneOldPartials(double latest_timestamp);

    mutable std::mutex mutex_;
    std::map<double, PendingTriplet> pending_triplets_;
    std::deque<Eigen::Vector3d> mean_window_;
    Eigen::Vector3d window_sum_ = Eigen::Vector3d::Zero();
    std::deque<UWBTriplet> uwb_buffer_;
    bool first_time_initialized_ = false;
    double first_time_ = 0.0;
    double uwb_min_range_ = 0.2;
    int mean_filter_window_size_ = 4;
    int interp_window_size_ = 4;
};
