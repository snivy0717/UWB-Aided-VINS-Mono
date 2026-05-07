#pragma once

#include <cstddef>
#include <deque>
#include <map>
#include <mutex>
#include <vector>

struct UWBMeasurement
{
    double timestamp = 0.0;
    int anchor_id = -1;
    double range = 0.0;
};

class UWBManager
{
  public:
    void setMaxInterval(double max_interval);
    void setBufferDuration(double buffer_duration);
    void clear();
    void addMeasurement(const UWBMeasurement &measurement);
    std::vector<UWBMeasurement> getMeasurementsNear(double timestamp) const;
    size_t size() const;
    bool getNearestMeasurement(double timestamp, UWBMeasurement &measurement, double &dt) const;

  private:
    void pruneOldMeasurements(double latest_timestamp);

    mutable std::mutex mutex_;
    std::deque<UWBMeasurement> buffer_;
    double max_interval_ = 0.05;
    double buffer_duration_ = 5.0;
};
