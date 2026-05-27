/*
 * 基础 UWB 测距管理器。
 *
 * 作用：
 *   管理单条 UWB 测距数据，即：
 *      timestamp + anchor_id + range
 *
 * 主要功能：
 *   1. 缓存原始 UWB 测距；
 *   2. 根据图像时间查找最近的 UWB 测距；
 *   3. 对单个 anchor 的测距进行线性插值；
 *   4. 删除过旧的 UWB 数据，避免 buffer 无限增长。
 *
 * 注意：
 *   这是较早阶段的 UWB 管理方式。
 *   在当前类 UVINS 流程中，主流程主要使用 UWBTripletManager。
 *   本类主要保留用于兼容、调试或非 UVINS 模式。
 */
#pragma once

#include <cstddef>
#include <deque>
#include <map>
#include <mutex>
#include <vector>

/*
 * 单条 UWB 测距数据。
 *
 * timestamp : 该条 UWB 测距的时间戳；
 * anchor_id : 基站编号，例如 0、1、2；
 * range     : tag 到该基站的测距值，单位一般为米。
 */

struct UWBMeasurement
{
    double timestamp = 0.0;
    int anchor_id = -1;
    double range = 0.0;
};

/*
 * UWBManager 用于管理“单基站单条测距”形式的 UWB 数据。
 *
 * 它不要求同一时刻一定有三个基站数据。
 * 它只是把所有 UWBMeasurement 放入 buffer，
 * 然后根据图像帧时间查找或插值得到可用的 UWB 测距。
 *
 * 当前 UVINS-style 主流程中，更推荐使用 UWBTripletManager。
 */

class UWBManager
{
  public:
    void setMaxInterval(double max_interval);
    void setInterpolationMaxGap(double max_gap);
    void setBufferDuration(double buffer_duration);
    void clear();
    void addMeasurement(const UWBMeasurement &measurement);
    std::vector<UWBMeasurement> getMeasurementsNear(double timestamp) const;
    bool getInterpolatedMeasurementsAt(double timestamp, std::vector<UWBMeasurement> &measurements) const;
    size_t size() const;
    bool getNearestMeasurement(double timestamp, UWBMeasurement &measurement, double &dt) const;

  private:
    void pruneOldMeasurements(double latest_timestamp);

    mutable std::mutex mutex_;
    std::deque<UWBMeasurement> buffer_;
    double max_interval_ = 0.05;
    double interpolation_max_gap_ = 0.2;
    double buffer_duration_ = 5.0;
};
