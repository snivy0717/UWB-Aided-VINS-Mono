/*
 * 类 UVINS 的 UWB 三基站测距管理器。
 *
 * 作用：
 *   将 /uwb/range 形式的单条测距：
 *      timestamp + anchor_id + range
 *
 *   重新组合成 UVINS 风格的三基站测距：
 *      timestamp + [D0, D1, D2]
 *
 * 主流程：
 *   1. 按时间戳暂存 anchor 0、1、2 的测距；
 *   2. 当同一时间戳下 D0、D1、D2 都到齐时，组成一个 UWBTriplet；
 *   3. 对三基站测距进行有效性检查；
 *   4. 对 UWBTriplet 做滑动均值滤波；
 *   5. 将滤波后的 UWBTriplet 放入插值 buffer；
 *   6. 在图像帧到来时，将 UWBTriplet 三次插值到图像时间。
 *
 * 这是当前 USE_UVINS_UWB_PIPELINE = 1 时使用的主输入流程。
 */

#pragma once

#include "uwb_manager.h"

#include <array>
#include <deque>
#include <map>
#include <mutex>

#include <eigen3/Eigen/Dense>

/*
 * 一组三基站 UWB 测距。
 *
 * timestamp : 该组三基站测距对应的时间；
 * ranges    : 三个 anchor 的测距，顺序为 [D0, D1, D2]。
 */
struct UWBTriplet
{
    double timestamp = 0.0;
    Eigen::Vector3d ranges = Eigen::Vector3d::Zero();
};

/*
 * 尚未组装完成的三基站测距。
 *
 * 因为 /uwb/range 是一条一条来的，
 * 所以同一时间戳下可能先收到 D0，再收到 D1、D2。
 *
 * ranges     : 暂存三个基站的距离；
 * has_anchor : 标记某个 anchor 的距离是否已经收到。
 */
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
    const char *invalidTripletReason(const Eigen::Vector3d &ranges) const;
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
    // Wide sanity upper bound for indoor UWB ranges. Kept internal for now.
    double uwb_max_range_ = 30.0;
    int mean_filter_window_size_ = 4;
    int interp_window_size_ = 4;
};
