#include "uwb_triplet_manager.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <ros/console.h>

// 设置 UWB 最小有效测距。
// 小于该阈值的测距会被认为无效，用于过滤明显错误的 UWB 数据。
void UWBTripletManager::setMinRange(double min_range)
{
    std::lock_guard<std::mutex> lock(mutex_);
    uwb_min_range_ = min_range > 0.0 ? min_range : 0.2;
}

// 设置 UWB 三基站测距的滑动均值滤波窗口大小。
// 窗口越大，输出越平滑，但响应会更慢。
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

// 设置 UWB 插值窗口大小。
// 当前三次插值要求使用 4 个 UWBTriplet，因此最大限制为 4。
void UWBTripletManager::setInterpolationWindowSize(int window_size)
{
    std::lock_guard<std::mutex> lock(mutex_);
    interp_window_size_ = window_size > 0 ? std::min(window_size, 4) : 4;
    while (static_cast<int>(uwb_buffer_.size()) > interp_window_size_)
        uwb_buffer_.pop_front();
}

void UWBTripletManager::setInterpolationMaxGap(double max_gap)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (max_gap > 0.0)
    {
        interp_max_gap_ = max_gap;
        return;
    }

    ROS_WARN("Invalid UWB interpolation max gap %.3f, keep %.3f", max_gap, interp_max_gap_);
}

void UWBTripletManager::setInterpolationTimeTolerance(double tolerance)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (tolerance >= 0.0)
    {
        interp_time_tolerance_ = tolerance;
        return;
    }

    ROS_WARN("Invalid UWB interpolation time tolerance %.3f, keep %.3f", tolerance, interp_time_tolerance_);
}

// 清空 UWBTripletManager 的所有缓存。
// 包括未组装完成的三基站数据、均值滤波窗口和插值 buffer。
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

// 添加一条单基站 UWB 测距。
// 输入是一条 UWBMeasurement，即 timestamp + anchor_id + range。
//
// 处理流程：
//   1. 根据 timestamp 找到对应的 PendingTriplet；
//   2. 按 anchor_id 填入 D0 / D1 / D2；
//   3. 如果三个 anchor 没有到齐，则继续等待；
//   4. 如果三个 anchor 都到齐，则生成 raw_triplet；
//   5. 检查 raw_triplet 是否有效；
//   6. 对 raw_triplet 进行滑动均值滤波；
//   7. 将滤波后的 triplet 放入 UWB 插值 buffer。
//
// accepted_triplet 返回未滤波的原始三基站测距；
// filtered_triplet 返回均值滤波后的三基站测距。
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
    {
        ROS_WARN_THROTTLE(1.0,
                          "UVINS UWB triplet dropped t: %.9f D0: %.3f D1: %.3f D2: %.3f min_range: %.3f max_range: %.3f reason: %s",
                          raw_triplet.timestamp,
                          raw_triplet.ranges[0],
                          raw_triplet.ranges[1],
                          raw_triplet.ranges[2],
                          uwb_min_range_,
                          uwb_max_range_,
                          invalidTripletReason(raw_triplet.ranges));
        return false;
    }

    UWBTriplet filtered = raw_triplet;
    filtered.ranges = meanFilter(raw_triplet.ranges);
    inputUWB(filtered);

    if (accepted_triplet)
        *accepted_triplet = raw_triplet;
    if (filtered_triplet)
        *filtered_triplet = filtered;
    return true;
}

// 将 UWB 三基站测距插值到指定的 VIO / 图像时间。
//
// VINS-Mono 是按图像帧时间进行后端处理的，
// 但 UWB 与图像不是完全同步的。
// 因此需要把 UWB 的 D0、D1、D2 分别插值到当前图像时间。
//
// 当前实现使用 4 个最近的 UWBTriplet 做三次插值。
// 输出 aligned_triplet：
//   timestamp = 当前图像时间；
//   ranges    = 插值后的 [D0, D1, D2]。
bool UWBTripletManager::processUWBAt(double vio_time, UWBTriplet &aligned_triplet) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    const double query_time = first_time_initialized_ ? vio_time - first_time_ : 0.0;
    const double unavailable_time = std::numeric_limits<double>::quiet_NaN();
    const double t0 = uwb_buffer_.size() > 0 ? uwb_buffer_[0].timestamp : unavailable_time;
    const double t1 = uwb_buffer_.size() > 1 ? uwb_buffer_[1].timestamp : unavailable_time;
    const double t2 = uwb_buffer_.size() > 2 ? uwb_buffer_[2].timestamp : unavailable_time;
    const double t3 = uwb_buffer_.size() > 3 ? uwb_buffer_[3].timestamp : unavailable_time;

    const char *skip_reason = nullptr;
    if (!first_time_initialized_)
    {
        skip_reason = "not enough UWB samples";
    }
    else if (interp_window_size_ != 4)
    {
        skip_reason = "unexpected UWB sample count";
    }
    else
    {
        skip_reason = interpolationWindowInvalidReason(query_time);
    }

    if (skip_reason)
    {
        ROS_WARN_THROTTLE(1.0,
                          "UVINS UWB interpolation skipped vio_time: %.9f query_time: %.9f buffer_size: %lu sample_t: [%.9f %.9f %.9f %.9f] interp_max_gap: %.3f interp_time_tolerance: %.3f reason: %s",
                          vio_time,
                          query_time,
                          static_cast<unsigned long>(uwb_buffer_.size()),
                          t0,
                          t1,
                          t2,
                          t3,
                          interp_max_gap_,
                          interp_time_tolerance_,
                          skip_reason);
        return false;
    }

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
        ROS_WARN_THROTTLE(1.0,
                          "UVINS UWB interpolation skipped vio_time: %.9f query_time: %.9f buffer_size: %lu sample_t: [%.9f %.9f %.9f %.9f] interp_max_gap: %.3f interp_time_tolerance: %.3f reason: invalid interpolated range",
                          vio_time,
                          query_time,
                          static_cast<unsigned long>(uwb_buffer_.size()),
                          t0,
                          t1,
                          t2,
                          t3,
                          interp_max_gap_,
                          interp_time_tolerance_);
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

// 判断同一时间戳下三个 anchor 的测距是否都已经收到。
// 只有 D0、D1、D2 全部存在，才能形成一个完整的 UWBTriplet。
bool UWBTripletManager::isComplete(const PendingTriplet &pending) const
{
    return pending.has_anchor[0] && pending.has_anchor[1] && pending.has_anchor[2];
}

// 检查三基站 UWB 测距是否有效。
// 每个 range 必须满足：
//   1. 是有限数值；
//   2. 大于最小有效测距 uwb_min_range_；
//   3. 小于等于宽松上限 uwb_max_range_。
bool UWBTripletManager::isValidTriplet(const Eigen::Vector3d &ranges) const
{
    for (int i = 0; i < 3; ++i)
    {
        if (!::isfinite(ranges[i]) || ranges[i] <= uwb_min_range_ || ranges[i] > uwb_max_range_)
            return false;
    }
    return true;
}

const char *UWBTripletManager::invalidTripletReason(const Eigen::Vector3d &ranges) const
{
    for (int i = 0; i < 3; ++i)
    {
        if (!::isfinite(ranges[i]))
            return "invalid range";
    }
    for (int i = 0; i < 3; ++i)
    {
        if (ranges[i] <= uwb_min_range_)
            return "below min range";
    }
    for (int i = 0; i < 3; ++i)
    {
        if (ranges[i] > uwb_max_range_)
            return "above max range";
    }
    return "unknown";
}

bool UWBTripletManager::isInterpolationWindowValid(double query_time) const
{
    return interpolationWindowInvalidReason(query_time) == nullptr;
}

const char *UWBTripletManager::interpolationWindowInvalidReason(double query_time) const
{
    if (uwb_buffer_.size() != 4)
    {
        if (uwb_buffer_.size() < 4)
            return "not enough UWB samples";
        return "unexpected UWB sample count";
    }

    for (int i = 1; i < 4; ++i)
    {
        const double gap = uwb_buffer_[i].timestamp - uwb_buffer_[i - 1].timestamp;
        if (gap <= 0.0)
            return "non-monotonic UWB timestamps";
        if (gap > interp_max_gap_)
            return "UWB sample gap too large";
    }

    if (!::isfinite(query_time) ||
        query_time < uwb_buffer_.front().timestamp - interp_time_tolerance_ ||
        query_time > uwb_buffer_.back().timestamp + interp_time_tolerance_)
    {
        return "query time outside interpolation window";
    }

    return nullptr;
}

// 对三基站 UWB 测距做滑动均值滤波。
// 目的是抑制 UWB 短时随机噪声，避免单次测距跳变直接影响后续 dP 优化。
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

// 检查三基站 UWB 测距是否有效。
// 每个 range 必须满足：
//   1. 是有限数值；
//   2. 大于最小有效测距 uwb_min_range_。
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

// 三次多项式插值。
// 使用 4 个样本点拟合：
//     range(t) = a * t^3 + b * t^2 + c * t + d
//
// 然后计算 query_time 对应的 range。
// D0、D1、D2 会分别调用该函数进行插值。
bool UWBTripletManager::interpolateCubic(const std::array<Eigen::Vector2d, 4> &samples,
                                         double query_time,
                                         double &value) const
{
    for (int i = 1; i < 4; ++i)
    {
        if (!(samples[i][0] > samples[i - 1][0]))
            return false;
    }

    if (!::isfinite(query_time) ||
        query_time < samples[0][0] - interp_time_tolerance_ ||
        query_time > samples[3][0] + interp_time_tolerance_)
        return false;

    const double eval_time = std::min(std::max(query_time, samples[0][0]), samples[3][0]);

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
    value = para[0] * eval_time * eval_time * eval_time +
            para[1] * eval_time * eval_time +
            para[2] * eval_time +
            para[3];

    return ::isfinite(value) && value > uwb_min_range_ && value <= uwb_max_range_;
}

// 删除长时间没有组装完成的 PendingTriplet。
// 如果某个时间戳下长期只收到部分 anchor 数据，说明该组数据不完整，应该丢弃。
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
