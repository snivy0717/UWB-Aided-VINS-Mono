#include "uvins_correction_manager.h"

#include "../parameters.h"

#include <algorithm>
#include <ceres/ceres.h>
#include <cmath>

namespace
{
/*
 * UWB 测距残差。
 *
 * 物理意义：
 *   加上 dP 后的 tag 位置，应该能够解释 UWB 实测距离。
 *
 * 修正后的 tag 位置：
 *
 *   p_tag = vio_p + dP + vio_q * p_uwb_imu
 *
 * 其中：
 *   vio_p     : VINS-Mono 原始位置；
 *   dP        : 当前优化的位置修正量；
 *   vio_q     : VINS-Mono 原始姿态；
 *   p_uwb_imu : UWB tag 相对于 IMU 的外参。
 *
 * 对第 i 个 anchor：
 *
 *   predicted_range_i = || p_tag - anchor_i ||
 *   range_residual_i  = predicted_range_i - measured_range_i
 *
 * 作用：
 *   让修正后的轨迹尽量符合 UWB 测距。
 */
struct UWBErr
{
    UWBErr(const Eigen::Vector3d &vio_p,
           const Eigen::Quaterniond &vio_q,
           const Eigen::Vector3d &p_uwb_imu,
           const std::vector<Eigen::Vector3d> &anchors,
           const Eigen::Vector3d &ranges,
           const Eigen::Matrix3d &information,
           double weight)
        : vio_p_(vio_p), tag_offset_(vio_q * p_uwb_imu),
          anchors_(anchors), ranges_(ranges), information_(information),
          sqrt_weight_(std::sqrt(std::max(weight, 0.0)))
    {
    }

    template <typename T>
    bool operator()(const T *const dP, T *residual) const
    {
        T dis_err[3];
        for (int i = 0; i < 3; ++i)
        {
            const T dx = T(vio_p_.x() + tag_offset_.x()) + dP[0] - T(anchors_[i].x());
            const T dy = T(vio_p_.y() + tag_offset_.y()) + dP[1] - T(anchors_[i].y());
            const T dz = T(vio_p_.z() + tag_offset_.z()) + dP[2] - T(anchors_[i].z());
            dis_err[i] = ceres::sqrt(dx * dx + dy * dy + dz * dz) - T(ranges_[i]);
        }

        residual[0] = T(0.0);
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
                residual[0] += dis_err[r] * T(information_(r, c)) * dis_err[c];
        }
        residual[0] *= T(sqrt_weight_);
        return true;
    }

    Eigen::Vector3d vio_p_;
    Eigen::Vector3d tag_offset_;
    std::vector<Eigen::Vector3d> anchors_;
    Eigen::Vector3d ranges_;
    Eigen::Matrix3d information_;
    double sqrt_weight_;
};

/*
 * VIO 先验残差。
 *
 * 物理意义：
 *   VINS-Mono 原始轨迹仍然具有一定可信度，
 *   因此 UWB 修正量 dP 不能无限变大。
 *
 * 该残差直接惩罚 dP 的大小：
 *
 *   residual ≈ dP^T * information * dP
 *
 * 作用：
 *   防止 UWB 异常测距把轨迹强行拉飞。
 */
struct VIOErr
{
    VIOErr(const Eigen::Matrix3d &information, double weight)
        : information_(information), sqrt_weight_(std::sqrt(std::max(weight, 0.0)))
    {
    }

    template <typename T>
    bool operator()(const T *const dP, T *residual) const
    {
        residual[0] = T(0.0);
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
                residual[0] += dP[r] * T(information_(r, c)) * dP[c];
        }
        residual[0] *= T(sqrt_weight_);
        return true;
    }

    Eigen::Matrix3d information_;
    double sqrt_weight_;
};

/*
 * 平滑残差。
 *
 * 物理意义：
 *   相邻两帧的修正轨迹应该连续，不能突然跳变。
 *
 * 当前帧修正后位置：
 *
 *   p_i_corrected = vio_p_i + dP_i
 *
 * 上一帧修正后位置：
 *
 *   p_{i-1}_corrected = vio_p_{i-1} + dP_{i-1}
 *
 * 平滑误差：
 *
 *   smooth_err = (vio_p_i - vio_p_{i-1}) + dP_i - dP_{i-1}
 *
 * 作用：
 *   抑制 dP 在时间上的突变，让修正轨迹更平滑。
 */
struct SmoothErr
{
    SmoothErr(const Eigen::Matrix3d &information,
              const Eigen::Vector3d &vio_p,
              const Eigen::Vector3d &vio_p_prev,
              double weight)
        : information_(information), vio_p_(vio_p), vio_p_prev_(vio_p_prev),
          sqrt_weight_(std::sqrt(std::max(weight, 0.0)))
    {
    }

    template <typename T>
    bool operator()(const T *const dP, const T *const dP_prev, T *residual) const
    {
        T smooth_err[3];
        for (int i = 0; i < 3; ++i)
            smooth_err[i] = T(vio_p_[i] - vio_p_prev_[i]) + dP[i] - dP_prev[i];

        residual[0] = T(0.0);
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
                residual[0] += smooth_err[r] * T(information_(r, c)) * smooth_err[c];
        }
        residual[0] *= T(sqrt_weight_);
        return true;
    }

    Eigen::Matrix3d information_;
    Eigen::Vector3d vio_p_;
    Eigen::Vector3d vio_p_prev_;
    double sqrt_weight_;
};
}

UVINSCorrectionManager::UVINSCorrectionManager()
{
    clear();
}

// 清空整个 UVINS correction 窗口。
// 系统初始化、重启或 estimator reset 时调用。
// 会重置 UWB 测距、dP、VIO 位姿、时间戳、协方差以及上次优化状态。
void UVINSCorrectionManager::clear()
{
    for (int i = 0; i < UVINS_OPT_WINDOW_SIZE; ++i)
    {
        Us[i].setZero();
        dPs[i].setZero();
        Vps[i].setZero();
        Vqs[i].setIdentity();
        Headers[i] = 0.0;
        Ps_cov[i].setIdentity();
    }
    last_correction.setZero();
    last_optimization_success = false;
    last_final_cost = 0.0;
    first_time_initialized_ = false;
    first_time_ = 0.0;
}

// 判断 correction 窗口是否已经填满。
// 当最后一个窗口位置有有效 UWB 数据时，认为窗口可以开始优化。
bool UVINSCorrectionManager::isWindowReady() const
{
    return !Us[UVINS_OPT_WINDOW_SIZE - 1].isZero();
}

// 统计当前 correction 窗口中有效 UWBTriplet 的数量。
int UVINSCorrectionManager::validCount() const
{
    int count = 0;
    for (int i = 0; i < UVINS_OPT_WINDOW_SIZE; ++i)
    {
        if (!Us[i].isZero())
            ++count;
    }
    return count;
}

// 判断上一次 Ceres 优化是否成功。
bool UVINSCorrectionManager::hasLastOptimization() const
{
    return last_optimization_success;
}

// 向 correction 滑动窗口中插入一帧新的 UWB / VIO 同步数据。
//
// 输入：
//   timestamp          : 当前图像帧时间；
//   aligned_uwb_ranges : 插值到图像时间的三基站 UWB 测距 [D0, D1, D2]；
//   vio_p              : VINS-Mono 当前原始位置；
//   vio_q              : VINS-Mono 当前原始姿态；
//   init_dP            : 当前帧 dP 初值；
//   cov                : 当前帧位置协方差或权重矩阵。
//
// 如果窗口未满，则插入到下一个位置；
// 如果窗口已满，则先左移窗口，丢弃最老数据，再插入最新帧。
void UVINSCorrectionManager::updateState(double timestamp,
                                         const Eigen::Vector3d &aligned_uwb_ranges,
                                         const Eigen::Vector3d &vio_p,
                                         const Eigen::Quaterniond &vio_q,
                                         const Eigen::Vector3d &init_dP,
                                         const Eigen::Matrix3d &cov)
{
    if (!first_time_initialized_)
    {
        first_time_ = timestamp;
        first_time_initialized_ = true;
    }

    int insert_index = lastValidIndex() + 1;
    if (insert_index >= UVINS_OPT_WINDOW_SIZE)
    {
        shiftLeft();
        insert_index = UVINS_OPT_WINDOW_SIZE - 1;
    }

    Us[insert_index] = aligned_uwb_ranges;
    dPs[insert_index] = init_dP;
    Vps[insert_index] = vio_p;
    Vqs[insert_index] = vio_q;
    Headers[insert_index] = timestamp - first_time_;
    Ps_cov[insert_index] = cov;
}

// 在当前 correction 窗口中优化 dP 序列。
//
// 优化变量：
//   dPs[0], dPs[1], ..., dPs[UVINS_OPT_WINDOW_SIZE - 1]
//
// 残差项：
//   1. UWBErr
//      约束修正后的位置符合 UWB 三基站测距；
//
//   2. VIOErr
//      约束 dP 不要过大，防止过度偏离 VINS-Mono 原始轨迹；
//
//   3. SmoothErr
//      约束相邻帧修正后轨迹连续，抑制跳变。
//
// 优化后：
//   取窗口最后一帧的 dP 作为当前时刻的 correction_out。
//
// 注意：
//   这里优化的是外部 dP，不会写回 VINS-Mono 内部 Ps/Rs/Vs/Bas/Bgs。
bool UVINSCorrectionManager::optimizeCorrection(Eigen::Vector3d &correction_out,
                                                double &final_cost_out,
                                                int &valid_count_out)
{
    correction_out.setZero();
    final_cost_out = 0.0;
    valid_count_out = validCount();
    last_optimization_success = false;

    if (!hasValidWindow() || UWB_ANCHOR_POSITIONS.size() != 3)
        return false;

    ceres::Problem problem;
    for (int i = 0; i < UVINS_OPT_WINDOW_SIZE; ++i)
        problem.AddParameterBlock(dPs[i].data(), 3);

    for (int i = 0; i < UVINS_OPT_WINDOW_SIZE; ++i)
    {
        Eigen::Matrix3d p_vio = Ps_cov[i];
        Eigen::Matrix3d p_vio_inv;
        if (!invertWithRegularization(p_vio, p_vio_inv))
            return false;

        Eigen::Matrix3d h_jacobian;
        const Eigen::Vector3d jacobian_position = Vps[i] + dPs[i] + Vqs[i] * P_UWB_IMU;
        if (!computeJacobian(jacobian_position, h_jacobian))
            return false;

        Eigen::Matrix3d p_uwb = h_jacobian * h_jacobian * p_vio;
        Eigen::Matrix3d p_uwb_inv;
        if (!invertWithRegularization(p_uwb, p_uwb_inv))
            return false;

        ceres::LossFunction *uwb_loss = new ceres::HuberLoss(0.15);
        ceres::CostFunction *uwb_cost =
            new ceres::AutoDiffCostFunction<UWBErr, 1, 3>(
                new UWBErr(Vps[i], Vqs[i], P_UWB_IMU, UWB_ANCHOR_POSITIONS, Us[i],
                           p_uwb_inv, UVINS_UWB_RESIDUAL_WEIGHT));
        problem.AddResidualBlock(uwb_cost, uwb_loss, dPs[i].data());

        ceres::LossFunction *vio_loss = new ceres::HuberLoss(0.15);
        ceres::CostFunction *vio_cost =
            new ceres::AutoDiffCostFunction<VIOErr, 1, 3>(
                new VIOErr(p_vio_inv, UVINS_VIO_RESIDUAL_WEIGHT));
        problem.AddResidualBlock(vio_cost, vio_loss, dPs[i].data());

        if (i > 0)
        {
            const double dt = Headers[i] - Headers[i - 1];
            Eigen::Matrix3d p_vio_pre = Ps_cov[i - 1] * std::max(dt, 1e-6) / 0.1;
            Eigen::Matrix3d p_vio_pre_inv;
            if (!invertWithRegularization(p_vio_pre, p_vio_pre_inv))
                return false;

            ceres::LossFunction *smooth_loss = new ceres::HuberLoss(0.15);
            ceres::CostFunction *smooth_cost =
                new ceres::AutoDiffCostFunction<SmoothErr, 1, 3, 3>(
                    new SmoothErr(p_vio_pre_inv, Vps[i], Vps[i - 1],
                                  UVINS_SMOOTH_RESIDUAL_WEIGHT));
            problem.AddResidualBlock(smooth_cost, smooth_loss, dPs[i].data(), dPs[i - 1].data());
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    correction_out = dPs[UVINS_OPT_WINDOW_SIZE - 1];
    final_cost_out = summary.final_cost;
    valid_count_out = validCount();
    last_correction = correction_out;
    last_final_cost = summary.final_cost;
    last_optimization_success = summary.IsSolutionUsable();
    return last_optimization_success;
}

// 获取最新窗口帧的诊断信息。
// 主要用于日志输出和 uvins_correction_debug.csv。
//
// 输出内容包括：
//   1. 当前帧时间戳；
//   2. UWB 实测距离；
//   3. 根据修正后位置预测的 UWB 距离；
//   4. UWB 测距残差；
//   5. 原始 VIO 位置；
//   6. 修正后位置。
bool UVINSCorrectionManager::getLastFrameDiagnostics(const Eigen::Vector3d &correction,
                                                     double &timestamp,
                                                     Eigen::Vector3d &measured_ranges,
                                                     Eigen::Vector3d &predicted_ranges,
                                                     Eigen::Vector3d &range_residuals,
                                                     Eigen::Vector3d &raw_position,
                                                     Eigen::Vector3d &corrected_position) const
{
    const int index = UVINS_OPT_WINDOW_SIZE - 1;
    if (Us[index].isZero() || !predictRanges(index, correction, predicted_ranges))
        return false;

    timestamp = Headers[index] + first_time_;
    measured_ranges = Us[index];
    range_residuals = predicted_ranges - measured_ranges;
    raw_position = Vps[index];
    corrected_position = Vps[index] + correction;
    return true;
}

// 从窗口末尾向前查找最后一个有效 UWB 数据的位置。
// 如果窗口为空，则返回 -1。
int UVINSCorrectionManager::lastValidIndex() const
{
    for (int i = UVINS_OPT_WINDOW_SIZE - 1; i >= 0; --i)
    {
        if (!Us[i].isZero())
            return i;
    }
    return -1;
}

// 计算 UWB 测距对 tag 位置的雅可比矩阵。
//
// 对每个 anchor：
//   range = || position - anchor ||
//
// 其对 position 的导数为：
//   (position - anchor) / range
//
// 也就是从 anchor 指向 tag 的单位方向向量。
bool UVINSCorrectionManager::computeJacobian(const Eigen::Vector3d &position, Eigen::Matrix3d &jacobian) const
{
    if (UWB_ANCHOR_POSITIONS.size() != 3)
        return false;

    for (int i = 0; i < 3; ++i)
    {
        const Eigen::Vector3d delta = position - UWB_ANCHOR_POSITIONS[i];
        const double distance = delta.norm();
        if (distance < 1e-6)
            return false;
        jacobian.row(i) = delta.transpose() / distance;
    }
    return true;
}

// 根据指定 correction dP 预测某个窗口帧的三基站 UWB 距离。
//
// 修正后的 tag 位置：
//   p_tag = Vps[index] + correction + Vqs[index] * P_UWB_IMU
//
// 然后分别计算 p_tag 到三个 anchor 的距离。
bool UVINSCorrectionManager::predictRanges(int index, const Eigen::Vector3d &correction, Eigen::Vector3d &predicted_ranges) const
{
    if (index < 0 || index >= UVINS_OPT_WINDOW_SIZE || UWB_ANCHOR_POSITIONS.size() != 3)
        return false;

    const Eigen::Vector3d tag_position = Vps[index] + correction + Vqs[index] * P_UWB_IMU;
    for (int i = 0; i < 3; ++i)
    {
        predicted_ranges[i] = (tag_position - UWB_ANCHOR_POSITIONS[i]).norm();
        if (!std::isfinite(predicted_ranges[i]))
            return false;
    }
    return true;
}

// 对 3x3 矩阵进行带正则化的安全求逆。
//
// 为了避免矩阵奇异或病态，先加一个很小的对角项：
//   matrix + 1e-6 * I
//
// 如果行列式过小或结果不是有限数，则返回 false。
bool UVINSCorrectionManager::invertWithRegularization(const Eigen::Matrix3d &matrix, Eigen::Matrix3d &inverse) const
{
    Eigen::Matrix3d regularized = matrix;
    regularized += 1e-6 * Eigen::Matrix3d::Identity();
    const double determinant = regularized.determinant();
    if (!std::isfinite(determinant) || std::fabs(determinant) < 1e-12)
        return false;
    inverse = regularized.inverse();
    return inverse.allFinite();
}

// 检查 correction 窗口是否完整且数值有效。
//
// 要求：
//   1. 窗口内有效帧数量等于 UVINS_OPT_WINDOW_SIZE；
//   2. UWB 测距、VIO 位置、dP、协方差矩阵都不是 NaN 或 Inf。
bool UVINSCorrectionManager::hasValidWindow() const
{
    if (validCount() != UVINS_OPT_WINDOW_SIZE)
        return false;

    for (int i = 0; i < UVINS_OPT_WINDOW_SIZE; ++i)
    {
        if (Us[i].isZero() || !Us[i].allFinite() || !Vps[i].allFinite() ||
            !dPs[i].allFinite() || !Ps_cov[i].allFinite())
        {
            return false;
        }
    }
    return true;
}

// 滑动窗口左移一格。
//
// 作用：
//   1. 丢弃最老的一帧；
//   2. 其余帧依次前移；
//   3. 最后一个位置清空，用于接收新的 UWB / VIO 数据。
void UVINSCorrectionManager::shiftLeft()
{
    for (int i = 0; i < UVINS_OPT_WINDOW_SIZE - 1; ++i)
    {
        Us[i] = Us[i + 1];
        dPs[i] = dPs[i + 1];
        Vps[i] = Vps[i + 1];
        Vqs[i] = Vqs[i + 1];
        Headers[i] = Headers[i + 1];
        Ps_cov[i] = Ps_cov[i + 1];
    }

    Us[UVINS_OPT_WINDOW_SIZE - 1].setZero();
    dPs[UVINS_OPT_WINDOW_SIZE - 1].setZero();
    Vps[UVINS_OPT_WINDOW_SIZE - 1].setZero();
    Vqs[UVINS_OPT_WINDOW_SIZE - 1].setIdentity();
    Headers[UVINS_OPT_WINDOW_SIZE - 1] = 0.0;
    Ps_cov[UVINS_OPT_WINDOW_SIZE - 1].setIdentity();
}
