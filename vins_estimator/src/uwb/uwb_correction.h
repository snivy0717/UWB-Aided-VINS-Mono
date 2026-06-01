/*
 * 单帧 UWB 位置修正求解器。
 *
 * 作用：
 *   根据当前 VIO 位姿和当前帧附近的 UWB 测距，
 *   线性化求解一个位置修正量 dP。
 *
 * 特点：
 *   1. 只使用当前单帧 UWB 数据；
 *   2. 不维护滑动窗口；
 *   3. 不包含 UVINS-style 的 UWBErr、VIOErr、SmoothErr 联合优化；
 *   4. 主要用于早期调试、fallback 或非 UVINS 模式。
 *
 * 当前主流程：
 *   当 USE_UVINS_UWB_PIPELINE = 1 时，
 *   主要使用 UVINSCorrectionManager，而不是本类。
 */

#pragma once

#include <Eigen/Dense>
#include <vector>

#include "uwb_manager.h"

/*
 * 单帧 UWB 修正结果。
 *
 * valid              : 当前 dP 是否通过有效性检查；
 * dP                 : 求解得到的位置修正量；
 * correction_norm    : dP 的模长；
 * used_anchor_count  : 实际参与求解的 anchor 数量；
 * mean_abs_residual  : UWB 测距平均绝对残差，用于诊断。
 */
struct UWBCorrectionResult
{
    bool valid = false;
    Eigen::Vector3d dP = Eigen::Vector3d::Zero();
    double correction_norm = 0.0;
    int used_anchor_count = 0;
    double mean_abs_residual = 0.0;
};

// 根据当前 VIO 位姿和 UWB 测距，计算单帧位置修正量 dP。
//
// 基本思想：
//   1. 根据当前 VIO 位姿预测 tag 到 anchor 的距离；
//   2. 将 UWB 测距残差对位置进行一阶线性化；
//   3. 构造最小二乘方程 A * dP = b；
//   4. 用 QR 分解求解 dP；
//   5. 如果 dP 太大，则认为该修正无效。
//
// 注意：
//   这是单帧线性化修正，不是当前类 UVINS 的窗口优化主流程。
class UWBCorrection
{
  public:
    static UWBCorrectionResult computePositionCorrection(
        const Eigen::Vector3d &p_vio,
        const Eigen::Matrix3d &R_vio,
        const Eigen::Vector3d &p_uwb_imu,
        const std::vector<Eigen::Vector3d> &anchors,
        const std::vector<UWBMeasurement> &measurements,
        double sigma,
        int min_anchor_count,
        double max_correction_norm);
};
