/*
 * 类 UVINS 的 UWB 外部轨迹修正优化器。
 *
 * 作用：
 *   维护一个 UWB / VIO 同步滑动窗口，
 *   并在窗口内优化每一帧的位置修正量 dP。
 *
 * 每个窗口元素包含：
 *   Us[i]      : 当前帧对齐后的 UWB 三基站测距 [D0, D1, D2]；
 *   dPs[i]     : 当前帧待优化的位置修正量；
 *   Vps[i]     : 当前帧 VINS-Mono 原始位置；
 *   Vqs[i]     : 当前帧 VINS-Mono 原始姿态；
 *   Headers[i] : 当前帧相对时间；
 *   Ps_cov[i]  : 当前帧位置协方差或权重矩阵。
 *
 * 优化目标：
 *   通过 UWBErr、VIOErr、SmoothErr 三类残差联合优化 dP。
 *
 * 重要说明：
 *   该模块只优化外部修正量 dP，用于输出修正轨迹：
 *
 *       p_corrected = p_vio + dP
 *
 *   它不会直接修改 VINS-Mono 后端内部状态：
 *
 *       Ps, Rs, Vs, Bas, Bgs
 *
 *   因此它属于“外部 corrected trajectory 输出”，
 *   不是把 UWBRangeFactor 接入 Estimator::optimization() 的紧耦合后端优化。
 */
/*
 * 当前 UWB 修正方式说明：
 *
 * 本工程当前实现的是类 UVINS 的外部 correction 输出方式。
 *
 * UWB 的作用是：
 *   根据三基站测距，在滑动窗口中优化一个位置修正量 dP，
 *   然后输出修正轨迹：
 *
 *       p_corrected = p_vio + output_dP
 *
 * 其中：
 *   p_vio      是 VINS-Mono 原始轨迹；
 *   output_dP  是 UWBErr + VIOErr + SmoothErr 优化后得到的平滑修正量。
 *
 * 需要特别区分：
 *
 *   1. 当前已经实现：
 *      类 UVINS correction window 优化。
 *      UWB 已经参与优化 dP，并影响黄色 corrected path。
 *
 *   2. 当前尚未实现：
 *      将 UWBRangeFactor 直接加入 Estimator::optimization()。
 *      因此 UWB 还没有直接修改 VINS-Mono 内部状态：
 *          Ps, Rs, Vs, Bas, Bgs
 *
 * 所以当前黄色轨迹是 UWB 修正后的外部输出轨迹，
 * 绿色轨迹是 VINS-Mono 原始轨迹。
 */
#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>

static const int UVINS_OPT_WINDOW_SIZE = 10;

class UVINSCorrectionManager
{
  public:
    UVINSCorrectionManager();

    void clear();
    bool isWindowReady() const;
    int validCount() const;
    bool hasLastOptimization() const;
    void updateState(double timestamp,
                     const Eigen::Vector3d &aligned_uwb_ranges,
                     const Eigen::Vector3d &vio_p,
                     const Eigen::Quaterniond &vio_q,
                     const Eigen::Vector3d &init_dP,
                     const Eigen::Matrix3d &cov);
    bool optimizeCorrection(Eigen::Vector3d &correction_out,
                            double &final_cost_out,
                            int &valid_count_out);
    bool getLastFrameDiagnostics(const Eigen::Vector3d &correction,
                                 double &timestamp,
                                 Eigen::Vector3d &measured_ranges,
                                 Eigen::Vector3d &predicted_ranges,
                                 Eigen::Vector3d &range_residuals,
                                 Eigen::Vector3d &raw_position,
                                 Eigen::Vector3d &corrected_position) const;

    Eigen::Vector3d Us[UVINS_OPT_WINDOW_SIZE];// 对齐到图像时间的三基站 UWB 测距，[D0, D1, D2]。
    Eigen::Vector3d dPs[UVINS_OPT_WINDOW_SIZE];// 每一帧对应的位置修正量 dP，是 Ceres 中真正优化的变量。
    Eigen::Vector3d Vps[UVINS_OPT_WINDOW_SIZE];// 每一帧 VINS-Mono 输出的原始位置。
    Eigen::Quaterniond Vqs[UVINS_OPT_WINDOW_SIZE];// 每一帧 VINS-Mono 输出的原始姿态。
    double Headers[UVINS_OPT_WINDOW_SIZE];// 每一帧的相对时间，用于构造平滑约束。
    Eigen::Matrix3d Ps_cov[UVINS_OPT_WINDOW_SIZE];// 位置协方差或权重矩阵，用于构造 VIOErr 和 SmoothErr 的信息矩阵。
    Eigen::Vector3d last_correction;// 上一次优化成功得到的修正量。
    bool last_optimization_success;// 上一次优化是否成功。
    double last_final_cost;// 上一次优化的最终 cost，用于判断优化质量。

  private:
    int lastValidIndex() const;
    void shiftLeft();
    std::vector<Eigen::Vector3d> anchorsForResidual() const;
    void logAnchorAlignmentOnce(const std::vector<Eigen::Vector3d> &anchors_for_residual) const;
    bool computeJacobian(const Eigen::Vector3d &position,
                         const std::vector<Eigen::Vector3d> &anchors_for_residual,
                         Eigen::Matrix3d &jacobian) const;
    bool predictRanges(int index, const Eigen::Vector3d &correction, Eigen::Vector3d &predicted_ranges) const;
    bool invertWithRegularization(const Eigen::Matrix3d &matrix, Eigen::Matrix3d &inverse) const;
    bool hasValidWindow() const;
    bool first_time_initialized_;
    double first_time_;
};
