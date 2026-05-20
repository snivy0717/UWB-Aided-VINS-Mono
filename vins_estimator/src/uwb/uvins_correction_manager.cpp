#include "uvins_correction_manager.h"

#include "../parameters.h"

#include <algorithm>
#include <ceres/ceres.h>
#include <cmath>

namespace
{
struct UWBErr
{
    UWBErr(const Eigen::Vector3d &vio_p,
           const Eigen::Quaterniond &vio_q,
           const Eigen::Vector3d &p_uwb_imu,
           const std::vector<Eigen::Vector3d> &anchors,
           const Eigen::Vector3d &ranges,
           const Eigen::Matrix3d &information)
        : vio_p_(vio_p), tag_offset_(vio_q * p_uwb_imu),
          anchors_(anchors), ranges_(ranges), information_(information)
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
        return true;
    }

    Eigen::Vector3d vio_p_;
    Eigen::Vector3d tag_offset_;
    std::vector<Eigen::Vector3d> anchors_;
    Eigen::Vector3d ranges_;
    Eigen::Matrix3d information_;
};

struct VIOErr
{
    explicit VIOErr(const Eigen::Matrix3d &information)
        : information_(information)
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
        return true;
    }

    Eigen::Matrix3d information_;
};

struct SmoothErr
{
    SmoothErr(const Eigen::Matrix3d &information,
              const Eigen::Vector3d &vio_p,
              const Eigen::Vector3d &vio_p_prev)
        : information_(information), vio_p_(vio_p), vio_p_prev_(vio_p_prev)
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
        return true;
    }

    Eigen::Matrix3d information_;
    Eigen::Vector3d vio_p_;
    Eigen::Vector3d vio_p_prev_;
};
}

UVINSCorrectionManager::UVINSCorrectionManager()
{
    clear();
}

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

bool UVINSCorrectionManager::isWindowReady() const
{
    return !Us[UVINS_OPT_WINDOW_SIZE - 1].isZero();
}

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

bool UVINSCorrectionManager::hasLastOptimization() const
{
    return last_optimization_success;
}

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
                new UWBErr(Vps[i], Vqs[i], P_UWB_IMU, UWB_ANCHOR_POSITIONS, Us[i], p_uwb_inv));
        problem.AddResidualBlock(uwb_cost, uwb_loss, dPs[i].data());

        ceres::LossFunction *vio_loss = new ceres::HuberLoss(0.15);
        ceres::CostFunction *vio_cost =
            new ceres::AutoDiffCostFunction<VIOErr, 1, 3>(new VIOErr(p_vio_inv));
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
                    new SmoothErr(p_vio_pre_inv, Vps[i], Vps[i - 1]));
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

int UVINSCorrectionManager::lastValidIndex() const
{
    for (int i = UVINS_OPT_WINDOW_SIZE - 1; i >= 0; --i)
    {
        if (!Us[i].isZero())
            return i;
    }
    return -1;
}

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
