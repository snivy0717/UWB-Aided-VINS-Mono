#include "uwb_correction.h"

#include <cmath>

UWBCorrectionResult UWBCorrection::computePositionCorrection(
    const Eigen::Vector3d &p_vio,
    const Eigen::Matrix3d &R_vio,
    const Eigen::Vector3d &p_uwb_imu,
    const std::vector<Eigen::Vector3d> &anchors,
    const std::vector<UWBMeasurement> &measurements,
    double sigma,
    int min_anchor_count,
    double max_correction_norm)
{
    UWBCorrectionResult result;
    const double safe_sigma = sigma > 0.0 ? sigma : 0.1;
    const int safe_min_anchor_count = min_anchor_count > 0 ? min_anchor_count : 3;
    const double safe_max_norm = max_correction_norm > 0.0 ? max_correction_norm : 2.0;

    std::vector<Eigen::Vector3d> rows;
    std::vector<double> rhs;
    double abs_residual_sum = 0.0;

    const Eigen::Vector3d tag_position = p_vio + R_vio * p_uwb_imu;
    for (const auto &measurement : measurements)
    {
        if (measurement.anchor_id < 0 || measurement.anchor_id >= static_cast<int>(anchors.size()))
            continue;
        if (measurement.range <= 0.0)
            continue;

        const Eigen::Vector3d range_vector = tag_position - anchors[measurement.anchor_id];
        const double predicted_range = range_vector.norm();
        if (predicted_range < 1e-6)
            continue;

        const double residual = predicted_range - measurement.range;
        const Eigen::Vector3d unit_direction = range_vector / predicted_range;
        rows.push_back(unit_direction / safe_sigma);
        rhs.push_back(-residual / safe_sigma);
        abs_residual_sum += std::abs(residual);
    }

    result.used_anchor_count = static_cast<int>(rows.size());
    if (result.used_anchor_count > 0)
        result.mean_abs_residual = abs_residual_sum / result.used_anchor_count;
    if (result.used_anchor_count < safe_min_anchor_count)
        return result;

    Eigen::MatrixXd A(result.used_anchor_count, 3);
    Eigen::VectorXd b(result.used_anchor_count);
    for (int i = 0; i < result.used_anchor_count; ++i)
    {
        A.row(i) = rows[i].transpose();
        b(i) = rhs[i];
    }

    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(A);
    if (qr.rank() < 3)
        return result;

    result.dP = qr.solve(b);
    result.correction_norm = result.dP.norm();
    result.valid = result.correction_norm <= safe_max_norm;
    return result;
}
