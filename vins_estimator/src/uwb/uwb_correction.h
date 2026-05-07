#pragma once

#include <Eigen/Dense>
#include <vector>

#include "uwb_manager.h"

struct UWBCorrectionResult
{
    bool valid = false;
    Eigen::Vector3d dP = Eigen::Vector3d::Zero();
    double correction_norm = 0.0;
    int used_anchor_count = 0;
    double mean_abs_residual = 0.0;
};

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
