#pragma once

#include <eigen3/Eigen/Dense>

static const int UVINS_OPT_WINDOW_SIZE = 10;

class UVINSCorrectionManager
{
  public:
    UVINSCorrectionManager();

    void clear();
    bool isWindowReady() const;
    int validCount() const;
    void updateState(double timestamp,
                     const Eigen::Vector3d &aligned_uwb_ranges,
                     const Eigen::Vector3d &vio_p,
                     const Eigen::Quaterniond &vio_q,
                     const Eigen::Vector3d &init_dP,
                     const Eigen::Matrix3d &cov);

    Eigen::Vector3d Us[UVINS_OPT_WINDOW_SIZE];
    Eigen::Vector3d dPs[UVINS_OPT_WINDOW_SIZE];
    Eigen::Vector3d Vps[UVINS_OPT_WINDOW_SIZE];
    Eigen::Quaterniond Vqs[UVINS_OPT_WINDOW_SIZE];
    double Headers[UVINS_OPT_WINDOW_SIZE];
    Eigen::Matrix3d Ps_cov[UVINS_OPT_WINDOW_SIZE];

  private:
    int lastValidIndex() const;
    void shiftLeft();
};
