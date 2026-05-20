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

    Eigen::Vector3d Us[UVINS_OPT_WINDOW_SIZE];
    Eigen::Vector3d dPs[UVINS_OPT_WINDOW_SIZE];
    Eigen::Vector3d Vps[UVINS_OPT_WINDOW_SIZE];
    Eigen::Quaterniond Vqs[UVINS_OPT_WINDOW_SIZE];
    double Headers[UVINS_OPT_WINDOW_SIZE];
    Eigen::Matrix3d Ps_cov[UVINS_OPT_WINDOW_SIZE];
    Eigen::Vector3d last_correction;
    bool last_optimization_success;
    double last_final_cost;

  private:
    int lastValidIndex() const;
    void shiftLeft();
    bool computeJacobian(const Eigen::Vector3d &position, Eigen::Matrix3d &jacobian) const;
    bool predictRanges(int index, const Eigen::Vector3d &correction, Eigen::Vector3d &predicted_ranges) const;
    bool invertWithRegularization(const Eigen::Matrix3d &matrix, Eigen::Matrix3d &inverse) const;
    bool hasValidWindow() const;
    bool first_time_initialized_;
    double first_time_;
};
