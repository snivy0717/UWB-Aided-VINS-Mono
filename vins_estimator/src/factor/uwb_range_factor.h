#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>

class UWBRangeFactor
{
  public:
    UWBRangeFactor(const Eigen::Vector3d &anchor_position,
                   const Eigen::Vector3d &p_uwb_imu,
                   double measured_range,
                   double sigma)
        : anchor_position_(anchor_position),
          p_uwb_imu_(p_uwb_imu),
          measured_range_(measured_range),
          sigma_(sigma > 0.0 ? sigma : 0.1)
    {
    }

    template <typename T>
    bool operator()(const T *const pose_i, T *residuals) const
    {
        Eigen::Matrix<T, 3, 1> p_i(pose_i[0], pose_i[1], pose_i[2]);
        Eigen::Quaternion<T> q_i(pose_i[6], pose_i[3], pose_i[4], pose_i[5]);
        q_i.normalize();

        Eigen::Matrix<T, 3, 1> p_uwb_imu(T(p_uwb_imu_.x()),
                                         T(p_uwb_imu_.y()),
                                         T(p_uwb_imu_.z()));
        Eigen::Matrix<T, 3, 1> anchor_position(T(anchor_position_.x()),
                                               T(anchor_position_.y()),
                                               T(anchor_position_.z()));

        Eigen::Matrix<T, 3, 1> tag_position = p_i + q_i * p_uwb_imu;
        Eigen::Matrix<T, 3, 1> range_vector = tag_position - anchor_position;
        T predicted_range = ceres::sqrt(range_vector.squaredNorm() + T(1e-12));

        residuals[0] = (predicted_range - T(measured_range_)) / T(sigma_);
        return true;
    }

  private:
    Eigen::Vector3d anchor_position_;
    Eigen::Vector3d p_uwb_imu_;
    double measured_range_;
    double sigma_;
};
