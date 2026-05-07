#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000;
//#define UNIT_SPHERE_ERROR

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string IMU_TOPIC;
extern int USE_UWB;
extern std::string UWB_TOPIC;
extern double UWB_NOISE;
extern double UWB_MAX_INTERVAL;
extern int USE_UWB_INTERPOLATION;
extern double UWB_INTERP_MAX_GAP;
extern int USE_UWB_FACTOR;
extern int USE_UWB_CORRECTION;
extern int UWB_CORRECTION_DEBUG_ONLY;
extern int UWB_CORRECTION_MIN_ANCHORS;
extern double UWB_CORRECTION_MAX_NORM;
extern int UWB_WORLD_ALIGNED;
extern double UWB_WORLD_TO_VINS_YAW;
extern Eigen::Vector3d UWB_WORLD_TO_VINS_TRANSLATION;
extern std::vector<Eigen::Vector3d> UWB_ANCHOR_POSITIONS;
extern Eigen::Vector3d P_UWB_IMU;
extern double TD;
extern double TR;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern double ROW, COL;


void readParameters(ros::NodeHandle &n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
