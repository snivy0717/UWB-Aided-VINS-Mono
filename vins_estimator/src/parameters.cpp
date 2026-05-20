#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string IMU_TOPIC;
int UWB_FUSION_MODE = 0;
int USE_UWB = 0;
std::string UWB_TOPIC = "/uwb/range";
double UWB_NOISE = 0.1;
double UWB_MAX_INTERVAL = 0.05;
int USE_UWB_INTERPOLATION = 1;
double UWB_INTERP_MAX_GAP = 0.2;
int USE_UVINS_UWB_PIPELINE = 1;
double UWB_MIN_RANGE = 0.2;
int UWB_MEAN_FILTER_WINDOW_SIZE = 4;
int UWB_INTERP_WINDOW_SIZE = 4;
int USE_UWB_FACTOR = 0;
int USE_UWB_CORRECTION = 0;
int UWB_CORRECTION_DEBUG_ONLY = 1;
int UWB_CORRECTION_MIN_ANCHORS = 3;
double UWB_CORRECTION_MAX_NORM = 2.0;
int USE_UWB_CORRECTED_OUTPUT = 1;
double UWB_CORRECTED_OUTPUT_MAX_NORM = 2.0;
double UVINS_CORRECTION_MAX_NORM = 2.0;
double UVINS_CORRECTION_MAX_FINAL_COST = 100.0;
double UVINS_CORRECTION_MAX_DELTA_NORM = 0.8;
int USE_UVINS_CORRECTED_OUTPUT = 1;
int UWB_WORLD_ALIGNED = 0;
double UWB_WORLD_TO_VINS_YAW = 0.0;
Eigen::Vector3d UWB_WORLD_TO_VINS_TRANSLATION{0.0, 0.0, 0.0};
std::vector<Eigen::Vector3d> UWB_ANCHOR_POSITIONS;
Eigen::Vector3d P_UWB_IMU{0.0, 0.0, 0.0};
double ROW, COL;
double TD, TR;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["imu_topic"] >> IMU_TOPIC;

    UWB_FUSION_MODE = 0;
    USE_UWB = 0;
    UWB_TOPIC = "/uwb/range";
    UWB_NOISE = 0.1;
    UWB_MAX_INTERVAL = 0.05;
    USE_UWB_INTERPOLATION = 1;
    UWB_INTERP_MAX_GAP = 0.2;
    USE_UVINS_UWB_PIPELINE = 1;
    UWB_MIN_RANGE = 0.2;
    UWB_MEAN_FILTER_WINDOW_SIZE = 4;
    UWB_INTERP_WINDOW_SIZE = 4;
    USE_UWB_FACTOR = 0;
    USE_UWB_CORRECTION = 0;
    UWB_CORRECTION_DEBUG_ONLY = 1;
    UWB_CORRECTION_MIN_ANCHORS = 3;
    UWB_CORRECTION_MAX_NORM = 2.0;
    USE_UWB_CORRECTED_OUTPUT = 1;
    UWB_CORRECTED_OUTPUT_MAX_NORM = 2.0;
    UVINS_CORRECTION_MAX_NORM = 2.0;
    UVINS_CORRECTION_MAX_FINAL_COST = 100.0;
    UVINS_CORRECTION_MAX_DELTA_NORM = 0.8;
    USE_UVINS_CORRECTED_OUTPUT = 1;
    UWB_WORLD_ALIGNED = 0;
    UWB_WORLD_TO_VINS_YAW = 0.0;
    UWB_WORLD_TO_VINS_TRANSLATION.setZero();
    UWB_ANCHOR_POSITIONS.clear();
    P_UWB_IMU.setZero();

    if (!fsSettings["uwb_fusion_mode"].empty())
        UWB_FUSION_MODE = static_cast<int>(fsSettings["uwb_fusion_mode"]);
    if (!fsSettings["use_uwb"].empty())
        USE_UWB = static_cast<int>(fsSettings["use_uwb"]);
    if (!fsSettings["uwb_topic"].empty())
        fsSettings["uwb_topic"] >> UWB_TOPIC;
    if (!fsSettings["uwb_noise"].empty())
        UWB_NOISE = static_cast<double>(fsSettings["uwb_noise"]);
    if (!fsSettings["uwb_max_interval"].empty())
        UWB_MAX_INTERVAL = static_cast<double>(fsSettings["uwb_max_interval"]);
    if (!fsSettings["use_uwb_interpolation"].empty())
        USE_UWB_INTERPOLATION = static_cast<int>(fsSettings["use_uwb_interpolation"]);
    if (!fsSettings["uwb_interp_max_gap"].empty())
        UWB_INTERP_MAX_GAP = static_cast<double>(fsSettings["uwb_interp_max_gap"]);
    if (!fsSettings["use_uvins_uwb_pipeline"].empty())
        USE_UVINS_UWB_PIPELINE = static_cast<int>(fsSettings["use_uvins_uwb_pipeline"]);
    if (!fsSettings["uwb_min_range"].empty())
        UWB_MIN_RANGE = static_cast<double>(fsSettings["uwb_min_range"]);
    if (!fsSettings["uwb_mean_filter_window_size"].empty())
        UWB_MEAN_FILTER_WINDOW_SIZE = static_cast<int>(fsSettings["uwb_mean_filter_window_size"]);
    if (!fsSettings["uwb_interp_window_size"].empty())
        UWB_INTERP_WINDOW_SIZE = static_cast<int>(fsSettings["uwb_interp_window_size"]);
    if (!fsSettings["use_uwb_factor"].empty())
        USE_UWB_FACTOR = static_cast<int>(fsSettings["use_uwb_factor"]);
    if (!fsSettings["use_uwb_correction"].empty())
        USE_UWB_CORRECTION = static_cast<int>(fsSettings["use_uwb_correction"]);
    if (!fsSettings["uwb_correction_debug_only"].empty())
        UWB_CORRECTION_DEBUG_ONLY = static_cast<int>(fsSettings["uwb_correction_debug_only"]);
    if (!fsSettings["uwb_correction_min_anchors"].empty())
        UWB_CORRECTION_MIN_ANCHORS = static_cast<int>(fsSettings["uwb_correction_min_anchors"]);
    if (!fsSettings["uwb_correction_max_norm"].empty())
        UWB_CORRECTION_MAX_NORM = static_cast<double>(fsSettings["uwb_correction_max_norm"]);
    if (!fsSettings["use_uwb_corrected_output"].empty())
        USE_UWB_CORRECTED_OUTPUT = static_cast<int>(fsSettings["use_uwb_corrected_output"]);
    if (!fsSettings["uwb_corrected_output_max_norm"].empty())
        UWB_CORRECTED_OUTPUT_MAX_NORM = static_cast<double>(fsSettings["uwb_corrected_output_max_norm"]);
    if (!fsSettings["uvins_correction_max_norm"].empty())
        UVINS_CORRECTION_MAX_NORM = static_cast<double>(fsSettings["uvins_correction_max_norm"]);
    if (!fsSettings["uvins_correction_max_final_cost"].empty())
        UVINS_CORRECTION_MAX_FINAL_COST = static_cast<double>(fsSettings["uvins_correction_max_final_cost"]);
    if (!fsSettings["uvins_correction_max_delta_norm"].empty())
        UVINS_CORRECTION_MAX_DELTA_NORM = static_cast<double>(fsSettings["uvins_correction_max_delta_norm"]);
    if (!fsSettings["use_uvins_corrected_output"].empty())
        USE_UVINS_CORRECTED_OUTPUT = static_cast<int>(fsSettings["use_uvins_corrected_output"]);
    if (!fsSettings["uwb_world_aligned"].empty())
        UWB_WORLD_ALIGNED = static_cast<int>(fsSettings["uwb_world_aligned"]);
    if (!fsSettings["uwb_world_to_vins_yaw"].empty())
        UWB_WORLD_TO_VINS_YAW = static_cast<double>(fsSettings["uwb_world_to_vins_yaw"]);
    if (!fsSettings["uwb_world_to_vins_translation"].empty())
    {
        cv::FileNode translation = fsSettings["uwb_world_to_vins_translation"];
        if (translation.isSeq() && translation.size() == 3)
        {
            UWB_WORLD_TO_VINS_TRANSLATION << static_cast<double>(translation[0]),
                                             static_cast<double>(translation[1]),
                                             static_cast<double>(translation[2]);
        }
        else
        {
            ROS_WARN("uwb_world_to_vins_translation should be a 3-element sequence; use zero translation");
        }
    }
    if (!fsSettings["p_uwb_imu"].empty())
    {
        cv::FileNode p_uwb_imu = fsSettings["p_uwb_imu"];
        if (p_uwb_imu.isSeq() && p_uwb_imu.size() == 3)
        {
            P_UWB_IMU << static_cast<double>(p_uwb_imu[0]),
                         static_cast<double>(p_uwb_imu[1]),
                         static_cast<double>(p_uwb_imu[2]);
        }
        else
        {
            ROS_WARN("p_uwb_imu should be a 3-element sequence; use zero translation");
        }
    }
    if (!fsSettings["uwb_anchor_positions"].empty())
    {
        cv::FileNode anchors = fsSettings["uwb_anchor_positions"];
        if (anchors.isSeq())
        {
            for (cv::FileNodeIterator it = anchors.begin(); it != anchors.end(); ++it)
            {
                cv::FileNode anchor = *it;
                if (anchor.isSeq() && anchor.size() == 3)
                {
                    UWB_ANCHOR_POSITIONS.emplace_back(static_cast<double>(anchor[0]),
                                                      static_cast<double>(anchor[1]),
                                                      static_cast<double>(anchor[2]));
                }
                else
                {
                    ROS_WARN("skip invalid uwb_anchor_positions entry; expected [x, y, z]");
                }
            }
        }
        else
        {
            ROS_WARN("uwb_anchor_positions should be a sequence of [x, y, z]");
        }
    }
    if (UWB_FUSION_MODE == 0)
    {
        USE_UWB = 0;
        USE_UWB_CORRECTION = 0;
        USE_UWB_FACTOR = 0;
    }
    else if (UWB_FUSION_MODE == 1)
    {
        USE_UWB = 1;
        USE_UWB_CORRECTION = 1;
        USE_UWB_FACTOR = 0;
    }
    else if (UWB_FUSION_MODE == 2)
    {
        USE_UWB = 1;
        USE_UWB_CORRECTION = 0;
        USE_UWB_FACTOR = 1;
        if (!UWB_WORLD_ALIGNED)
            ROS_WARN("UWB fusion mode 2 requires coordinate alignment. Do not enable range factor before uwb_world_aligned=1.");
        ROS_WARN("UWB range factor mode is selected, but current project has not connected UWBRangeFactor to Estimator::optimization() yet.");
    }
    else
    {
        ROS_WARN("Unknown uwb_fusion_mode %d; fallback to mode 0, original VINS-Mono", UWB_FUSION_MODE);
        UWB_FUSION_MODE = 0;
        USE_UWB = 0;
        USE_UWB_CORRECTION = 0;
        USE_UWB_FACTOR = 0;
    }
    ROS_INFO("UWB_FUSION_MODE: %d", UWB_FUSION_MODE);
    ROS_INFO("USE_UWB: %d", USE_UWB);
    if (USE_UWB)
    {
        ROS_INFO_STREAM("UWB_TOPIC: " << UWB_TOPIC);
        ROS_INFO("UWB_NOISE: %f UWB_MAX_INTERVAL: %f", UWB_NOISE, UWB_MAX_INTERVAL);
        ROS_INFO("USE_UWB_INTERPOLATION: %d UWB_INTERP_MAX_GAP: %f", USE_UWB_INTERPOLATION, UWB_INTERP_MAX_GAP);
        ROS_INFO("USE_UVINS_UWB_PIPELINE: %d UWB_MIN_RANGE: %f UWB_MEAN_FILTER_WINDOW_SIZE: %d UWB_INTERP_WINDOW_SIZE: %d",
                 USE_UVINS_UWB_PIPELINE, UWB_MIN_RANGE,
                 UWB_MEAN_FILTER_WINDOW_SIZE, UWB_INTERP_WINDOW_SIZE);
        ROS_INFO("USE_UWB_FACTOR: %d UWB_WORLD_ALIGNED: %d", USE_UWB_FACTOR, UWB_WORLD_ALIGNED);
        ROS_INFO("USE_UWB_CORRECTION: %d UWB_CORRECTION_DEBUG_ONLY: %d", USE_UWB_CORRECTION, UWB_CORRECTION_DEBUG_ONLY);
        ROS_INFO("UWB_CORRECTION_MIN_ANCHORS: %d UWB_CORRECTION_MAX_NORM: %f", UWB_CORRECTION_MIN_ANCHORS, UWB_CORRECTION_MAX_NORM);
        ROS_INFO("USE_UWB_CORRECTED_OUTPUT: %d UWB_CORRECTED_OUTPUT_MAX_NORM: %f",
                 USE_UWB_CORRECTED_OUTPUT, UWB_CORRECTED_OUTPUT_MAX_NORM);
        ROS_INFO("USE_UVINS_CORRECTED_OUTPUT: %d UVINS_CORRECTION_MAX_NORM: %f UVINS_CORRECTION_MAX_FINAL_COST: %f UVINS_CORRECTION_MAX_DELTA_NORM: %f",
                 USE_UVINS_CORRECTED_OUTPUT, UVINS_CORRECTION_MAX_NORM,
                 UVINS_CORRECTION_MAX_FINAL_COST, UVINS_CORRECTION_MAX_DELTA_NORM);
        if (USE_UWB_CORRECTION && !UWB_CORRECTION_DEBUG_ONLY)
            ROS_WARN("use_uwb_correction is enabled with debug_only=0; applying correction is not recommended before real-data and coordinate-frame validation");
        ROS_INFO("UWB_WORLD_TO_VINS_YAW: %f", UWB_WORLD_TO_VINS_YAW);
        ROS_INFO_STREAM("UWB_WORLD_TO_VINS_TRANSLATION: " << UWB_WORLD_TO_VINS_TRANSLATION.transpose());
        if (USE_UWB_FACTOR && !UWB_WORLD_ALIGNED)
            ROS_WARN("use_uwb_factor is enabled but uwb_world_aligned is 0; do not enable UWB range factor until anchor frame and VINS world are aligned");
        ROS_INFO_STREAM("P_UWB_IMU: " << P_UWB_IMU.transpose());
        ROS_INFO("UWB anchor count: %lu", UWB_ANCHOR_POSITIONS.size());
    }

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;

    // create folder if not exists
    FileSystemHelper::createDirectoryIfNotExists(OUTPUT_PATH.c_str());

    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %f COL: %f ", ROW, COL);

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());
        
    } 

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }
    
    fsSettings.release();
}
