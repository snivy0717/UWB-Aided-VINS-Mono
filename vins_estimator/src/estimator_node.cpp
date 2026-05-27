#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "vins_estimator/UWBRange.h"
#include "estimator.h"
#include "parameters.h"
#include "uwb/uwb_manager.h"
#include "uwb/uwb_triplet_manager.h"
#include "utility/visualization.h"


Estimator estimator;
UWBManager uwb_manager;
UWBTripletManager uwb_triplet_manager;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::PointCloudConstPtr> relo_buf;
int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;

void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        uwb_manager.clear();
        uwb_triplet_manager.clear();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

void relocalization_callback(const sensor_msgs::PointCloudConstPtr &points_msg)
{
    //printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}

/*
 * UWB ROS 回调函数。
 *
 * 输入 topic：
 *   /uwb/range
 *
 * 每条消息只包含一个 anchor 的测距：
 *   timestamp + anchor_id + range
 *
 * 当 USE_UVINS_UWB_PIPELINE = 1 时：
 *   不直接把单条 range 送入 estimator，
 *   而是先交给 UWBTripletManager，
 *   等 D0、D1、D2 三个基站数据组装完成后，
 *   再进行滤波和后续插值。
 */
void uwb_callback(const vins_estimator::UWBRangeConstPtr &uwb_msg)
{
    if (!USE_UWB)
        return;

    UWBMeasurement measurement;
    measurement.timestamp = uwb_msg->header.stamp.toSec();
    measurement.anchor_id = uwb_msg->anchor_id;
    measurement.range = uwb_msg->range;

    if (USE_UVINS_UWB_PIPELINE)
    {
        UWBTriplet accepted_triplet;
        UWBTriplet filtered_triplet;
        if (uwb_triplet_manager.addRangeMeasurement(measurement, &accepted_triplet, &filtered_triplet))
        {
            ROS_INFO_THROTTLE(1.0,
                              "UVINS UWB triplet accepted t: %.9f D0: %.3f D1: %.3f D2: %.3f",
                              accepted_triplet.timestamp,
                              accepted_triplet.ranges[0],
                              accepted_triplet.ranges[1],
                              accepted_triplet.ranges[2]);
            ROS_INFO_THROTTLE(1.0,
                              "UVINS UWB mean filtered t: %.9f D0: %.3f D1: %.3f D2: %.3f buffer_size: %lu",
                              filtered_triplet.timestamp,
                              filtered_triplet.ranges[0],
                              filtered_triplet.ranges[1],
                              filtered_triplet.ranges[2],
                              static_cast<unsigned long>(uwb_triplet_manager.tripletBufferSize()));
        }
        else
        {
            ROS_DEBUG_THROTTLE(1.0,
                               "UVINS UWB triplet incomplete or invalid t: %.9f anchor_id: %d range: %.3f partial_size: %lu min_range: %.3f",
                               measurement.timestamp, measurement.anchor_id, measurement.range,
                               static_cast<unsigned long>(uwb_triplet_manager.partialBufferSize()),
                               UWB_MIN_RANGE);
        }
        return;
    }

    uwb_manager.addMeasurement(measurement);
    ROS_INFO_THROTTLE(1.0, "UWB received t: %.9f anchor_id: %d range: %.3f buffer_size: %lu",
                      measurement.timestamp, measurement.anchor_id, measurement.range,
                      static_cast<unsigned long>(uwb_manager.size()));
}

/*
 * 将 UWB 数据对齐到当前图像帧时间。
 *
 * VINS-Mono 后端是以图像帧为主节奏运行的，
 * 因此 UWB 测距必须先对齐到 image_timestamp，
 * 才能与该图像帧对应的 VIO 位姿一起用于 dP 修正。
 *
 * UVINS-style 模式下：
 *   使用 UWBTripletManager::processUWBAt()
 *   对 D0、D1、D2 分别做三次插值。
 */
// thread: visual-inertial odometry
void process()
{
    while (true)
    {
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();
        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                double t = imu_msg->header.stamp.toSec();
                double img_t = img_msg->header.stamp.toSec() + estimator.td;
                if (t <= img_t)
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

                }
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }
            // set relocalization frame
            sensor_msgs::PointCloudConstPtr relo_msg = NULL;
            while (!relo_buf.empty())
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != NULL)
            {
                vector<Vector3d> match_points;
                double frame_stamp = relo_msg->header.stamp.toSec();
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                ROS_ASSERT(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
            if (USE_UWB)
            {
                const double image_timestamp = img_msg->header.stamp.toSec();
                std::vector<UWBMeasurement> aligned_uwb_measurements;
                if (USE_UVINS_UWB_PIPELINE)
                {
                    UWBTriplet aligned_triplet;
                    if (uwb_triplet_manager.processUWBAt(image_timestamp, aligned_triplet))
                    {
                        aligned_uwb_measurements.reserve(3);
                        for (int anchor_id = 0; anchor_id < 3; ++anchor_id)
                        {
                            UWBMeasurement aligned_measurement;
                            aligned_measurement.timestamp = image_timestamp;
                            aligned_measurement.anchor_id = anchor_id;
                            aligned_measurement.range = aligned_triplet.ranges[anchor_id];
                            aligned_uwb_measurements.push_back(aligned_measurement);
                        }
                        ROS_INFO_THROTTLE(1.0,
                                          "UVINS UWB aligned image_t: %.9f D0: %.3f D1: %.3f D2: %.3f method: cubic",
                                          image_timestamp,
                                          aligned_triplet.ranges[0],
                                          aligned_triplet.ranges[1],
                                          aligned_triplet.ranges[2]);
                    }
                    else
                    {
                        ROS_DEBUG_THROTTLE(1.0,
                                           "UVINS UWB pipeline waiting for 4 UWB triplets or valid interpolation image_t: %.9f triplet_buffer_size: %lu",
                                           image_timestamp,
                                           static_cast<unsigned long>(uwb_triplet_manager.tripletBufferSize()));
                    }
                }
                else
                {
                    ROS_DEBUG_THROTTLE(1.0, "Feature frame t: %.9f, USE_UWB: %d, UWB buffer_size: %lu, max_interval: %.3f",
                                       image_timestamp, USE_UWB,
                                       static_cast<unsigned long>(uwb_manager.size()), UWB_MAX_INTERVAL);
                    if (USE_UWB_INTERPOLATION)
                    {
                        std::vector<UWBMeasurement> interpolated_measurements;
                        if (uwb_manager.getInterpolatedMeasurementsAt(image_timestamp, interpolated_measurements))
                        {
                            aligned_uwb_measurements = interpolated_measurements;
                            for (const auto &uwb : interpolated_measurements)
                            {
                                ROS_INFO_THROTTLE(1.0, "UWB interpolated image_t: %.9f anchor_id: %d range: %.3f method: linear",
                                                  image_timestamp, uwb.anchor_id, uwb.range);
                            }
                        }
                        else
                        {
                            ROS_DEBUG_THROTTLE(1.0, "No UWB interpolation image_t: %.9f buffer_size: %lu interp_max_gap: %.3f use_interpolation: %d",
                                               image_timestamp, static_cast<unsigned long>(uwb_manager.size()),
                                               UWB_INTERP_MAX_GAP, USE_UWB_INTERPOLATION);
                        }
                    }
                    else
                    {
                        const auto uwb_measurements = uwb_manager.getMeasurementsNear(image_timestamp);
                        if (!uwb_measurements.empty())
                        {
                            aligned_uwb_measurements = uwb_measurements;
                            for (const auto &uwb : uwb_measurements)
                            {
                                ROS_INFO_THROTTLE(1.0, "UWB match image_t: %.9f uwb_t: %.9f dt: %.6f anchor_id: %d range: %.3f",
                                                  image_timestamp, uwb.timestamp, uwb.timestamp - image_timestamp,
                                                  uwb.anchor_id, uwb.range);
                            }
                        }
                        else
                        {
                            UWBMeasurement nearest_uwb;
                            double nearest_dt = 0.0;
                            if (uwb_manager.getNearestMeasurement(image_timestamp, nearest_uwb, nearest_dt))
                            {
                                ROS_DEBUG_THROTTLE(1.0, "No UWB match image_t: %.9f buffer_size: %lu nearest_uwb_t: %.9f nearest_dt: %.6f max_interval: %.3f",
                                                   image_timestamp, static_cast<unsigned long>(uwb_manager.size()),
                                                   nearest_uwb.timestamp, nearest_dt, UWB_MAX_INTERVAL);
                            }
                            else
                            {
                                ROS_DEBUG_THROTTLE(1.0, "No UWB match image_t: %.9f buffer_size: %lu max_interval: %.3f",
                                                   image_timestamp, static_cast<unsigned long>(uwb_manager.size()),
                                                   UWB_MAX_INTERVAL);
                            }
                        }
                    }
                }
                if (!aligned_uwb_measurements.empty())
                // 将已经对齐到图像时间的 UWB 三基站测距送入 Estimator。
                // 后续会在 Estimator::processImage() 中参与类 UVINS correction 窗口优化。
                    estimator.inputUWB(image_timestamp, aligned_uwb_measurements);
            }
            estimator.processImage(image, img_msg->header);

            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";

            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
            if (relo_msg != NULL)
                pubRelocalization(estimator);
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);
    ros::Subscriber sub_uwb;
    if (USE_UWB)
    {
        uwb_manager.setMaxInterval(UWB_MAX_INTERVAL);
        uwb_manager.setInterpolationMaxGap(UWB_INTERP_MAX_GAP);
        uwb_triplet_manager.setMinRange(UWB_MIN_RANGE);
        uwb_triplet_manager.setMeanFilterWindowSize(UWB_MEAN_FILTER_WINDOW_SIZE);
        uwb_triplet_manager.setInterpolationWindowSize(UWB_INTERP_WINDOW_SIZE);
        uwb_triplet_manager.setInterpolationMaxGap(UWB_INTERP_MAX_GAP);
        uwb_triplet_manager.setInterpolationTimeTolerance(0.15);
        sub_uwb = n.subscribe(UWB_TOPIC, 2000, uwb_callback);
        ROS_INFO_STREAM("subscribe UWB topic: " << UWB_TOPIC);
    }

    std::thread measurement_process{process};
    ros::spin();

    return 0;
}
