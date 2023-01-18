//
// Created by ywl on 23-1-18.
//

#ifndef SFAST_LIO_IMU_BRIDGE_HPP
#define SFAST_LIO_IMU_BRIDGE_HPP

#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>

#include "use-ikfom.hpp"
// 用于传递当前帧IMU测量的相关数据 用于在迭代阶段对预测进行更新
struct IMUBridge
{
    deque<sensor_msgs::Imu::ConstPtr> qCurrentIMU;          // 当前帧的IMU观测数据
    state_ikfom stateCurrentBegin;                          // 上一次的后验估计
    Eigen::Matrix<double, 24, 24> covCurrentBegin;          // 上一次估计的后验协方差
    Eigen::Matrix<double, 12, 12> Q;                        // 噪声协方差
    double last_lidar_end_time;
    double last_delta_time;
    Eigen::Vector3d mean_acc;
};

#endif //SFAST_LIO_IMU_BRIDGE_HPP
