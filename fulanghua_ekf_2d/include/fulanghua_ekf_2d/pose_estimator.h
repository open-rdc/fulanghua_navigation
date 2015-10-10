/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Daiki Maekawa and Chiba Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FULANGHUA_EKF_2D__POSE_ESTIMATOR_H
#define FULANGHUA_EKF_2D__POSE_ESTIMATOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <Eigen/Geometry>
#include <Eigen/Core>

namespace fulanghua {

class PoseEstimator
{
public:
    PoseEstimator(ros::NodeHandle &node);
    
    void spin();

    bool estimate(Eigen::Vector2d &pos, double &yaw, Eigen::Matrix3d &cov_xy_th, const ros::Time &filter_stamp, double dt);

private:
    Eigen::Vector4d jacob_motion_model(const Eigen::Vector4d &x, const Eigen::Vector2d &u, double dt) {
        Eigen::Matrix4d J;
        J << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             -dt*u(0)*sin(x(2)), dt*cos(x(2)), 1.0, 0.0,
              dt*u(0)*cos(x(2)), dt*sin(x(2)), 0.0, 1.0;

        return J;

    }

    Eigen::Vector4d jacob_observation_model(const Eigen::Vector4d &x) {
        Eigen::Matrix4d J;
        J << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0;

         return J;
    }
    
    Eigen::Matrix4d motion_model(const Eigen::Vector4d &x, const Eigen::Vector2d &u, double dt) {
        Eigen::Matrix4d F;
        F << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0;

        Eigen::MatrixXd B(4, 2);
        B << dt*cos(x(2)), 0.0,
             dt*sin(x(2)), 0.0,
             0.0, dt,
             1.0, 0.0;

        return F*x + B*u;
    }

    Eigen::Matrix4d observation_model(const Eigen::Vector4d &x) {
        Eigen::Matrix4d H;
        H << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0;

        return H*x;
    }

    ros::Publisher pose_pub_;
    ros::Subscriber odom_sub_, imu_sub_, gpos_meas_sub_;
    
    Eigen::Matrix4d cov_est_;
    Eigen::Vector4d x_est_;
    Eigen::Matrix4d motion_cov_;
    Eigen::Matrix4d observation_cov_;

    tf::StampedTransform old_odom_meas_;
    
};

} //namespace fulanghua

#endif //FULANGHUA_EKF_2d__POSE_ESTIMATOR_H
