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
#include <tf/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>

namespace fulanghua {

class PoseEstimator
{
public:
    PoseEstimator(ros::NodeHandle &node);
    
    void spin();

    bool estimate(Eigen::Vector2d &pos, double &yaw, Eigen::Matrix3d &cov_xy_th, const ros::Time &filter_stamp, double dt);

private:
    void publish_odom(const ros::Time &stamp, const Eigen::Vector2d &pos, double yaw
    , const Eigen::Matrix3d &cov_xy_th);

    void publish_pose(const ros::Time &stamp, const Eigen::Vector2d &pos, double yaw
    , const Eigen::Matrix3d &cov_xy_th);

    void imu_callback(const sensor_msgs::Imu &msg) {
        imu_stamp_ = msg.header.stamp;

        tf::Quaternion q;
        quaternionMsgToTF(msg.orientation, q);
        tf::Transform trans(q, tf::Vector3(0, 0, 0));
        tf::StampedTransform meas(trans.inverse(), msg.header.stamp, world_frame_, "imu");
    
        transformer_.setTransform(meas);
    }

    void odom_callback(const nav_msgs::Odometry &msg) {
        odom_stamp_ = msg.header.stamp;

        tf::Quaternion q;
        tf::quaternionMsgToTF(msg.pose.pose.orientation, q);
        tf::Transform trans(q, tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, 0));
        tf::StampedTransform meas(trans.inverse(), msg.header.stamp, world_frame_, "odom");
        
        transformer_.setTransform(meas);
    }

    void gpos_meas_callback(const nav_msgs::Odometry &msg) {
        gpos_meas_stamp_ = msg.header.stamp;

        tf::Quaternion q(0, 0, 0, 1);
        tf::Transform trans(q, tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, 0));
        tf::StampedTransform meas(trans.inverse(), msg.header.stamp, world_frame_, "gpos_meas");

        transformer_.setTransform(meas);
    }

    Eigen::Matrix4d jacob_motion_model(const Eigen::Vector4d &x, const Eigen::Vector2d &u, double dt) {
        Eigen::Matrix4d J;
        J << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             -dt*u(0)*sin(x(2)), dt*cos(x(2)), 1.0, 0.0,
              dt*u(0)*cos(x(2)), dt*sin(x(2)), 0.0, 1.0;

        return J;

    }

    Eigen::Vector4d motion_model(const Eigen::Vector4d &x, const Eigen::Vector2d &u, double dt) {
        Eigen::Matrix4d F;
        F << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 0.0;

        Eigen::MatrixXd B(4, 2);
        B << dt*cos(x(2)), 0.0,
             dt*sin(x(2)), 0.0,
             0.0, dt,
             1.0, 0.0;

        return F*x + B*u;
    }

    Eigen::Matrix4d jacob_observation_model(const Eigen::Vector4d &x) {
        Eigen::Matrix4d J;
        J << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0;
        
         return J;
    }

    Eigen::Vector4d observation_model(const Eigen::Vector4d &x) {
        Eigen::Matrix4d H;
        H << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0;

        return H*x;
    }

    ros::Publisher pose_pub_, odom_pub_;
    ros::Subscriber odom_sub_, imu_sub_, gpos_meas_sub_;
    
    Eigen::Matrix4d cov_est_;
    Eigen::Vector4d x_est_;
    Eigen::Matrix4d motion_cov_;
    Eigen::Matrix4d observation_cov_;

    tf::StampedTransform old_odom_meas_;
    tf::Transformer transformer_;
    tf::TransformBroadcaster tf_broadcaster_;
    
    bool publish_odom_topic_;
    bool publish_pose_topic_;
    double update_rate_;
    std::string output_frame_;
    std::string base_frame_;
    std::string world_frame_;

    ros::Time odom_stamp_;
    ros::Time gpos_meas_stamp_;
    ros::Time imu_stamp_;

};

} //namespace fulanghua

#endif //FULANGHUA_EKF_2d__POSE_ESTIMATOR_H
