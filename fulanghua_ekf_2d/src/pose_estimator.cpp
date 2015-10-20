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

#include <fulanghua_ekf_2d/pose_estimator.h>

#include <boost/assign.hpp>

#include <Eigen/LU>
#include <Eigen/Dense>

namespace fulanghua {

double normalize_rad(double th) {
    while(th > M_PI) th -= 2 * M_PI;
    while(th < -M_PI) th += 2 * M_PI;

    return th;
}

PoseEstimator::PoseEstimator(ros::NodeHandle &node) : 
    publish_pose_topic_(true),
    publish_odom_topic_(false),
    update_rate_(100),
    output_frame_("odom"),
    base_frame_("base_link"),
    world_frame_("map"),
    x_est_(Eigen::Vector4d::Zero()),
    cov_est_(Eigen::Matrix4d::Identity())
{
    ros::NodeHandle nh_private("~");

    nh_private.param("output_frame", output_frame_, output_frame_);
    nh_private.param("base_frame", base_frame_, base_frame_);
    
    double motion_x_err, motion_y_err, motion_ang_err, motion_vel_err;
    double observation_x_err, observation_y_err, observation_ang_err, observation_vel_err;
    
    nh_private.param("motion_x_err", motion_x_err, 0.1);
    nh_private.param("motion_y_err", motion_y_err, 0.1);
    nh_private.param("motion_ang_err", motion_ang_err, 0.1);
    nh_private.param("motion_vel_err", motion_vel_err, 0.05);
    
    nh_private.param("observation_x_err", observation_x_err, 0.1);
    nh_private.param("observation_y_err", observation_y_err, 0.1);
    nh_private.param("observation_ang_err", observation_ang_err, 0.1);
    nh_private.param("observation_vel_err", observation_vel_err, 0.05);

    motion_cov_ << motion_x_err * motion_x_err, 0.0, 0.0, 0.0,
                   0.0, motion_y_err * motion_y_err, 0.0, 0.0,
                   0.0, 0.0, motion_ang_err * motion_ang_err, 0.0,
                   0.0, 0.0, 0.0, motion_vel_err * motion_vel_err;

    observation_cov_ << observation_x_err * observation_x_err, 0.0, 0.0, 0.0,
                       0.0, observation_y_err * observation_y_err, 0.0, 0.0,
                       0.0, 0.0, observation_ang_err * observation_ang_err, 0.0,
                       0.0, 0.0, 0.0, observation_vel_err * observation_vel_err;
    
    nh_private.param("publish_pose_topic", publish_pose_topic_, publish_pose_topic_);
    nh_private.param("publish_odom_topic", publish_odom_topic_, publish_odom_topic_);

    imu_sub_ = node.subscribe("imu", 10, &PoseEstimator::imu_callback, this);
    odom_sub_ = node.subscribe("odom", 10, &PoseEstimator::odom_callback, this);

    gpos_meas_sub_ = node.subscribe("meas_gpos", 10, &PoseEstimator::gpos_meas_callback, this);
    
    if(publish_odom_topic_)
        odom_pub_ = node.advertise<nav_msgs::Odometry>("combined_odom", 10);

    if(publish_pose_topic_)
        pose_pub_ = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("self_pose", 10);

}

void PoseEstimator::spin() {
    ros::Rate rate(update_rate_);
    ros::Time old_filter_stamp(0);
    bool meas_initialized = false;

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        
        if(old_filter_stamp != ros::Time(0) && imu_stamp_ != ros::Time(0) && odom_stamp_ != ros::Time(0)) {
            meas_initialized = true;
        } else {
            meas_initialized = false;
        }

        if(meas_initialized) {
            ros::Time now = ros::Time::now();
            ros::Time filter_stamp = now;

            filter_stamp = std::min(filter_stamp, odom_stamp_);
            filter_stamp = std::min(filter_stamp, imu_stamp_);
            filter_stamp = std::min(filter_stamp, gpos_meas_stamp_);

            double dt = (now - old_filter_stamp).toSec();
            if(fabs(dt) < 0.0001) continue;
            
            if((ros::Time::now() - filter_stamp).toSec() > 1.0) {
                ROS_WARN("receive data are too old!");
                continue;
            }

            Eigen::Vector2d est_pos;
            double est_yaw;
            Eigen::Matrix3d est_cov;
            if(!estimate(est_pos, est_yaw, est_cov, filter_stamp, dt)) continue;
            
            if(publish_odom_topic_) publish_odom(filter_stamp, est_pos, est_yaw, est_cov);
            if(publish_pose_topic_) publish_pose(filter_stamp, est_pos, est_yaw, est_cov);
                        
            geometry_msgs::TransformStamped tran;
            tran.header.stamp = filter_stamp;
            tran.header.frame_id = output_frame_;
            tran.child_frame_id = base_frame_;
            tran.transform.translation.x = est_pos.x();
            tran.transform.translation.y = est_pos.y();
            tran.transform.translation.z = 0.0;
            tran.transform.rotation = tf::createQuaternionMsgFromYaw(est_yaw);

            tf_broadcaster_.sendTransform(tran);

            old_filter_stamp = now;
        } else {
            old_filter_stamp = ros::Time::now();
            
            if(transformer_.canTransform(world_frame_, "odom", old_filter_stamp)) {
                transformer_.lookupTransform("odom", world_frame_, old_filter_stamp, old_odom_meas_);
            }
        }
    }
}

bool PoseEstimator::estimate(Eigen::Vector2d &pos, double &yaw, Eigen::Matrix3d &cov_xy_th, const ros::Time &filter_stamp, double dt) {
    
    tf::StampedTransform odom_meas;
    if(!transformer_.canTransform(world_frame_, "odom", filter_stamp)) {
        ROS_WARN("Failed transform of odom data");
        return false;
    } else {
        transformer_.lookupTransform("odom", world_frame_, filter_stamp, odom_meas);
    }
    
    double gpos_meas_x, gpos_meas_y;
    tf::StampedTransform gpos_meas;
    if(!transformer_.canTransform(world_frame_, "gpos_meas", filter_stamp)) {
        ROS_WARN("Failed transform of gpos_meas data");
        return false;
    } else {
        transformer_.lookupTransform("gpos_meas", world_frame_, filter_stamp, gpos_meas);
    }

    gpos_meas_x = gpos_meas.getOrigin().x();
    gpos_meas_y = gpos_meas.getOrigin().y();
    
    tf::StampedTransform imu_meas;
    if(!transformer_.canTransform(world_frame_, "imu", filter_stamp)) {
        ROS_WARN("Failed transform of imu data");
        return false;
    } else {
        transformer_.lookupTransform("imu", world_frame_, filter_stamp, imu_meas);
    }
    
    double odom_linear_x = (odom_meas.getOrigin().x() - old_odom_meas_.getOrigin().x()) / dt;
    
    double odom_angular_z;
    double tmp_r, tmp_p, odom_yaw;
    odom_meas.getBasis().getEulerYPR(odom_yaw, tmp_p, tmp_r);
    double tmp_old_r, tmp_old_p, old_odom_yaw;
    old_odom_meas_.getBasis().getEulerYPR(old_odom_yaw, tmp_old_p, tmp_old_r);
    odom_angular_z = (odom_yaw - old_odom_yaw) / dt;

    ///< @todo use tf_listener for imu frame
    double imu_roll, imu_pitch, imu_yaw;
    imu_meas.getBasis().getEulerYPR(imu_yaw, imu_pitch, imu_roll);

    //predict
    Eigen::Vector2d u(odom_linear_x, odom_angular_z);
    Eigen::Vector4d x_pred = motion_model(x_est_, u, dt);
    Eigen::Matrix4d JF = jacob_motion_model(x_pred, u, dt);
    Eigen::Matrix4d cov_pred = JF * cov_est_ * JF.transpose() + motion_cov_;

    //update
    Eigen::Vector4d x(gpos_meas_x, gpos_meas_y, imu_yaw, odom_linear_x);
    Eigen::Vector4d z = observation_model(x);
    Eigen::Matrix4d H = jacob_observation_model(x_pred);
    Eigen::Vector4d y = z - observation_model(x_pred);
    y(2) = normalize_rad(y(2));
    Eigen::Matrix4d S = H * cov_pred * H.transpose() + observation_cov_;
    Eigen::Matrix4d K = cov_pred * H.transpose() * S.inverse();
    
    //storage estimation results
    x_est_ = x_pred + K * y;
    cov_est_ = (Eigen::Matrix4d::Identity() - K * H) * cov_pred;

    old_odom_meas_ = odom_meas;
    
    pos.x() = x_est_(0);
    pos.y() = x_est_(1);
    yaw = x_est_(2);
    cov_xy_th = cov_est_.block(0, 0, 3, 3);
    
    return true;
}

void PoseEstimator::publish_odom(const ros::Time &stamp, const Eigen::Vector2d &pos, double yaw, const Eigen::Matrix3d &cov_xy_th) {
    nav_msgs::Odometry est_odom;
    est_odom.header.stamp = stamp;
    est_odom.header.frame_id = output_frame_;
    est_odom.child_frame_id = base_frame_;

    est_odom.pose.pose.position.x = pos.x();
    est_odom.pose.pose.position.y = pos.y();
    est_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    est_odom.pose.covariance = boost::assign::list_of
        (cov_xy_th(0)) (0) (0) (0) (0) (0)
        (0) (cov_xy_th(1)) (0) (0) (0) (0)
        (0) (0) (999999.9) (0) (0) (0)
        (0) (0) (0) (999999.9) (0) (0)
        (0) (0) (0) (0) (0) (cov_xy_th(2));

    odom_pub_.publish(est_odom);
}

void PoseEstimator::publish_pose(const ros::Time &stamp, const Eigen::Vector2d &pos, double yaw, const Eigen::Matrix3d &cov_xy_th) {
    geometry_msgs::PoseWithCovarianceStamped est_pose;
    est_pose.header.stamp = stamp;
    est_pose.header.frame_id = base_frame_;
    est_pose.pose.pose.position.x = pos.x();
    est_pose.pose.pose.position.y = pos.y();
    est_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    est_pose.pose.covariance = boost::assign::list_of
        (cov_xy_th(0)) (0) (0) (0) (0) (0)
        (0) (cov_xy_th(1)) (0) (0) (0) (0)
        (0) (0) (999999.9) (0) (0) (0)
        (0) (0) (0) (999999.9) (0) (0)
        (0) (0) (0) (0) (0) (cov_xy_th(2));

    pose_pub_.publish(est_pose);
}



} //namespace fulanghua

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "pose_estimator");

    ros::NodeHandle nh;
    fulanghua::PoseEstimator estimator(nh);
    estimator.spin();

    return 0;
}
