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

namespace fulanghua {

PoseEstimator::PoseEstimator(ros::NodeHandle &node) : 
    update_rate_(100)
{

}

ros::Time PoseEstimator::update(const ros::Time &old_filter_stamp) {
    ros::Time now = ros::Time::now();
    ros::Time filter_stamp = now;

    double dt = (now - old_filter_stamp).toSec();
    if(fabs(dt) < 0.0001) {
        return ros::Time(0);
    }

    filter_stamp = std::min(filter_stamp, _odom_stamp);
    filter_stamp = std::min(filter_stamp, _ref_gl_stamp);
    filter_stamp
}

void PoseEstimator::spin() {
    bool meas_initialized = false;
    ros::Time old_filter_stamp(0);
    ros::Rate r(update_rate_);

    while(ros::ok()) {
        ros::spinOnce();
        
        if(meas_initialized) {
            ros::Time est_stamp = update(old_filter_stamp);

            if(est_stamp != ros::Time(0)) {
                old_filter_stamp = est_stamp;
            }

            geometry_msgs::PoseWithCovarianceStamped est_pose;
            est_pose.header.stamp = est_stamp;
            est_pose.header.frame_id = world_frame_;
            
        }

        r.sleep();
        
    }
}

} //namespace fulanghua

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "pose_estimator");

    ros::NodeHandle nh;
    fulanghua::PoseEstimator estimator(nh);
    estimator.spin();

    return 0;
}

