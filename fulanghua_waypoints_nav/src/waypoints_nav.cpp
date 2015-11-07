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

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <fulanghua_srvs/Pose.h>

#include <yaml-cpp/yaml.h>

#include <vector>
#include <fstream>
#include <string>
#include <exception>
#include <math.h>
#include <limits>

#ifdef NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}
#endif

class SwitchRunningStatus : public std::exception {
public:
    SwitchRunningStatus() : std::exception() { }
};

class WaypointsNavigation{
public:
    WaypointsNavigation() :
        has_activate_(false),
        move_base_action_("move_base", true),
        rate_(10),
        last_moved_time_(0)
    {
        while((move_base_action_.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
        {
            ROS_INFO("Waiting...");
        }
        
        ros::NodeHandle private_nh("~");
        private_nh.param("robot_frame", robot_frame_, std::string("/base_link"));
        private_nh.param("world_frame", world_frame_, std::string("/map"));
        
        double max_update_rate;
        private_nh.param("max_update_rate", max_update_rate, 10.0);
        rate_ = ros::Rate(max_update_rate);
        std::string filename = "";
        private_nh.param("filename", filename, filename);
        if(filename != ""){
            ROS_INFO_STREAM("Read waypoints data from " << filename);
            if(!readFile(filename)) {
                ROS_ERROR("Failed loading waypoints file");
            }
            current_waypoint_ = waypoints_.begin();
        } else {
            ROS_ERROR("waypoints file doesn't have name");
        }
        
        ros::NodeHandle nh;
        start_server_ = nh.advertiseService("start_wp_nav", &WaypointsNavigation::startNavigationCallback, this);
        suspend_server_ = nh.advertiseService("suspend_wp_pose", &WaypointsNavigation::suspendPoseCallback, this);
        resume_server_ = nh.advertiseService("resume_wp_pose", &WaypointsNavigation::resumePoseCallback, this);
        cmd_vel_sub_ = nh.subscribe("icart_mini/cmd_vel", 1, &WaypointsNavigation::cmdVelCallback, this);
        marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
        clear_costmaps_srv_ = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    }

    bool startNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response) {
        if(has_activate_) {
            response.success = false;
            return false;
        }
        
        std_srvs::Empty empty;
        while(!clear_costmaps_srv_.call(empty)) {
            ROS_WARN("Resend clear costmap service");
            sleep();
        }

        current_waypoint_ = waypoints_.begin();
        has_activate_ = true;
        response.success = true;
        return true;
    }

    bool resumePoseCallback(fulanghua_srvs::Pose::Request &request, fulanghua_srvs::Pose::Response &response) {
        if(has_activate_) {
            response.status = false;
            return false;
        }
        
        std_srvs::Empty empty;
        clear_costmaps_srv_.call(empty);
        //move_base_action_.cancelAllGoals();
        
        ///< @todo calculating metric with request orientation
        double min_dist = std::numeric_limits<double>::max();
        for(std::vector<geometry_msgs::PointStamped>::iterator it = waypoints_.begin(); it != waypoints_.end(); it++) {
            double dist = hypot(it->point.x - request.pose.position.x, it->point.y - request.pose.position.y);
            if(dist < min_dist) {
                min_dist = dist;
                current_waypoint_ = it;
            }
        }
        
        response.status = true;
        has_activate_ = true;

        return true;
    }

    bool suspendPoseCallback(fulanghua_srvs::Pose::Request &request, fulanghua_srvs::Pose::Response &response) {
        if(!has_activate_) {
            response.status = false;
            return false;
        }
        
        //move_base_action_.cancelAllGoals();
        startNavigationGL(request.pose);
        while(!navigationFinished() && ros::ok()) {
            sleep();
        }
        response.status = true;
        has_activate_ = false;

        return true;
    }
    
    void cmdVelCallback(const geometry_msgs::Twist &msg){
        if(msg.linear.x > -0.001 && msg.linear.x < 0.001   &&
           msg.linear.y > -0.001 && msg.linear.y < 0.001   &&
           msg.linear.z > -0.001 && msg.linear.z < 0.001   &&
           msg.angular.x > -0.001 && msg.angular.x < 0.001 &&
           msg.angular.y > -0.001 && msg.angular.y < 0.001 &&
           msg.angular.z > -0.001 && msg.angular.z < 0.001){
            
            ROS_INFO("command velocity all zero");
        }else{
            last_moved_time_ = ros::Time::now().toSec();
        }
    }

    bool readFile(const std::string &filename){
        waypoints_.clear();
        try{
            std::ifstream ifs(filename.c_str(), std::ifstream::in);
            if(ifs.good() == false){
                return false;
            }

            YAML::Node node;
            
            #ifdef NEW_YAMLCPP
                node = YAML::Load(ifs);
            #else
                YAML::Parser parser(ifs);
                parser.GetNextDocument(node);
            #endif

            #ifdef NEW_YAMLCPP
                const YAML::Node &wp_node_tmp = node["waypoints"];
                const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
            #else
                const YAML::Node *wp_node = node.FindValue("waypoints");
            #endif

            if(wp_node != NULL){
                for(int i=0; i < wp_node->size(); i++){
                    geometry_msgs::PointStamped point;

                    (*wp_node)[i]["point"]["x"] >> point.point.x;
                    (*wp_node)[i]["point"]["y"] >> point.point.y;
                    (*wp_node)[i]["point"]["z"] >> point.point.z;

                    waypoints_.push_back(point);

                }
            }else{
                return false;
            }
            
            #ifdef NEW_YAMLCPP
                const YAML::Node &fp_node_tmp = node["finish_pose"];
                const YAML::Node *fp_node = fp_node_tmp ? &fp_node_tmp : NULL;
            #else
                const YAML::Node *fp_node = node.FindValue("finish_pose");
            #endif

            if(fp_node != NULL){
                (*fp_node)["pose"]["position"]["x"] >> finish_pose_.position.x;
                (*fp_node)["pose"]["position"]["y"] >> finish_pose_.position.y;
                (*fp_node)["pose"]["position"]["z"] >> finish_pose_.position.z;

                (*fp_node)["pose"]["orientation"]["x"] >> finish_pose_.orientation.x;
                (*fp_node)["pose"]["orientation"]["y"] >> finish_pose_.orientation.y;
                (*fp_node)["pose"]["orientation"]["z"] >> finish_pose_.orientation.z;
                (*fp_node)["pose"]["orientation"]["w"] >> finish_pose_.orientation.w;
            }else{
                return false;
            }

        }catch(YAML::ParserException &e){
            return false;

        }catch(YAML::RepresentationException &e){
            return false;
        }

        return true;
    }

    bool shouldSendGoal(){
        bool ret = true;
        actionlib::SimpleClientGoalState state = move_base_action_.getState();
        if((state != actionlib::SimpleClientGoalState::ACTIVE) &&
           (state != actionlib::SimpleClientGoalState::PENDING) && 
           (state != actionlib::SimpleClientGoalState::RECALLED) &&
           (state != actionlib::SimpleClientGoalState::PREEMPTED))
        {
            ret = false;
        }

        if(waypoints_.empty()){
            ret = false;
        }

        return ret;
    }

    bool navigationFinished(){
        return move_base_action_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }

    bool onNavigationPoint(const geometry_msgs::Point &dest, double dist_err = 0.8){
        tf::StampedTransform robot_gl = getRobotPosGL();

        const double wx = dest.x;
        const double wy = dest.y;
        const double rx = robot_gl.getOrigin().x();
        const double ry = robot_gl.getOrigin().y();
        const double dist = std::sqrt(std::pow(wx - rx, 2) + std::pow(wy - ry, 2));

        return dist < dist_err;
    }

    tf::StampedTransform getRobotPosGL(){
        tf::StampedTransform robot_gl;
        try{
            tf_listener_.lookupTransform(world_frame_, robot_frame_, ros::Time(0.0), robot_gl);
        }catch(tf::TransformException &e){
            ROS_WARN_STREAM("tf::TransformException: " << e.what());
        }

        return robot_gl;
    }

    void sleep(){
        rate_.sleep();
        ros::spinOnce();
        publishMarkers();
    }

    void startNavigationGL(const geometry_msgs::Point &dest){
        geometry_msgs::Pose pose;
        pose.position = dest;
        pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        startNavigationGL(pose);
    }

    void startNavigationGL(const geometry_msgs::Pose &dest){
        move_base_msgs::MoveBaseGoal move_base_goal;
        move_base_goal.target_pose.header.stamp = ros::Time::now();
        move_base_goal.target_pose.header.frame_id = world_frame_;
        move_base_goal.target_pose.pose.position = dest.position;
        move_base_goal.target_pose.pose.orientation = dest.orientation;
        
        move_base_action_.sendGoal(move_base_goal);
    }

    void publishMarkers(){
        visualization_msgs::MarkerArray markers_array;
        for(int i=0; i < waypoints_.size(); i++){
            visualization_msgs::Marker marker, label;
            marker.header.frame_id = world_frame_;
            marker.header.stamp = ros::Time::now();
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.pose.position.z = marker.scale.z / 2.0;
            marker.color.r = 0.8f;
            marker.color.g = 0.2f;
            marker.color.b = 0.2f;
            
            std::stringstream name;
            name << "waypoint " << i;
            marker.ns = name.str();
            marker.id = i;
            marker.pose.position.x = waypoints_[i].point.x;
            marker.pose.position.y = waypoints_[i].point.y;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.a = 1.0f;
            markers_array.markers.push_back(marker);

            //ROS_INFO_STREAM("waypoints \n" << waypoints_[i]);
        }
        marker_pub_.publish(markers_array);
    }

    void run(){
        while(ros::ok()){
            try {
                if(has_activate_) {
                    geometry_msgs::Pose goal_pose;
                    if(current_waypoint_ == waypoints_.end()-1) {
                        ROS_INFO("prepare finish pose");
                        double goal_direction = atan2(finish_pose_.position.y - current_waypoint_->point.y,
                                                      finish_pose_.position.x - current_waypoint_->point.x);
                        
                        goal_pose.position = current_waypoint_->point;
                        goal_pose.orientation = tf::createQuaternionMsgFromYaw(goal_direction);
                    } else {
                        ROS_INFO("calculate waypoint direction");
                        double goal_direction = atan2((current_waypoint_+1)->point.y - current_waypoint_->point.y,
                                                      (current_waypoint_+1)->point.x - current_waypoint_->point.x);
                        
                        ROS_INFO_STREAM("goal_direction = " << goal_direction);
                        ROS_INFO_STREAM("current_waypoint_+1 " << (current_waypoint_+1)->point.y);
                        ROS_INFO_STREAM("current_waypoint_" << current_waypoint_->point.y);

                        goal_pose.position = current_waypoint_->point;
                        goal_pose.orientation = tf::createQuaternionMsgFromYaw(goal_direction);
                    }

                    startNavigationGL(goal_pose);
                    double start_nav_time = ros::Time::now().toSec();
                    while(!onNavigationPoint(goal_pose.position)) {
                        if(!has_activate_)
                            throw SwitchRunningStatus();
                        
                        double time = ros::Time::now().toSec();
                        if(time - start_nav_time > 10.0 && time - last_moved_time_ > 10.0) {
                            ROS_WARN("Resend the navigation goal.");
                            std_srvs::Empty empty;
                            clear_costmaps_srv_.call(empty);
                            startNavigationGL(goal_pose);
                            start_nav_time = time;
                        }
                        sleep();
                    }

                    if(current_waypoint_ != waypoints_.end()-1) {
                        current_waypoint_++;
                    } else {
                        startNavigationGL(finish_pose_);
                        while(!navigationFinished() && ros::ok()) sleep();
                        has_activate_ = false;
                    }
                }
            } catch(const SwitchRunningStatus &e) {
                ROS_INFO_STREAM("running status switched");
            }

            sleep();
        }
    }

private:
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_;
    std::vector<geometry_msgs::PointStamped> waypoints_;
    std::vector<geometry_msgs::PointStamped>::iterator current_waypoint_;
    geometry_msgs::Pose finish_pose_;
    bool has_activate_;
    std::string robot_frame_, world_frame_;
    tf::TransformListener tf_listener_;
    ros::Rate rate_;
    ros::ServiceServer start_server_, suspend_server_, resume_server_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher marker_pub_;
    ros::ServiceClient clear_costmaps_srv_;
    double last_moved_time_;

};

int main(int argc, char *argv[]){
    ros::init(argc, argv, ROS_PACKAGE_NAME);
    WaypointsNavigation w_nav;
    w_nav.run();

    return 0;
}
