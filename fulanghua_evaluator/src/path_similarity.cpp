#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>

namespace fulanghua {

class PathSimilarity {
public:
    PathSimilarity(ros::NodeHandle &nh) : 
        path_markers_pub_(nh.advertise<visualization_msgs::MarkerArray>("path_markers", 10)),
        path_a_sub_(nh.subscribe("path_a", 1, &PathSimilarity::pathACallback, this)),
        path_b_sub_(nh.subscribe("path_b", 1, &PathSimilarity::pathBCallback, this))
    {

    }

    void spin() {
        ros::Rate r(100);
        while(ros::ok()) {
            ros::spinOnce();
            visualization_msgs::MarkerArray viz_markers;
            int marker_id = 0;
            
            for(auto pose_a : path_a_.poses) {
                double pose_len = get_distance(std::begin(path_a_.poses)->pose.position, pose_a.pose.position);
                for(auto pose_b : path_b_.poses) {
                    if(pose_len <= get_distance(std::begin(path_b_.poses)->pose.position, pose_b.pose.position)) {
                        visualization_msgs::Marker line_strip;
                        line_strip.header.frame_id = "map";
                        line_strip.header.stamp = ros::Time::now();
                        line_strip.ns = "fulanghua_evaluator";
                        
                        line_strip.id = marker_id;
                        marker_id++;

                        line_strip.scale.x = 0.05;
                        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
                        line_strip.action = visualization_msgs::Marker::ADD;
                        line_strip.color.b = 1.0;
                        line_strip.color.a = 1.0;
                        line_strip.points.push_back(pose_a.pose.position);
                        line_strip.points.push_back(pose_b.pose.position);
                        viz_markers.markers.push_back(line_strip);
                        
                        break;
                    }
                }
            }
            path_markers_pub_.publish(viz_markers);

            r.sleep();
        }
    }

private:
    double get_distance(const geometry_msgs::Point &pose1, const geometry_msgs::Point &pose2) {
        return sqrt(pow(pose1.x - pose2.x, 2) + 
                    pow(pose1.y - pose2.y, 2) + 
                    pow(pose1.z - pose2.z, 2));
    }

    void pathACallback(const nav_msgs::Path &msg) {
        path_a_ = msg;
    }

    void pathBCallback(const nav_msgs::Path &msg) {
        path_b_ = msg;
    }

    nav_msgs::Path path_a_, path_b_;
    ros::Publisher path_markers_pub_;
    ros::Subscriber path_a_sub_, path_b_sub_;
};

} //namespace fulanghua

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "path_similarity");

    ros::NodeHandle nh;
    fulanghua::PathSimilarity similarity_evaluator(nh);
    similarity_evaluator.spin();

    return 0;
}
