#include <ros/ros.h>
#include <fulanghua_srvs/suspend_nav.h>

#include <vector>
#include <fstream>
#include <string>
#include <exception>
#include <math.h>
#include <limits>

bool add(fulanghua_srvs::suspend_nav::Request &req, fulanghua_srvs::suspend_nav::Response &res){
    res.resume_input = req.resume_input;
    /*ROS_INFO("request resume_input: %ld", (long int)req.resume_input);
    ROS_INFO("response resume_input: %ld", (long int)res.resume_input);*/
    return true;
}
int main(int argc, char **argv){
    ros::init(argc,argv,"suspend_nav");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("suspend_nav",add);
    ROS_INFO("stopping now.");
    ros::spin();

    return 0;
}
