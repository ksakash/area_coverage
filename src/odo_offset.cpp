#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

ros::Publisher pub;

void cb (const nav_msgs::Odometry& msg) {
    nav_msgs::Odometry new_msg = msg;
    new_msg.pose.pose.position.x += 1;
    new_msg.pose.pose.position.y += 1;
    pub.publish (new_msg);
}

int main (int argc, char** argv) {
    ros::init (argc, argv, "odo_offset");
    ros::NodeHandle nh;
    pub = nh.advertise<nav_msgs::Odometry>("/uav1/mavros/odometry/in/offset", 10, true);
    ros::Subscriber sub = nh.subscribe("/uav1/mavros/odometry/in", 20, cb);
    ros::spin();
    return 0;
}
