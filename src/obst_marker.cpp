#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

void fillMsg (int id, float x, float y, float z, visualization_msgs::Marker& marker) {
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
}

int main (int argc, char** argv) {
    ros::init (argc, argv, "obst_marker");
    ros::NodeHandle nh;
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("viz_marker", 0, true);
    vector<vector<float>> obstacles = {{2,0}, {3,0}, {1,2}, {3,2}, {1,4}, {2,4}};
    visualization_msgs::MarkerArray marker_array;
    for (int i = 0; i < 6; i++) {
        visualization_msgs::Marker msg;
        fillMsg (i, obstacles[i][0], obstacles[i][1], 2, msg);
        marker_array.markers.push_back (msg);
    }
    vis_pub.publish (marker_array);
    ros::spin();
    return 0;
}
