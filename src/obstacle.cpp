#include <ros/ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo_client.hh>

#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include <Range.pb.h>

using namespace std;

typedef const boost::shared_ptr<const
    sensor_msgs::msgs::Range> ConstSensorMsgsRangePtr;

int msg0 = 0, msg1 = 0, msg2 = 0;
int msg3 = 0, msg4 = 0, msg5 = 0;
geometry_msgs::PoseStamped curr_pose;

void cb0 (ConstSensorMsgsRangePtr &_msg) {
    int z = curr_pose.pose.position.z;
    msg0 = (_msg->current_distance() > 1.5 && z > 1.5)? 1 : 0;
}

void cb1 (ConstSensorMsgsRangePtr &_msg) {
    msg1 = (_msg->current_distance() > 1.5)? 1 : 0;
}

void cb2 (ConstSensorMsgsRangePtr &_msg) {
    msg2 = (_msg->current_distance() > 1.5)? 1 : 0;
}

void cb3 (ConstSensorMsgsRangePtr &_msg) {
    msg3 = (_msg->current_distance() > 1.5)? 1 : 0;
}

void cb4 (ConstSensorMsgsRangePtr &_msg) {
    msg4 = (_msg->current_distance() > 1.5)? 1 : 0;
}

void cb5 (ConstSensorMsgsRangePtr &_msg) {
    msg5 = (_msg->current_distance() > 1.5)? 1 : 0;
}

void fillMsg (std_msgs::Int8MultiArray& msg) {
    msg.data = {msg0, msg1, msg2, msg3, msg4, msg5};
}

void pose_cb (const geometry_msgs::PoseStamped::ConstPtr& msg) {
    curr_pose = *msg;
}

int main (int argc, char** argv) {
    ros::init (argc, argv, "obstacle_avoidance_node");
    gazebo::client::setup (argc, argv);
    ros::NodeHandle nh;

    curr_pose.pose.position.x = 0;
    curr_pose.pose.position.y = 0;
    curr_pose.pose.position.z = 0;

    ros::Publisher pub = nh.advertise<std_msgs::Int8MultiArray>("/obstacle/map", 20);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);

    gazebo::transport::NodePtr node (new gazebo::transport::Node());
    node->Init();

    gazebo::transport::SubscriberPtr sub0 =
        node->Subscribe ("/gazebo/default/iris_opt_flow/link/lidar0", cb0);

    gazebo::transport::SubscriberPtr sub1 =
        node->Subscribe ("/gazebo/default/iris_opt_flow/link/lidar1", cb1);

    gazebo::transport::SubscriberPtr sub2 =
        node->Subscribe ("/gazebo/default/iris_opt_flow/link/lidar2", cb2);

    gazebo::transport::SubscriberPtr sub3 =
        node->Subscribe ("/gazebo/default/iris_opt_flow/link/lidar3", cb3);

    gazebo::transport::SubscriberPtr sub4 =
        node->Subscribe ("/gazebo/default/iris_opt_flow/link/lidar4", cb4);

    gazebo::transport::SubscriberPtr sub5 =
        node->Subscribe ("/gazebo/default/iris_opt_flow/link/lidar5", cb5);

    ros::Rate rate (20);

    while (ros::ok()) {
        // cout << msg0 << " " << msg1 << " " << msg2 << endl;
        // cout << msg3 << " " << msg4 << " " << msg5 << endl;
        // cout << "------------------------" << endl;
        std_msgs::Int8MultiArray msg;
        fillMsg (msg);
        pub.publish (msg);
        ros::spinOnce ();
        rate.sleep ();
    }

    gazebo::client::shutdown();
    return 0;
}
