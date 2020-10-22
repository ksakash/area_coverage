/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <vector>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace std;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

int time_limit = 3;
int hit_frequency = 100;
float target_margin = 0.2;

geometry_msgs::PoseStamped curr_pose;
geometry_msgs::PoseStamped next_target;

void pose_cb (const geometry_msgs::PoseStamped::ConstPtr& msg) {
    curr_pose = *msg;
}

bool withinRange (const geometry_msgs::PoseStamped& target,
                  const geometry_msgs::PoseStamped& curr) {
    float diffx = abs (target.pose.position.x - curr.pose.position.x);
    float diffy = abs (target.pose.position.y - curr.pose.position.y);
    float diffz = abs (target.pose.position.z - curr.pose.position.z);
    float m = target_margin;
    if (diffx <= m && diffy <= m && diffz <= m) return true;
    else return false;
}

bool reachedTarget (const geometry_msgs::PoseStamped& target) {
    ros::Time then = ros::Time::now();
    int count = 0;
    ros::Rate rate (100.0);
    while ((ros::Time::now() - then) < ros::Duration (time_limit)) {
        if (withinRange (target, curr_pose)) {
            count++;
        }
        rate.sleep();
    }
    if (count >= hit_frequency) return true;
    else return false;
}

void target_handler (std::vector<geometry_msgs::PoseStamped>& plan) {
    next_target = plan.back();
    while (!plan.empty()) {
        if (reachedTarget (plan.back())) {
            cout << "Achieved: " << plan.back().pose.position.x << " "
                 << plan.back().pose.position.y << " " << plan.back().pose.position.z << endl;
            plan.pop_back ();
            if (!plan.empty()) next_target = plan.back();
        }
    }
}

ros::Publisher local_pos_pub;

void pub_thread () {
    ros::Rate rate (20.0);
    while (ros::ok()) {
        local_pos_pub.publish (next_target);
        ros::spinOnce ();
        rate.sleep();
    }
}

void process_input (string filename, std::vector<geometry_msgs::PoseStamped>& plan) {
    std::string line;
    std::ifstream file (filename);
    while (getline (file, line)) {
        if (line.size() == 0) continue;
        stringstream ss (line);
        string strx, stry, strz;
        ss >> strx >> stry >> strz;
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = std::stof (strx);
        pose.pose.position.y = std::stof (stry);
        pose.pose.position.z = std::stof (strz);
        plan.push_back (pose);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Rate rate(20.0);

    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    string filename = "/home/ksakash/misc/catkin_ws/src/beginner_tutorials/cfg/roboty0.plan";
    std::vector<geometry_msgs::PoseStamped> plan;

    process_input (filename, plan);

    geometry_msgs::PoseStamped pose;
    pose = plan[0];

    for(int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    next_target = pose;
    boost::thread th (pub_thread);

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    while (ros::ok() && current_state.mode != "OFFBOARD" &&
           !(set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent)) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Offboard enabled");
    while (ros::ok() && !current_state.armed &&
           !(arming_client.call(arm_cmd) &&
            arm_cmd.response.success)) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle armed");

    reverse (plan.begin(), plan.end());

    boost::thread th1 (target_handler, plan);
    th1.join ();

    th.join();

    return 0;
}
