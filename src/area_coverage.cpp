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

mavros_msgs::State current_state0;
void state_cb0 (const mavros_msgs::State::ConstPtr& msg) {
    current_state0 = *msg;
}

mavros_msgs::State current_state1;
void state_cb1 (const mavros_msgs::State::ConstPtr& msg) {
    current_state1 = *msg;
}

int time_limit = 3;
int hit_frequency = 100;
float target_margin = 0.2;

geometry_msgs::PoseStamped curr_pose0;
geometry_msgs::PoseStamped curr_pose1;

geometry_msgs::PoseStamped next_target0;
geometry_msgs::PoseStamped next_target1;

void pose_cb0 (const geometry_msgs::PoseStamped::ConstPtr& msg) {
    curr_pose0 = *msg;
}

void pose_cb1 (const geometry_msgs::PoseStamped::ConstPtr& msg) {
    curr_pose1 = *msg;
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

bool reachedTarget (const geometry_msgs::PoseStamped& target,
                    const geometry_msgs::PoseStamped& curr_pose) {
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

void target_handler0 (std::vector<geometry_msgs::PoseStamped>& plan0,
                      std::vector<geometry_msgs::PoseStamped>& plan1) {
    next_target0 = plan0.back();
    next_target1 = plan1.back();
    while (ros::ok() && !plan0.empty() && !plan1.empty()) {
        if (reachedTarget (plan0.back(), curr_pose0) &&
            reachedTarget (plan1.back(), curr_pose1)) {
            cout << "UAV0 Achieved: " << plan0.back().pose.position.x << " "
                 << plan0.back().pose.position.y << " " << plan0.back().pose.position.z << endl;
            cout << "UAV1 Achieved: " << plan1.back().pose.position.x+1 << " "
                 << plan1.back().pose.position.y+1 << " " << plan1.back().pose.position.z << endl;
            plan0.pop_back ();
            plan1.pop_back ();
            if (!plan0.empty()) next_target0 = plan0.back();
            if (!plan1.empty()) next_target1 = plan1.back();
        }
    }
}

ros::Publisher local_pos_pub0;
ros::Publisher local_pos_pub1;

void pub_thread () {
    ros::Rate rate (20.0);
    while (ros::ok()) {
        local_pos_pub0.publish (next_target0);
        local_pos_pub1.publish (next_target1);
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
    ros::init(argc, argv, "area_coverage");
    ros::NodeHandle nh;

    ros::Subscriber state_sub0 = nh.subscribe<mavros_msgs::State>
            ("/uav0/mavros/state", 10, state_cb0);
    ros::Subscriber pose_sub0 = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav0/mavros/local_position/pose", 10, pose_cb0);
    local_pos_pub0 = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav0/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client0 = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client0 = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav0/mavros/set_mode");

    ros::Subscriber state_sub1 = nh.subscribe<mavros_msgs::State>
            ("/uav1/mavros/state", 10, state_cb1);
    ros::Subscriber pose_sub1 = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav1/mavros/local_position/pose", 10, pose_cb1);
    local_pos_pub1 = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav1/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client1 = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client1 = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav1/mavros/set_mode");

    ros::Rate rate(20.0);

    while(ros::ok() && !current_state0.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok() && !current_state1.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    string filename = "/home/ksakash/misc/catkin_ws/src/beginner_tutorials/cfg/roboty0.plan";
    std::vector<geometry_msgs::PoseStamped> plan0;

    process_input (filename, plan0);

    filename = "/home/ksakash/misc/catkin_ws/src/beginner_tutorials/cfg/roboty1.plan";
    std::vector<geometry_msgs::PoseStamped> plan1;

    process_input (filename, plan1);
    for (geometry_msgs::PoseStamped& wp : plan1) {
        wp.pose.position.x -= 1;
        wp.pose.position.y -= 1;
    }

    geometry_msgs::PoseStamped pose0, pose1;
    pose0 = plan0[0];
    pose1 = plan1[0];

    for(int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub0.publish(pose0);
        local_pos_pub0.publish(pose1);
        ros::spinOnce();
        rate.sleep();
    }
    next_target0 = pose0;
    next_target1 = pose1;
    boost::thread th (pub_thread);

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    while (ros::ok() && current_state0.mode != "OFFBOARD" &&
           !(set_mode_client0.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent)) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Offboard enabled in UAV0");
    while (ros::ok() && !current_state0.armed &&
           !(arming_client0.call(arm_cmd) &&
            arm_cmd.response.success)) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle armed in UAV0");

    while (ros::ok() && current_state1.mode != "OFFBOARD" &&
           !(set_mode_client1.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent)) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Offboard enabled in UAV1");
    while (ros::ok() && !current_state1.armed &&
           !(arming_client1.call(arm_cmd) &&
            arm_cmd.response.success)) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle armed in UAV1");

    reverse (plan0.begin(), plan0.end());
    reverse (plan1.begin(), plan1.end());

    boost::thread th1 (target_handler0, plan0, plan1);
    th1.join();
    th.join();

    return 0;
}
