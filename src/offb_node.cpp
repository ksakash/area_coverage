/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <vector>
#include <iomanip>
#include <iostream>
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
    // cout << "count: " << count << endl;
    if (count >= hit_frequency) return true;
    else return false;
}

void target_handler (std::vector<geometry_msgs::PoseStamped>& plan) {
    // change the target according to the plan
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

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    // send a few setpoints before starting
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

    // ros::Time last_request = ros::Time::now();

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

    // ros::Duration (5.0).sleep ();
    // ROS_INFO ("try to note if the target is achieved!!");

    // reachedTarget (pose);

    std::vector<geometry_msgs::PoseStamped> plan;
    geometry_msgs::PoseStamped pose1;
    pose1.pose.position.x = 0;
    pose1.pose.position.y = 0;
    pose1.pose.position.z = 2;

    geometry_msgs::PoseStamped pose2;
    pose2.pose.position.x = 0;
    pose2.pose.position.y = 2;
    pose2.pose.position.z = 2;

    geometry_msgs::PoseStamped pose3;
    pose3.pose.position.x = 2;
    pose3.pose.position.y = 2;
    pose3.pose.position.z = 2;

    geometry_msgs::PoseStamped pose4;
    pose4.pose.position.x = 2;
    pose4.pose.position.y = 0;
    pose4.pose.position.z = 2;

    geometry_msgs::PoseStamped pose5;
    pose5.pose.position.x = 0;
    pose5.pose.position.y = 0;
    pose5.pose.position.z = 2;

    plan.push_back (pose1);
    plan.push_back (pose2);
    plan.push_back (pose3);
    plan.push_back (pose4);
    plan.push_back (pose5);
    reverse (plan.begin(), plan.end());

    boost::thread th1 (target_handler, plan);

    // while(ros::ok()) {
    //     if( current_state.mode != "OFFBOARD" &&
    //         (ros::Time::now() - last_request > ros::Duration(5.0))) {
    //         if( set_mode_client.call(offb_set_mode) &&
    //             offb_set_mode.response.mode_sent) {
    //             ROS_INFO("Offboard enabled");
    //         }
    //         last_request = ros::Time::now();
    //     } else {
    //         if( !current_state.armed &&
    //             (ros::Time::now() - last_request > ros::Duration(5.0))) {
    //             if( arming_client.call(arm_cmd) &&
    //                 arm_cmd.response.success) {
    //                 ROS_INFO("Vehicle armed");
    //             }
    //             last_request = ros::Time::now();
    //         }
    //     }
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    th.join();

    return 0;
}
