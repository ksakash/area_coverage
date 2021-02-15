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

#include "controls_pkg/InitWaypointSet.h"
#include "controls_pkg/Waypoint.h"
#include "controls_pkg/GoTo.h"
#include "controls_pkg/GoToPose.h"

#include <std_msgs/Time.h>

using namespace std;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

int time_limit = 3;
int hit_frequency = 40;
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
        if (count >= hit_frequency) return true;
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
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

void pub_thread () {
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    ros::Rate rate (20.0);

    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if(set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if(arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        // local_pos_pub.publish (next_target);
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

void process_file (string filename, controls_pkg::InitWaypointSet& srv) {
    std::string line;
    std::ifstream file (filename);
    while (getline (file, line)) {
        if (line.size() == 0) continue;
        stringstream ss (line);
        string strx, stry, strz;
        ss >> strx >> stry >> strz;
        geometry_msgs::Point p;
        controls_pkg::Waypoint w;
        w.point.x = std::stof (strx);
        w.point.y = std::stof (stry);
        w.point.z = std::stof (strz);
        w.max_forward_speed = 0.5;
        w.use_fixed_heading = true;
        srv.request.waypoints.push_back (w);
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
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient client = nh.serviceClient<controls_pkg::InitWaypointSet>
            ("start_waypoint_list");
    ros::ServiceClient goto_client = nh.serviceClient<controls_pkg::GoTo>
            ("go_to");
    ros::ServiceClient goto_pose_client = nh.serviceClient<controls_pkg::GoToPose>
            ("go_to_pose");

    ros::Rate rate(20.0);

    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    string filename = "/home/ksakash/projects/control_ws/src/controls_pkg/cfg/waypoints";
    std::vector<geometry_msgs::PoseStamped> plan;

    process_input (filename, plan);

    geometry_msgs::PoseStamped pose;
    pose = plan[0];

    for(int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    // next_target = pose;
    boost::thread th (pub_thread);

    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";

    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;

    // while (ros::ok() && current_state.mode != "OFFBOARD" &&
    //     !(set_mode_client.call(offb_set_mode) &&
    //         offb_set_mode.response.mode_sent)) {
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    // ROS_INFO("OFFBOARD enabled");
    // while (ros::ok() && !current_state.armed &&
    //     !(arming_client.call(arm_cmd) &&
    //         arm_cmd.response.success)) {
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    // ROS_INFO("Vehicle armed");

    // reverse (plan.begin(), plan.end());

    // boost::thread th1 (target_handler, plan);
    // th1.join ();

    ros::Duration (5.0).sleep ();

    controls_pkg::InitWaypointSet srv;
    process_file (filename, srv);

    std_msgs::Time t;
    t.data = ros::Time::now ();

    std_msgs::String intr;
    intr.data = "cubic";

    srv.request.start_time = t;
    srv.request.start_now = true;
    srv.request.max_forward_speed = 0.5;
    srv.request.heading_offset = 0;
    srv.request.interpolator = intr;

    // controls_pkg::GoTo srv;
    // srv.request.waypoint.point.x = 0;
    // srv.request.waypoint.point.y = 0;
    // srv.request.waypoint.point.z = 3;
    // srv.request.waypoint.max_forward_speed = 0.5;
    // srv.request.interpolator = "cubic";
    // srv.request.max_forward_speed = 0.5;

    if (client.call (srv)) {
        std::cout << "Successfull!!" << std::endl;
    }
    else {
        std::cout << "ERROR in calling the service!!" << std::endl;
    }

    ros::spin ();
    th.join();

    return 0;
}
