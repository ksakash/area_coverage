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

int time_limit = 0;
int hit_frequency = 0;
int target_margin = 0;

geometry_msgs::PoseStamped curr;
geometry_msgs::PoseStamped next_target;

bool withinRange (const geometry_msgs::PoseStamped& target,
                  const geometry_msgs::PoseStamped& curr) {
    int diffx = abs (target.pose.position.x - curr.pose.position.x);
    int diffy = abs (target.pose.position.y - curr.pose.position.y);
    int diffz = abs (target.pose.position.z - curr.pose.position.z);
    int m = target_margin;
    if (diffx <= m && diffy <= m && diffz <= m) return true;
    else return false;
}

bool reachedTarget (const geometry_msgs::PoseStamped& target) {
    ros::Time then = ros::Time::now();
    int count = 0;
    while ((ros::Time::now() - then) < ros::Duration (time_limit)) {
        if (withinRange (target, curr)) {
            count++;
        }
    }
    if (count >= hit_frequency) return true;
    else return false;
}

void target_handler (std::vector<geometry_msgs::PoseStamped>& plan) {
    // change the target according to the plan
    next_target = plan.back();
    while (!plan.empty()) {
        if (reachedTarget (plan.back())) {
            plan.pop_back ();
            if (!plan.empty()) next_target = plan.back();
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
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
    pose.pose.position.x = 1;
    pose.pose.position.y = 1;
    pose.pose.position.z = 2;

    // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()) {
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
