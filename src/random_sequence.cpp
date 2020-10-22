#include <ros/ros.h>

#include <boost/thread.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int8MultiArray.h>

using namespace std;

mavros_msgs::State current_state;
void state_cb (const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

int time_limit = 1;
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

int getrandom (int l, int r) {
    return (int) (l + int (std::rand()) % (r - l + 1));
}

vector<int> obstacle;
void cb (const std_msgs::Int8MultiArray::ConstPtr& msg) {
    obstacle = {msg->data[0], msg->data[1], msg->data[2],
                msg->data[3], msg->data[4], msg->data[5]};
}

void get_next_target (geometry_msgs::PoseStamped& target) {
    vector<int> arr;
    for (int i = 0; i < obstacle.size(); i++) {
        if (obstacle[i]) arr.push_back (i);
    }
    if (arr.size() == 0) return;
    int index = getrandom (0, arr.size()-1);
    int dir = arr[index];
    if (dir == 0) {
        target.pose.position.z -= 1;
    }
    else if (dir == 1) {
        target.pose.position.z += 1;
    }
    else if (dir == 2) {
        target.pose.position.x += 1;
    }
    else if (dir == 3) {
        target.pose.position.x -= 1;
    }
    else if (dir == 4) {
        target.pose.position.y -= 1;
    }
    else {
        target.pose.position.y += 1;
    }
}

void target_handler (geometry_msgs::PoseStamped& target) {
    ros::Rate rate (20);
    while (ros::ok()) {
        if (reachedTarget (target)) {
            get_next_target (target);
            next_target = target;
            cout << target.pose.position.x << " "
                 << target.pose.position.y << " "
                 << target.pose.position.z << endl;
        }
        ros::spinOnce ();
        rate.sleep ();
    }
}

ros::Publisher local_pos_pub;

void pub_thread () {
    ros::Rate rate (20);
    while (ros::ok()) {
        local_pos_pub.publish (next_target);
        ros::spinOnce ();
        rate.sleep ();
    }
}

int main (int argc, char** argv) {
    ros::init (argc, argv, "random_sequence");
    ros::NodeHandle nh;
    obstacle.resize (6);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber ob_map_sub = nh.subscribe<std_msgs::Int8MultiArray>
            ("/obstacle/map", 10, cb);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Rate rate (20);

    while (ros::ok() && !current_state.connected) {
        ros::spinOnce ();
        rate.sleep ();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish (pose);
        ros::spinOnce ();
        rate.sleep ();
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

    boost::thread th1 (target_handler, pose);

    th1.join ();
    th.join ();

    return 0;
}
