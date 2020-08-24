#include <ros/ros.h>

#include <boost/thread.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace std;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
    cout << current_state.mode << endl;
}

void run () {
    ros::spin ();
}

int main (int argc, char** argv) {
    ros::init (argc, argv, "temp");
    ros::NodeHandle nh;

    ros::Subscriber sub =  nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    boost::thread th (run);
    th.join();
    return 0;
}
