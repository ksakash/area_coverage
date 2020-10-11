#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/msgs/msgs.hh"
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>

#include <Range.pb.h>

using namespace std;

int main (int argc, char** argv) {
    ros::init (argc, argv, "obstacle_avoidance_node");
    return 0;
}
