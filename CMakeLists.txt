cmake_minimum_required(VERSION 3.0.2)
project(controls_pkg)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  message_generation
)

find_package(Boost 1.58 REQUIRED COMPONENTS system thread filesystem)
find_package(gazebo REQUIRED)

find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

add_message_files (
  FILES
  Waypoint.msg
)
add_service_files (
  FILES
  InitWaypointSet.srv
  GoTo.srv
  GoToPose.srv
  GoToIncremental.srv
)

generate_messages (
  DEPENDENCIES
  std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp visualization_msgs nav_msgs message_generation
)

set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
)

add_executable(offb_node src/offb_node.cpp)
target_link_libraries(offb_node ${catkin_LIBRARIES})

add_executable(simple_offb src/simple_offb_node.cpp)
target_link_libraries(simple_offb ${catkin_LIBRARIES})

add_executable(odo_offset src/odo_offset.cpp)
target_link_libraries(odo_offset ${catkin_LIBRARIES})

add_executable(area_coverage src/area_coverage.cpp)
target_link_libraries(area_coverage ${catkin_LIBRARIES})

add_executable(marker src/obst_marker.cpp)
target_link_libraries(marker ${catkin_LIBRARIES})

add_executable(random src/random_sequence.cpp)
target_link_libraries(random ${catkin_LIBRARIES})

link_libraries(
  ${Boost_SYSTEM_LIBRARY_RELEASE}
  ${Boost_THREAD_LIBRARY_RELEASE}
  ${Boost_TIMER_LIBRARY_RELEASE}
  ${GAZEBO_LIBRARIES}
  ${PROTOBUF_LIBRARY}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

set(gz_msgs
  msgs/Range.proto
)

PROTOBUF_GENERATE_CPP(SEN_PROTO_SRCS SEN_PROTO_HDRS ${gz_msgs})
add_library(gz_msgs SHARED ${SEN_PROTO_SRCS})

set (GZ_MSGS_INCLUDE_DIRS "/home/ksakash/projects/control_ws/devel/include")
include_directories(${GZ_MSGS_INCLUDE_DIRS})

add_executable(obstacle src/obstacle.cpp)
target_link_libraries(obstacle gz_msgs ${catkin_LIBRARIES})
add_dependencies(obstacle gz_msgs)
