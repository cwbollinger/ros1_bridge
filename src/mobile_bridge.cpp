// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"

#include "ros1_bridge/bridge.hpp"


int main(int argc, char * argv[])
{
//  std::vector<std::pair<std::string, std::string>> topics;
//
//  for(int i = 0; i < argc; ++i) {
//    std::cout << argv[i] << std::endl;
//  }
//   
//  for(int i = 1; i < argc; i += 2) {
//    topics.push_back(std::make_pair(std::string(argv[i]), std::string(argv[i+1])));
//  }
  std::string robot_name("default");
  if(argc == 2) {
    robot_name = std::string(argv[1]);
  }

  // ROS 1 node
  ros::init(argc, argv, "ros_bridge");
  ros::NodeHandle ros1_node;

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("ros_bridge");

  size_t queue_size = 10;

  //std::vector<ros1_bridge::BridgeHandles> handles;
  //for(auto &pair : topics) {
  //  handles.push_back(
  //    ros1_bridge::create_bidirectional_bridge(
  //      ros1_node, ros2_node, pair.second, pair.second, pair.first, queue_size)
  //  );
  //}

  std::vector<ros1_bridge::Bridge1to2Handles> out_handles;
  out_handles.push_back(
    ros1_bridge::create_bridge_from_1_to_2(
      ros1_node, ros2_node, std::string("geometry_msgs/PoseStamped"), std::string("/robot_pose"), queue_size,
      std::string("geometry_msgs/PoseStamped"), robot_name + std::string("/pose"), queue_size)
  );
  out_handles.push_back(
    ros1_bridge::create_bridge_from_1_to_2(
      ros1_node, ros2_node, std::string("std_msgs/Bool"), std::string("/switch_found"), queue_size,
      std::string("std_msgs/Bool"), robot_name + std::string("/switch_found"), queue_size)
  );
  out_handles.push_back(
    ros1_bridge::create_bridge_from_1_to_2(
      ros1_node, ros2_node, std::string("nav_msgs/OccupancyGrid"), std::string("/costmap_repeating"), queue_size,
      std::string("nav_msgs/OccupancyGrid"), robot_name + std::string("/costmap"), queue_size)
  );

  std::vector<ros1_bridge::Bridge2to1Handles> in_handles;
  in_handles.push_back(
    ros1_bridge::create_bridge_from_2_to_1(
      ros2_node, ros1_node, std::string("nav_msgs/Path"), robot_name + std::string("/path"), queue_size,
      std::string("nav_msgs/Path"), std::string("/path"), queue_size)
  );

  auto handles_feedback = ros1_bridge::create_bidirectional_bridge(
		  ros1_node, ros2_node, "std_msgs/String", "std_msgs/String", "/accept_feedback", queue_size);

  auto handles_time_coord = ros1_bridge::create_bidirectional_bridge(
		  ros1_node, ros2_node, "std_msgs/Time", "std_msgs/Time", "/time_coord", queue_size);

  // bridge one example topic
  // std::string topic_name = "chatter";
  // std::string ros1_type_name = "std_msgs/String";
  // std::string ros2_type_name = "std_msgs/String";
  // size_t queue_size = 10;

  // auto handles = ros1_bridge::create_bidirectional_bridge(
  //   ros1_node, ros2_node, ros1_type_name, ros2_type_name, topic_name, queue_size);

  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // ROS 2 spinning loop
  rclcpp::executors::SingleThreadedExecutor executor;
  while (ros1_node.ok() && rclcpp::ok()) {
    executor.spin_node_once(ros2_node, std::chrono::milliseconds(1000));
  }

  return 0;
}
