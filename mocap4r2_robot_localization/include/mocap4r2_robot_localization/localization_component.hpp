// Copyright 2022 Intelligent Robotics Lab
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

// Author: Jose Miguel Guerrero Hernandez <josemiguel.guerrero@urjc.es>

#ifndef MOCAP4R2_ROBOT_LOCALIZATION__LOCALIZATION_COMPONENT_HPP_
#define MOCAP4R2_ROBOT_LOCALIZATION__LOCALIZATION_COMPONENT_HPP_


#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <vector>
#include <boost/optional.hpp>

#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include "rclcpp/rclcpp.hpp"


namespace mocap4r2_robot_localization
{
class LocalizationNode : public rclcpp::Node
{
public:
  explicit LocalizationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  void rigid_bodies_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg);

  geometry_msgs::msg::Pose get_pose_from_vector(const std::vector<double> & init_pos);

  inline bool is_uninitialized(const geometry_msgs::msg::Pose & pose)
  {
    return pose.position.x == 0.0 && pose.position.y == 0.0 && pose.position.z == 0.0 &&
           pose.orientation.x == 0.0 && pose.orientation.y == 0.0 && pose.orientation.z == 0.0 &&
           pose.orientation.w == 0.0;
  }

  void compute_odometry(
    const tf2::Transform & root2robot_tf,
    nav_msgs::msg::Odometry::UniquePtr & odom_msg);

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr rigid_body_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  std::string root_frame_;
  std::string robot_frame_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string mocap_frame_;
  nav_msgs::msg::Odometry::SharedPtr last_valid_odom_ = nullptr;
  rclcpp::Time last_msg_time_;
   // Timeout period (e.g. 2 seconds)
  rclcpp::TimerBase::SharedPtr odom_timer_;

  std::string rigid_body_topic_;
  std::string rigid_body_name_;
  std::string odometry_topic_;
  mocap4r2_msgs::msg::RigidBody::SharedPtr last_valid_rigid_body_;

  tf2::Transform root2map_, map2root_; // root is the frame of the mocap system (in this case the origin of the vicon system)
  tf2::Transform mocap2robot_; // mocap is the frame of the mocap object robot is the base_link of the robot
  tf2::Transform root2mocap_;  // root is the frame of the mocap system (in this case the origin of the vicon system)

  geometry_msgs::msg::PoseStamped prev_pose_;
  double alpha_;

  bool valid_mocap2robot_{false};

  std::vector<double> pose_covariance_;
  std::vector<double> twist_covariance_;
};

}  // namespace mocap4r2_robot_localization

#endif  // MOCAP4R2_ROBOT_LOCALIZATION__LOCALIZATION_COMPONENT_HPP_
