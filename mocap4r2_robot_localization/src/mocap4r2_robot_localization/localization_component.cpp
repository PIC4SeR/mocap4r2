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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <nav_msgs/msg/odometry.hpp>

#include <cmath>
#include <vector>

#include "mocap4r2_robot_localization/localization_component.hpp"
#include "mocap4r2_msgs/msg/rigid_body.hpp"

#include "rclcpp/rclcpp.hpp"

namespace mocap4r2_robot_localization
{

using std::placeholders::_1;
using std::placeholders::_2;

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions & options)
: Node("mocap4r2_gt", options),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

  declare_parameter<std::string>("root_frame", "world");
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("odom_frame", "odom");
  declare_parameter<std::string>("robot_frame", "base_footprint");
  declare_parameter<std::string>("mocap_frame", "base_mocap");
  declare_parameter<std::string>("rigid_body_topic", "rigid_bodies");
  declare_parameter<std::string>("odometry_topic", "odometry");
  declare_parameter<std::string>("odometry_filtered_topic", "odom_filtered");
  declare_parameter<std::string>("rigid_body_name", "robot");
  declare_parameter<double>("alpha", 0.1);
  declare_parameter<std::string>("velocity_filter", "ema");
  declare_parameter<double>("kalman_process_noise", 1.0);
  declare_parameter<double>("kalman_measurement_noise", 1e-2);
  declare_parameter<double>("kalman_gate", 0.0);
  declare_parameter<int>("kalman_max_coast", 10);
  declare_parameter<std::vector<double>>("init_root2map_xyzrpy", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  declare_parameter<std::vector<double>>("covariance.pose", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  declare_parameter<std::vector<double>>("covariance.twist", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  
  get_parameter("root_frame", root_frame_);
  get_parameter("map_frame", map_frame_);
  get_parameter("odom_frame", odom_frame_);
  get_parameter("robot_frame", robot_frame_);
  get_parameter("mocap_frame", mocap_frame_);
  get_parameter("rigid_body_topic", rigid_body_topic_);
  get_parameter("rigid_body_name", rigid_body_name_);
  get_parameter("alpha", alpha_);
  get_parameter("velocity_filter", velocity_filter_);
  get_parameter("kalman_process_noise", kalman_q_);
  get_parameter("kalman_measurement_noise", kalman_r_);
  get_parameter("kalman_gate", kalman_gate_);
  get_parameter("kalman_max_coast", kalman_max_coast_);
  kf_x_.set_noise(kalman_q_, kalman_r_);
  kf_y_.set_noise(kalman_q_, kalman_r_);
  kf_z_.set_noise(kalman_q_, kalman_r_);
  kf_yaw_.set_noise(kalman_q_, kalman_r_);
  get_parameter("odometry_topic", odometry_topic_);
  get_parameter("odometry_filtered_topic", odometry_filtered_topic_);
  get_parameter("covariance.pose", pose_covariance_);
  get_parameter("covariance.twist", twist_covariance_);

  rigid_body_sub_ = create_subscription<mocap4r2_msgs::msg::RigidBodies>(
    rigid_body_topic_, rclcpp::SensorDataQoS(), std::bind(&LocalizationNode::rigid_bodies_callback, this, _1));

  odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 10);
  odometry_filtered_pub_ =
    create_publisher<nav_msgs::msg::Odometry>(odometry_filtered_topic_, 10);

  std::vector<double> init_root2map_coordinates;
  get_parameter("init_root2map_xyzrpy", init_root2map_coordinates);

  if (init_root2map_coordinates.size() == 6u) {
    tf2::fromMsg(get_pose_from_vector(init_root2map_coordinates), root2map_);
  } else {
    RCLCPP_ERROR(get_logger(), "Error in init_mocap xyzrpy coordinates - setting all values to 0");
    tf2::fromMsg(get_pose_from_vector({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), root2map_);
  }
  // Send the static transform root to map
  if(root_frame_ != map_frame_)
  {
    geometry_msgs::msg::TransformStamped root2map_msg;
    root2map_msg.header.stamp = get_clock()->now();
    root2map_msg.header.frame_id = root_frame_;
    root2map_msg.child_frame_id = map_frame_;
    root2map_msg.transform = tf2::toMsg(root2map_);
    tf_static_broadcaster_->sendTransform(root2map_msg);
  }
  map2root_ = root2map_.inverse();
}

void
LocalizationNode::rigid_bodies_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
{
  if (!valid_mocap2robot_) { // Find mocap2robot (the mocap object to robot base_link)
    try {
      auto mocap2robot_msg = tf_buffer_.lookupTransform(
        mocap_frame_, robot_frame_, tf2::TimePointZero);
      tf2::fromMsg(mocap2robot_msg.transform, mocap2robot_);
      valid_mocap2robot_ = true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(
        get_logger(), "Transform %s->%s exception: [%s]",
        mocap_frame_.c_str(), robot_frame_.c_str(), e.what());
    }
  } else {
    // find index for which the rigid body name is the same as the one we are looking for
    auto robot_it = std::find_if(
      msg->rigidbodies.begin(), msg->rigidbodies.end(),
      [this](const mocap4r2_msgs::msg::RigidBody & rb) {
        return rb.rigid_body_name == rigid_body_name_;
      });
      
    mocap4r2_msgs::msg::RigidBody::SharedPtr valid_rb;

    RCLCPP_DEBUG(get_logger(), "pose x: %f, y: %f, z: %f", robot_it->pose.position.x, robot_it->pose.position.y, robot_it->pose.position.z);
    RCLCPP_DEBUG(get_logger(), "pose qx: %f, qy: %f, qz: %f, qw: %f", robot_it->pose.orientation.x, robot_it->pose.orientation.y, robot_it->pose.orientation.z, robot_it->pose.orientation.w);   
    RCLCPP_DEBUG(get_logger(), "is uninitialized: %d", this->is_uninitialized(robot_it->pose));
    if (robot_it != msg->rigidbodies.end() &&
      (this->is_uninitialized(robot_it->pose) == false))
    {
      RCLCPP_DEBUG(get_logger(), "Rigid body %s found in mocap system", rigid_body_name_.c_str());
      valid_rb = std::make_shared<mocap4r2_msgs::msg::RigidBody>(*robot_it);
      last_valid_rigid_body_ = valid_rb;
    }
    else if (
      this->is_uninitialized(robot_it->pose) && last_valid_rigid_body_ != nullptr
      )
    {
      RCLCPP_WARN(get_logger(), "Reusing last valid rigid body data for %s", rigid_body_name_.c_str());
      valid_rb = last_valid_rigid_body_;
    }
    else
    {
      RCLCPP_WARN(get_logger(), "Rigid body %s not found in mocap system", rigid_body_name_.c_str());
      return;
    }

    // Check if the mocap is publishing the robot pose and is not zero
    tf2::Quaternion q;
    tf2::fromMsg(valid_rb->pose.orientation, q);
    if (q.length2() < 1e-6) {
      RCLCPP_WARN(get_logger(), "Zero quaternion received from mocap system. Check that the robot is being tracked");
      return;
    }
    root2mocap_.setOrigin(tf2::Vector3(valid_rb->pose.position.x, valid_rb->pose.position.y, valid_rb->pose.position.z));
    root2mocap_.setRotation(q);

    tf2::Transform map2odom, map2robot;
    tf2::Transform mocap2odom;
    try {
      auto mocap2odom_msg = tf_buffer_.lookupTransform(
        mocap_frame_, odom_frame_, tf2::TimePointZero);
      tf2::fromMsg(mocap2odom_msg.transform, mocap2odom);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(
        get_logger(), "Transform %s->%s exception: [%s]",
        mocap_frame_.c_str(), odom_frame_.c_str(), e.what());
      return;
    }

    map2odom = map2root_ * root2mocap_ * mocap2odom;
    map2robot = map2root_ * root2mocap_ * mocap2robot_; 

    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.frame_id = map_frame_;
    odom_msg->header.stamp = msg->header.stamp;
    odom_msg->child_frame_id = robot_frame_;
    LocalizationNode::compute_odometry(map2robot, odom_msg);

    geometry_msgs::msg::TransformStamped map2odom_msg;
    map2odom_msg.header.frame_id = map_frame_;
    map2odom_msg.header.stamp = msg->header.stamp;
    map2odom_msg.child_frame_id = odom_frame_;
    map2odom_msg.transform = tf2::toMsg(map2odom);

    // Filtered topic: same odometry but with the coast-corrected pose.
    auto odom_filtered = std::make_unique<nav_msgs::msg::Odometry>(*odom_msg);
    odom_filtered->pose.pose = filtered_pose_;
    odometry_filtered_pub_->publish(std::move(odom_filtered));

    odometry_pub_->publish(std::move(odom_msg));
    tf_broadcaster_->sendTransform(map2odom_msg);
  }
}


geometry_msgs::msg::Pose
LocalizationNode::get_pose_from_vector(const std::vector<double> & init_pos)
{
  geometry_msgs::msg::Pose ret;

  if (init_pos.size() == 6u) {
    tf2::Quaternion q;
    q.setEuler(init_pos[3], init_pos[4], init_pos[5]);

    ret.position.x = init_pos[0];
    ret.position.y = init_pos[1];
    ret.position.z = init_pos[2];
    ret.orientation.x = q.x();
    ret.orientation.y = q.y();
    ret.orientation.z = q.z();
    ret.orientation.w = q.w();

    return ret;
  } else {
    RCLCPP_WARN(get_logger(), "Trying to get Pose for a wrong vector");
    return ret;
  }
}

void LocalizationNode::compute_odometry(
  const tf2::Transform & map2robot_tf,
  nav_msgs::msg::Odometry::UniquePtr & odom_msg)
{
  // Minimum dt (s) for a valid finite difference; guards against duplicate or
  // out-of-order mocap timestamps that would otherwise blow up the velocity.
  constexpr double kMinDt = 1e-4;

  // map-frame pose of the robot. The filtered topic uses this raw pose unless
  // the sample is gated, in which case it coasts on the prediction (set below).
  tf2::toMsg(map2robot_tf, odom_msg->pose.pose);
  filtered_pose_ = odom_msg->pose.pose;

  // Fill covariances (independent of the velocity estimate).
  for (size_t i = 0; i < 6; i++) {
    odom_msg->pose.covariance[i * 6 + i] = pose_covariance_[i];
    odom_msg->twist.covariance[i * 6 + i] = twist_covariance_[i];
  }

  const rclcpp::Time t2 = odom_msg->header.stamp;
  const tf2::Vector3 pos = map2robot_tf.getOrigin();
  const double yaw2 = tf2::getYaw(map2robot_tf.getRotation());

  // First sample: seed the previous pose / filters, publish zero velocity.
  if (!velocity_initialized_) {
    kf_x_.reset(pos.x());
    kf_y_.reset(pos.y());
    kf_z_.reset(pos.z());
    kf_yaw_.reset(yaw2);
    smoothed_twist_ = geometry_msgs::msg::Twist();
    odom_msg->twist.twist = smoothed_twist_;
    velocity_initialized_ = true;
    prev_pose_.header = odom_msg->header;
    prev_pose_.pose = odom_msg->pose.pose;
    return;
  }

  const rclcpp::Time t1 = prev_pose_.header.stamp;
  const double dt = (t2 - t1).seconds();

  if (dt <= kMinDt) {
    // Duplicate / out-of-order sample: re-publish the last estimate and keep
    // the previous pose so the next valid sample sees the correct interval.
    RCLCPP_WARN(
      get_logger(), "Non-positive dt (%.6f s) for robot odometry; reusing last velocity", dt);
    odom_msg->twist.twist = smoothed_twist_;
    return;
  }

  if (velocity_filter_ == "kalman") {
    // Constant-velocity Kalman on the map-frame position. Predict first, then
    // gate the (x, y) position innovation before correcting.
    kf_x_.predict(dt);
    kf_y_.predict(dt);
    kf_z_.predict(dt);
    kf_yaw_.predict(dt);
    const double yaw_ref = kf_yaw_.position();
    const double yaw_unwrapped = yaw_ref + std::remainder(yaw2 - yaw_ref, 2.0 * M_PI);

    // Mahalanobis gate on the 2-DOF (x, y) position innovation.
    const double d2 = kf_x_.innovation_sq(pos.x()) + kf_y_.innovation_sq(pos.y());
    const bool gating = kalman_gate_ > 0.0;
    if (gating && d2 > kalman_gate_ && coast_count_ < kalman_max_coast_) {
      ++coast_count_;  // reject outlier (swap/teleport): coast on the prediction
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Gated robot mocap update (d2=%.1f > %.1f); coasting (%d)",
        d2, kalman_gate_, coast_count_);
    } else {
      if (gating && coast_count_ >= kalman_max_coast_ && d2 > kalman_gate_) {
        // Persistent outlier: the target really moved -> re-acquire the track.
        kf_x_.reset(pos.x()); kf_y_.reset(pos.y());
        kf_z_.reset(pos.z()); kf_yaw_.reset(yaw2);
        RCLCPP_WARN(get_logger(), "Robot track re-acquired after %d gated samples",
                    coast_count_);
      } else {
        kf_x_.correct(pos.x()); kf_y_.correct(pos.y());
        kf_z_.correct(pos.z()); kf_yaw_.correct(yaw_unwrapped);
      }
      coast_count_ = 0;
    }

    // nav_msgs/Odometry twist is expressed in the child (robot) frame, so
    // rotate the map-frame linear velocity into the body frame.
    const tf2::Vector3 v_body = tf2::quatRotate(
      map2robot_tf.getRotation().inverse(),
      tf2::Vector3(kf_x_.velocity(), kf_y_.velocity(), kf_z_.velocity()));
    smoothed_twist_.linear.x = v_body.x();
    smoothed_twist_.linear.y = v_body.y();
    smoothed_twist_.linear.z = v_body.z();
    // Roll / pitch rates assumed negligible for a planar ground robot.
    smoothed_twist_.angular.x = 0.0;
    smoothed_twist_.angular.y = 0.0;
    smoothed_twist_.angular.z = kf_yaw_.velocity();
  } else {  // "ema": finite difference (body frame) + exponential moving average
    tf2::Transform p1;
    tf2::fromMsg(prev_pose_.pose, p1);
    const double dt_inv = 1.0 / dt;
    const tf2::Vector3 delta_trans = tf2::quatRotate(
      map2robot_tf.getRotation().inverse(), (map2robot_tf.getOrigin() - p1.getOrigin()));

    const double q1_w = p1.getRotation().w();
    const double q1_x = p1.getRotation().x();
    const double q1_y = p1.getRotation().y();
    const double q1_z = p1.getRotation().z();
    const double q2_w = map2robot_tf.getRotation().w();
    const double q2_x = map2robot_tf.getRotation().x();
    const double q2_y = map2robot_tf.getRotation().y();
    const double q2_z = map2robot_tf.getRotation().z();

    smoothed_twist_.linear.x = (1 - alpha_) * smoothed_twist_.linear.x +
      alpha_ * (delta_trans.x() * dt_inv);
    smoothed_twist_.linear.y = (1 - alpha_) * smoothed_twist_.linear.y +
      alpha_ * (delta_trans.y() * dt_inv);
    smoothed_twist_.linear.z = (1 - alpha_) * smoothed_twist_.linear.z +
      alpha_ * (delta_trans.z() * dt_inv);

    smoothed_twist_.angular.x = (1 - alpha_) * smoothed_twist_.angular.x + alpha_ * (
      2.0 * dt_inv * (q1_w * q2_x - q1_x * q2_w - q1_y * q2_z + q1_z * q2_y));
    smoothed_twist_.angular.y = (1 - alpha_) * smoothed_twist_.angular.y + alpha_ * (
      2.0 * dt_inv * (q1_w * q2_y + q1_x * q2_z - q1_y * q2_w - q1_z * q2_x));
    smoothed_twist_.angular.z = (1 - alpha_) * smoothed_twist_.angular.z + alpha_ * (
      2.0 * dt_inv * (q1_w * q2_z - q1_x * q2_y + q1_y * q2_x - q1_z * q2_w));
  }

  odom_msg->twist.twist = smoothed_twist_;

  // While coasting on a gated sample, the filtered topic reports the predicted
  // (map-frame) pose instead of the rejected raw measurement, so a swap/teleport
  // does not leak into the position. Roll/pitch are dropped (planar coast).
  if (velocity_filter_ == "kalman" && coast_count_ > 0) {
    tf2::Transform pred;
    pred.setOrigin(tf2::Vector3(kf_x_.position(), kf_y_.position(), kf_z_.position()));
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, kf_yaw_.position());
    pred.setRotation(q);
    tf2::toMsg(pred, filtered_pose_);
  }

  // Inflate the reported covariance while coasting on gated samples so
  // downstream consumers can down-weight a velocity that is being extrapolated.
  if (coast_count_ > 0) {
    const double infl = 1.0 + coast_count_;
    for (size_t i = 0; i < 6; i++) {
      odom_msg->pose.covariance[i * 6 + i] *= infl;
      odom_msg->twist.covariance[i * 6 + i] *= infl;
    }
  }

  // Advance the previous pose only after a successful update.
  prev_pose_.header = odom_msg->header;
  prev_pose_.pose = odom_msg->pose.pose;
}

}  // namespace mocap4r2_robot_localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mocap4r2_robot_localization::LocalizationNode)
