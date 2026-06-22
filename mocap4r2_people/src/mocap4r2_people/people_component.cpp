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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <cmath>
#include <string>
#include <vector>
#include <ranges>

#include "mocap4r2_people/people_component.hpp"
#include "mocap4r2_msgs/msg/rigid_body.hpp"

#include "rclcpp/rclcpp.hpp"

namespace mocap4r2_people
{

using std::placeholders::_1;
using std::placeholders::_2;

PeopleNode::PeopleNode(const rclcpp::NodeOptions & options)
: Node("mocap4r2_people", options),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

  declare_parameter<std::string>("root_frame", "mocap");
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("rigid_body_topic", "rigid_bodies");
  declare_parameter<std::string>("people_topic", "people");
  declare_parameter<std::string>("rigid_body_prefix", "person");
  declare_parameter<int>("tag.id", 0);
  declare_parameter<int>("tag.group_id", -1);
  declare_parameter<int>("tag.behaviour", 2);
  declare_parameter<double>("alpha", 0.1);
  declare_parameter<std::string>("velocity_filter", "ema");
  declare_parameter<double>("kalman_process_noise", 1.0);
  declare_parameter<double>("kalman_measurement_noise", 1e-4);
  declare_parameter<bool>("publish_map", true);
  declare_parameter<std::vector<double>>("init_root2map_xyzrpy", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  

  int id, group_id, behaviour;
  get_parameter("tag.id", id);
  get_parameter("tag.group_id", group_id);
  get_parameter("tag.behaviour", behaviour);
  tags_["id"] = std::to_string(id);
  tags_["group_id"] = std::to_string(group_id);
  tags_["behaviour"] = std::to_string(behaviour);

  get_parameter("root_frame", root_frame_);
  get_parameter("map_frame", map_frame_);
  get_parameter("rigid_body_topic", rigid_body_topic_);
  get_parameter("rigid_body_prefix", rigid_body_prefix_);
  get_parameter("people_topic", people_topic_);
  get_parameter("alpha", alpha_);
  get_parameter("velocity_filter", velocity_filter_);
  get_parameter("kalman_process_noise", kalman_q_);
  get_parameter("kalman_measurement_noise", kalman_r_);
  get_parameter("publish_map", publish_map_);
  std::vector<double> init_root2map_coordinates;
  get_parameter("init_root2map_xyzrpy", init_root2map_coordinates);

  if(publish_map_)
  {
    if (init_root2map_coordinates.size() == 6u) {
    tf2::fromMsg(get_pose_from_vector(init_root2map_coordinates), root2map_);
    } else {
      RCLCPP_ERROR(get_logger(), "Error in init_mocap xyzrpy coordinates - setting all values to 0");
      tf2::fromMsg(get_pose_from_vector({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), root2map_);
    }
    // Send the static transform root to map
    if(root_frame_ != map_frame_)
    {
      root2map_msg_.header.stamp = get_clock()->now();
      root2map_msg_.header.frame_id = root_frame_;
      root2map_msg_.child_frame_id = map_frame_;
      root2map_msg_.transform = tf2::toMsg(root2map_);
    }
  }

  rigid_body_sub_ = create_subscription<mocap4r2_msgs::msg::RigidBodies>(
    rigid_body_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PeopleNode::rigid_bodies_callback, this, _1));

  people_pub_ = create_publisher<people_msgs::msg::People>(people_topic_, 10);
  pose_array_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("pose_array", 10);
  valid_map2root_ = map_frame_ == root_frame_;
  last_valid_people_bodies_ = std::make_shared<mocap4r2_msgs::msg::RigidBodies>();
}

void
PeopleNode::rigid_bodies_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
{
  if (!valid_map2root_) { // Find root2map (the mocap system origin in the map frame)
    try {
      auto map2root_msg = tf_buffer_.lookupTransform(
        map_frame_, root_frame_, tf2::TimePointZero);
      tf2::fromMsg(map2root_msg.transform, map2root_);
      valid_map2root_ = true;
    } catch (const tf2::TransformException & e) {
      if(publish_map_){
        tf_static_broadcaster_->sendTransform(root2map_msg_);
        map2root_ = root2map_.inverse();
        valid_map2root_ = true;
      }
      RCLCPP_WARN(
        get_logger(), "Transform %s->%s exception: [%s]",
        root_frame_.c_str(), map_frame_.c_str(), e.what());
    }
  } else {
    // filter out the non valid elements
    auto people = std::ranges::filter_view(
      msg->rigidbodies, [this](const mocap4r2_msgs::msg::RigidBody & rb) {
        return rb.rigid_body_name.find(rigid_body_prefix_) != std::string::npos;
      });
    
    // mocap4r2_msgs::msg::RigidBodies::SharedPtr valid_people = std::make_shared<mocap4r2_msgs::msg::RigidBodies>();

    
    for (const auto & person : people) {
      // Check if the mocap is publishing the person pose and is not zero  
      tf2::Quaternion q;
      tf2::fromMsg(person.pose.orientation, q);
      if (q.length2() < 1e-6 ) {
        RCLCPP_WARN(
          get_logger(),
          "Zero quaternion received from mocap system. Check that the person is being tracked");
        continue;
      }
      // obtain the person index in valid_people
      auto person_it = std::find_if(
        last_valid_people_bodies_->rigidbodies.begin(), last_valid_people_bodies_->rigidbodies.end(),
        [this, &person](const mocap4r2_msgs::msg::RigidBody & rb) {
          return rb.rigid_body_name == person.rigid_body_name;
        });

      if (person_it == last_valid_people_bodies_->rigidbodies.end()) {
        last_valid_people_bodies_->rigidbodies.push_back(person);
      } else {
        *person_it = person;
      }
      
    }

    if (last_valid_people_bodies_->rigidbodies.empty()) {
      RCLCPP_WARN(get_logger(), "No people found in mocap system");
      return;
    }


    auto people_msg = std::make_unique<people_msgs::msg::People>();
    auto pose_array_msg = std::make_unique<geometry_msgs::msg::PoseArray>();

    for (const auto & person : last_valid_people_bodies_->rigidbodies) {
      // Check if the mocap is publishing the person pose and is not zero  
      tf2::Quaternion q;
      tf2::fromMsg(person.pose.orientation, q);
      if (q.length2() < 1e-6) {
        RCLCPP_WARN(
          get_logger(),
          "Zero quaternion received from mocap system. Check that the person is being tracked");
        continue;
      }
      // obtain the root2person transform

      tf2::Transform root2person;
      root2person.setOrigin(
        tf2::Vector3(
          person.pose.position.x, person.pose.position.y,
          person.pose.position.z));
      root2person.setRotation(q);

      RCLCPP_DEBUG(
          get_logger(),
          "pose of the person in vicon frame %s \n x: %f \ty: %f \ttheta: %f", person.rigid_body_name.c_str(), 
          person.pose.position.x, person.pose.position.y,
          person.pose.position.z);

      // obtain the map2person transform
      tf2::Transform map2person = map2root_ * root2person;

      //consider markers instead of pose_array
      auto person_pose = std::make_unique<geometry_msgs::msg::Pose>();
      tf2::toMsg(map2person, *person_pose);

      geometry_msgs::msg::TransformStamped person_pose_msg;
      person_pose_msg.header = msg->header;
      person_pose_msg.header.frame_id = map_frame_;
      person_pose_msg.child_frame_id = person.rigid_body_name;
      person_pose_msg.transform = tf2::toMsg(map2person);

      tf_broadcaster_->sendTransform(person_pose_msg);

      auto person_msg = std::make_unique<people_msgs::msg::Person>();
      fill_person_msg(person.rigid_body_name, *person_pose, msg->header, person_msg);

      pose_array_msg->poses.push_back(std::move(*person_pose));
      people_msg->people.push_back(std::move(*person_msg));
    }
    pose_array_msg->header = msg->header;
    pose_array_msg->header.frame_id = map_frame_;
    people_msg->header = msg->header;
    people_msg->header.frame_id = map_frame_;

    people_pub_->publish(std::move(people_msg));
    pose_array_pub_->publish(std::move(pose_array_msg));
  }
}

void PeopleNode::fill_person_msg(
  const std::string & person_name,
  const geometry_msgs::msg::Pose & pose,
  const std_msgs::msg::Header & header,
  people_msgs::msg::Person::UniquePtr & person_msg)
{
  person_msg->name = person_name;
  person_msg->reliability = 1.0;

  // fill the pose of the person: x, y, yaw
  person_msg->position.x = pose.position.x;
  person_msg->position.y = pose.position.y;
  // get the yaw from the quaternion
  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);
  person_msg->position.z = tf2::getYaw(q);

  // fill the velocity: vx, vy, vtheta
  compute_velocity(pose, header, person_msg);

  // fill the tags
  for (const auto & tag : tags_) {
    person_msg->tagnames.push_back(tag.first);
    if (tag.first == "id") {
      std::string s = person_name;
      std::string delimiter = "_";
      size_t pos = 0;
      std::string token;
      while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);
        s.erase(0, pos + delimiter.length());
      }
      person_msg->tags.push_back(s);
      continue;
    }
    person_msg->tags.push_back(tag.second);
  }
}

void PeopleNode::compute_velocity(
  const geometry_msgs::msg::Pose & person_pose,
  const std_msgs::msg::Header & header,
  people_msgs::msg::Person::UniquePtr & person_msg)
{
  // Minimum dt (s) for a valid finite difference; guards against duplicate or
  // out-of-order mocap timestamps that would otherwise blow up the velocity.
  constexpr double kMinDt = 1e-4;

  const std::string & name = person_msg->name;
  const rclcpp::Time t2 = header.stamp;

  tf2::Quaternion q2;
  tf2::fromMsg(person_pose.orientation, q2);
  const double yaw2 = tf2::getYaw(q2);

  // First time we see this person: seed the previous pose / filters and
  // publish zero velocity (no baseline to differentiate against yet).
  auto prev_it = prev_poses_.find(name);
  if (prev_it == prev_poses_.end()) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose = person_pose;
    ps.header = header;
    prev_poses_[name] = ps;

    PersonKalman & kf = kalman_[name];
    kf.x.set_noise(kalman_q_, kalman_r_);
    kf.y.set_noise(kalman_q_, kalman_r_);
    kf.yaw.set_noise(kalman_q_, kalman_r_);
    kf.x.reset(person_pose.position.x);
    kf.y.reset(person_pose.position.y);
    kf.yaw.reset(yaw2);

    smoothed_twists_[name] = geometry_msgs::msg::Twist();
    person_msg->velocity.x = 0.0;
    person_msg->velocity.y = 0.0;
    person_msg->velocity.z = 0.0;
    return;
  }

  const rclcpp::Time t1 = prev_it->second.header.stamp;
  const double dt = (t2 - t1).seconds();

  geometry_msgs::msg::Twist & twist = smoothed_twists_[name];

  if (dt <= kMinDt) {
    // Duplicate / out-of-order sample: re-publish the last estimate and keep
    // the previous pose so the next valid sample sees the correct interval.
    RCLCPP_WARN(
      get_logger(), "Non-positive dt (%.6f s) for person %s; reusing last velocity",
      dt, name.c_str());
    person_msg->velocity.x = twist.linear.x;
    person_msg->velocity.y = twist.linear.y;
    person_msg->velocity.z = twist.angular.z;
    return;
  }

  if (velocity_filter_ == "kalman") {
    PersonKalman & kf = kalman_[name];
    twist.linear.x = kf.x.update(person_pose.position.x, dt);
    twist.linear.y = kf.y.update(person_pose.position.y, dt);
    // Unwrap the yaw measurement onto the filter's current estimate so the
    // constant-velocity model sees a continuous angle across +/-pi.
    const double yaw_ref = kf.yaw.position();
    const double yaw_unwrapped = yaw_ref + std::remainder(yaw2 - yaw_ref, 2.0 * M_PI);
    twist.angular.z = kf.yaw.update(yaw_unwrapped, dt);
  } else {  // "ema": finite difference + per-person exponential moving average
    tf2::Transform p1;
    tf2::fromMsg(prev_it->second.pose, p1);
    const double dt_inv = 1.0 / dt;
    const tf2::Vector3 delta_trans = tf2::Vector3(
      person_pose.position.x, person_pose.position.y, person_pose.position.z) - p1.getOrigin();

    const double q1_w = p1.getRotation().w();
    const double q1_x = p1.getRotation().x();
    const double q1_y = p1.getRotation().y();
    const double q1_z = p1.getRotation().z();
    const double raw_wz = 2.0 * dt_inv *
      (q1_w * q2.z() - q1_x * q2.y() + q1_y * q2.x() - q1_z * q2.w());

    twist.linear.x = (1 - alpha_) * twist.linear.x + alpha_ * (delta_trans.x() * dt_inv);
    twist.linear.y = (1 - alpha_) * twist.linear.y + alpha_ * (delta_trans.y() * dt_inv);
    twist.angular.z = (1 - alpha_) * twist.angular.z + alpha_ * raw_wz;
  }

  person_msg->velocity.x = twist.linear.x;
  person_msg->velocity.y = twist.linear.y;
  person_msg->velocity.z = twist.angular.z;

  // Advance the previous pose only after a successful update.
  prev_it->second.pose = person_pose;
  prev_it->second.header = header;
}

geometry_msgs::msg::Pose
PeopleNode::get_pose_from_vector(const std::vector<double> & init_pos)
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

}  // namespace mocap4r2_people

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mocap4r2_people::PeopleNode)
