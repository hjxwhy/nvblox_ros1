/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "nvblox_ros/transformer.hpp"

#include <algorithm>
#include <memory>
#include <string>

namespace nvblox
{

Transformer::Transformer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
: nh_(nh), nh_private_(nh_private)
{
  // Get params like "use_tf_transforms".
  use_tf_transforms_ = nh_private_.param("use_tf_transforms", use_tf_transforms_);
  use_topic_transforms_ = nh_private_.param("use_topic_transforms",use_topic_transforms_);

  // Init the transform listeners if we ARE using TF at all.
  if (use_tf_transforms_) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(ros::Duration());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
}

bool Transformer::lookupTransformToGlobalFrame(
  const std::string & sensor_frame,
  const ros::Time & timestamp,
  Transform * transform)
{
  if (!use_tf_transforms_ && !use_topic_transforms_) {
    // ERROR HERE, literally can't do anything.
    ROS_DEBUG(
      "Not using TF OR topic transforms, what do you want us to use?");
    return false;
  }

  // Check if we have a transform queue.
  if (!use_topic_transforms_) {
    // Then I guess we're using TF.
    // Try to look up the pose in TF.
    return lookupTransformTf(global_frame_, sensor_frame, timestamp, transform);
  } else {
    // We're using topic transforms.
    if (sensor_frame != pose_frame_) {
      // Ok but we also need a pose_frame -> sensor_frame lookup here.
      Transform T_P_S;
      if (!lookupSensorTransform(sensor_frame, &T_P_S)) {
        return false;
      }

      // Now we have the TPS reports, we need T_G_P which is global to pose
      // frame. This comes from the queue.
      Transform T_G_P;
      if (!lookupTransformQueue(timestamp, &T_G_P)) {
        return false;
      }

      *transform = T_G_P * T_P_S;
      return true;
    } else {
      return lookupTransformQueue(timestamp, transform);
    }
  }
  return false;
}

void Transformer::transformCallback(
  const geometry_msgs::TransformStamped::ConstPtr transform_msg)
{
  ros::Time timestamp = transform_msg->header.stamp;
  transform_queue_[timestamp.toSec()] =
    tf2::transformToEigen(*transform_msg).matrix().cast<float>();
}

void Transformer::poseCallback(
  const geometry_msgs::PoseStamped::ConstPtr transform_msg)
{
  ros::Time timestamp = transform_msg->header.stamp;
  Eigen::Affine3d T_G_P_double;
  Eigen::fromMsg(transform_msg->pose, T_G_P_double);
  transform_queue_[timestamp.toSec()] =
    T_G_P_double.matrix().cast<float>();
}

bool Transformer::lookupTransformTf(
  const std::string & from_frame,
  const std::string & to_frame,
  const ros::Time & timestamp,
  Transform * transform)
{
  geometry_msgs::TransformStamped T_S_C_msg;
  try {
    if (tf_buffer_->canTransform(from_frame, to_frame, timestamp)) {
      T_S_C_msg = tf_buffer_->lookupTransform(from_frame, to_frame, timestamp);
    } else {
      return false;
    }
  } catch (tf2::TransformException & e) {
    return false;
  }

  *transform = tf2::transformToEigen(T_S_C_msg).matrix().cast<float>();
  return true;
}

bool Transformer::lookupTransformQueue(const ros::Time & timestamp, Transform * transform)
{
  uint64_t timestamp_ns = timestamp.toNSec();

  auto closest_match = transform_queue_.lower_bound(timestamp_ns);
  if (closest_match == transform_queue_.end()) {
    return false;
  }

  // If we're too far off on the timestamp:
  uint64_t distance = std::max(closest_match->first, timestamp_ns) -
    std::min(closest_match->first, timestamp_ns);
  if (distance > timestamp_tolerance_ns_) {
    return false;
  }

  // We just do nearest neighbor here.
  // TODO(holeynikova): add interpolation!
  *transform = closest_match->second;
  return true;
}

bool Transformer::lookupSensorTransform(
  const std::string & sensor_frame,
  Transform * transform)
{
  auto it = sensor_transforms_.find(sensor_frame);
  if (it == sensor_transforms_.end()) {
    // Couldn't find sensor transform. Gotta look it up.
    if (!use_tf_transforms_) {
      // Well we're kind out out of options here.
      return false;
    }
    bool success = lookupTransformTf(
      pose_frame_, sensor_frame,
      ros::Time::now(), transform);
    if (success) {
      sensor_transforms_[sensor_frame] = *transform;
    } else {
      ROS_INFO(
        "Could not look up transform to sensor.");
    }
    return success;
  } else {
    *transform = it->second;
    return true;
  }
}

}  // namespace nvblox
