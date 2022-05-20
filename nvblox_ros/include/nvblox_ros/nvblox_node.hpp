/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef NVBLOX_ROS__NVBLOX_NODE_HPP_
#define NVBLOX_ROS__NVBLOX_NODE_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
// #include <tf2_eigen/tf2_eigen.h>

#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <nvblox/nvblox.h>

#include "nvblox_ros/conversions.hpp"
#include "nvblox_ros/transformer.hpp"

namespace nvblox
{

class NvbloxNode
{
public:
  NvbloxNode(const ros::NodeHandle& nh,
             const ros::NodeHandle& nh_private);
  virtual ~NvbloxNode() = default;

  // Callback functions. These just stick images in a queue.
  void depthImageCallback(
    const sensor_msgs::Image::ConstPtr & depth_img_ptr,
    const sensor_msgs::CameraInfo::ConstPtr & camera_info_msg);
  void colorImageCallback(
    const sensor_msgs::Image::ConstPtr & color_img_ptr,
    const sensor_msgs::CameraInfo::ConstPtr & color_info_msg);

  bool savePly(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response);

  // Does whatever processing there is to be done, depending on what
  // transforms are available.
  void processDepthQueue();
  void processColorQueue();

  // Process a single images
  virtual bool processDepthImage(
    sensor_msgs::Image::ConstPtr & depth_img_ptr,
    sensor_msgs::CameraInfo::ConstPtr & camera_info_msg);
  virtual bool processColorImage(
    sensor_msgs::Image::ConstPtr & color_img_ptr,
    sensor_msgs::CameraInfo::ConstPtr & camera_info_msg);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Helper functions to make the code more readable.
  void updateEsdf(const ros::Time & timestamp);
  void updateMesh(const ros::Time & timestamp);

  // ROS publishers and subscribers

  // Transformer to handle... everything, let's be honest.
  Transformer transformer_;

  // Time Sync
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::CameraInfo>
    time_policy_t;

  // Depth sub.
  std::shared_ptr<message_filters::Synchronizer<time_policy_t>> timesync_depth_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_camera_info_sub_;

  // Color sub
  std::shared_ptr<message_filters::Synchronizer<time_policy_t>> timesync_color_;
  message_filters::Subscriber<sensor_msgs::Image> color_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> color_camera_info_sub_;

  // Optional transform subs.
  ros::Subscriber transform_sub_;
  ros::Subscriber pose_sub_;

  // Publishers
  ros::Publisher  mesh_publisher_;
  ros::Publisher pointcloud_publisher_;
  ros::Publisher map_slice_publisher_;
  ros::ServiceServer save_ply_service_;

  // ROS & nvblox settings
  float voxel_size_ = 0.05f;
  bool esdf_ = true;
  bool esdf_2d_ = false;
  bool distance_slice_ = true;
  bool mesh_ = true;
  float slice_height_ = 0.5f;

  // Used for ESDF slicing. Everything between min and max height will be
  // compressed to a single 2D level, output at slice_height_.
  float min_height_ = 0.0f;
  float max_height_ = 2.0f;

  // ROS settings & update throttles
  std::string global_frame_ = "map";
  /// Pose frame to use if using transform topics.
  std::string pose_frame_ = "base_link";
  float max_tsdf_update_hz_ = 10.0f;
  float max_color_update_hz_ = 5.0f;
  float max_mesh_update_hz_ = 5.0f;
  float max_esdf_update_hz_ = 2.0f;

  // Mapper
  // Holds the map layers and their associated integrators
  // - TsdfLayer, ColorLayer, EsdfLayer, MeshLayer
  std::unique_ptr<RgbdMapper> mapper_;

  // The most important part: the ROS converter. Just holds buffers as state.
  RosConverter converter_;

  // Caches for GPU images
  ColorImage color_image_;
  DepthImage depth_image_;

  // Output directory
  std::string output_dir_ = "";

  // State for integrators running at various speeds.
  ros::Time last_tsdf_update_time_;
  ros::Time last_color_update_time_;
  ros::Time last_esdf_update_time_;
  ros::Time last_mesh_update_time_;

  // Cache the last known number of subscribers.
  size_t mesh_subscriber_count_ = 0;

  // Image queues.
  std::deque<std::pair<sensor_msgs::Image::ConstPtr,
    sensor_msgs::CameraInfo::ConstPtr>>
  depth_image_queue_;
  std::deque<std::pair<sensor_msgs::Image::ConstPtr,
    sensor_msgs::CameraInfo::ConstPtr>>
  color_image_queue_;
};

}  // namespace nvblox

#endif  // NVBLOX_ROS__NVBLOX_NODE_HPP_
