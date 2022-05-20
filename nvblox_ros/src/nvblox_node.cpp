/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <glog/logging.h>

#include <memory>

#include <ros/ros.h>

#include <nvblox/core/cuda/warmup.h>

#include "nvblox_ros/nvblox_node.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::InstallFailureSignalHandler();
  ros::init(argc, argv, "nvblox_node");

  // Warmup CUDA so it doesn't affect our timings *as* much for the first
  // CUDA call.
  nvblox::warmupCuda();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  nvblox::NvbloxNode node(nh, nh_private);
  ros::spin();
  ros::shutdown();
  return 0;
}
