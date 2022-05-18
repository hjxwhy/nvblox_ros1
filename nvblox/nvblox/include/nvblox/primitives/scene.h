/*
Copyright 2022 NVIDIA CORPORATION

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

---- Original voxblox license, which this file is heavily based on: ----
Copyright (c) 2016, ETHZ ASL
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of voxblox nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once

#include <memory>
#include <vector>

#include "nvblox/core/blox.h"
#include "nvblox/core/camera.h"
#include "nvblox/core/image.h"
#include "nvblox/core/layer.h"
#include "nvblox/primitives/primitives.h"

namespace nvblox {
namespace primitives {

class Scene {
 public:
  Scene();

  /// === Creating an environment ===
  void addPrimitive(std::unique_ptr<Primitive> primitive);

  /// Convenience functions for setting up bounded areas.
  void addGroundLevel(float height);
  void addCeiling(float height);

  /// Add 4 walls (infinite planes) bounding the space. In case this is not the
  /// desired behavior, can use addObject to add walls manually one by one.
  /// If infinite walls are undesirable, then use cubes.
  void addPlaneBoundaries(float x_min, float x_max, float y_min, float y_max);

  /// Deletes all objects!
  void clear();

  /// === Generating synthetic data from environment ===
  /// Generates a synthetic view given camera parameters and a transformation
  /// of the camera to the scene.
  void generateDepthImageFromScene(const Camera& camera, const Transform& T_S_C,
                                   float max_dist,
                                   DepthImage* depth_frame) const;

  /// === Computing ground truth SDFs ===
  template <typename VoxelType>
  void generateSdfFromScene(float max_dist,
                            VoxelBlockLayer<VoxelType>* layer) const;

  /// Computes distance to an arbitrary point across all objects.
  /// Positive distance is computed only up to max_dist, though negative
  /// distance may be smaller than -max_dist.
  float getSignedDistanceToPoint(const Vector3f& coords, float max_dist) const;

  /// Get the intersection of a ray with the first hit object in the scene.
  bool getRayIntersection(const Vector3f& ray_origin,
                          const Vector3f& ray_direction, float max_dist,
                          Vector3f* ray_intersection, float* ray_dist) const;

  const AxisAlignedBoundingBox& aabb() const { return aabb_; }
  AxisAlignedBoundingBox& aabb() { return aabb_; }

 protected:
  template <typename VoxelType>
  inline void setVoxel(float dist, VoxelType* voxel) const;

  /// Vector storing pointers to all the objects in this world.
  std::vector<std::unique_ptr<Primitive>> primitives_;

  /// World boundaries... Can be changed arbitrarily, just sets ground truth
  /// generation and visualization bounds, accurate only up to block size.
  AxisAlignedBoundingBox aabb_;
};

}  // namespace primitives
}  // namespace nvblox

#include "nvblox/primitives/impl/scene_impl.h"
