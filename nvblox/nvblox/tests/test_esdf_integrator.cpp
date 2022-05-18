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
*/
#include <gtest/gtest.h>
#include <string>

#include "nvblox/core/cuda/warmup.h"
#include "nvblox/core/indexing.h"
#include "nvblox/core/layer.h"
#include "nvblox/core/types.h"
#include "nvblox/core/voxels.h"
#include "nvblox/integrators/esdf_integrator.h"
#include "nvblox/integrators/projective_tsdf_integrator.h"
#include "nvblox/io/ply_writer.h"
#include "nvblox/io/pointcloud_io.h"
#include "nvblox/primitives/scene.h"

#include "nvblox/tests/utils.h"

using namespace nvblox;

constexpr float kFloatEpsilon = 1e-4;

// Have various obstacles that the ESDF can try to navigate around.
enum class Obstacle {
  kAxisAlignedPlane,  // 0
  kAngledPlane,       // 1
  kSphereOrigin,      // 2
  kBox,               // 3
  kBoxWithSphere,     // 4
  kBoxWithCube        // 5
};

class EsdfIntegratorTest : public ::testing::TestWithParam<Obstacle> {
 protected:
  void SetUp() override;

  void addParameterizedObstacleToScene(const Obstacle& obstacle);

  bool outputFlatSliceEsdfAsPly(const EsdfLayer& layer,
                                const std::string& ply_path, float height);
  bool outputFlatSliceTsdfAsPly(const TsdfLayer& layer,
                                const std::string& ply_path, float height);

  // Returns PERCENTAGE ABOVE THRESHOLD
  float compareEsdfToGt(const EsdfLayer& esdf_layer, const TsdfLayer& gt_layer,
                        float error_threshold);

  // Returns PERCENTAGE ABOVE THRESHOLD
  float compareEsdfToEsdf(const EsdfLayer& esdf_layer,
                          const EsdfLayer& gt_layer, float error_threshold);

  // Returns if all the voxels in the ESDF are valid.
  bool validateEsdf(const EsdfLayer& esdf_layer,
                    float max_squared_distance_vox);

  bool output_pointclouds_ = false;
  float block_size_;
  float voxel_size_ = 0.10f;
  float max_distance_ = 4.0f;
  float very_small_cutoff_ = 2e-3f;
  float small_cutoff_ = 2e-2f;

  TsdfLayer::Ptr tsdf_layer_;
  TsdfLayer::Ptr gt_sdf_layer_;
  EsdfLayer::Ptr esdf_layer_;

  EsdfIntegrator esdf_integrator_;

  // A simulation scene.
  primitives::Scene scene_;

  // Camera and TSDF parameters for incremental integration.
  std::shared_ptr<Camera> camera_;
  ProjectiveTsdfIntegrator tsdf_integrator_;
};

void EsdfIntegratorTest::SetUp() {
  timing::Timing::Reset();
  std::srand(0);
  block_size_ = VoxelBlock<bool>::kVoxelsPerSide * voxel_size_;

  tsdf_layer_.reset(new TsdfLayer(voxel_size_, MemoryType::kUnified));
  gt_sdf_layer_.reset(new TsdfLayer(voxel_size_, MemoryType::kUnified));
  esdf_layer_.reset(new EsdfLayer(voxel_size_, MemoryType::kUnified));

  esdf_integrator_.max_distance_m() = max_distance_;
  esdf_integrator_.min_weight() = 1.0f;

  camera_.reset(new Camera(300, 300, 320, 240, 640, 480));
}

void EsdfIntegratorTest::addParameterizedObstacleToScene(
    const Obstacle& obstacle) {
  if (obstacle == Obstacle::kBox || obstacle == Obstacle::kBoxWithCube ||
      obstacle == Obstacle::kBoxWithSphere) {
    scene_.aabb() = AxisAlignedBoundingBox(Vector3f(-5.5f, -5.5f, -0.5f),
                                           Vector3f(5.5f, 5.5f, 5.5f));
    scene_.addPlaneBoundaries(-5.0f, 5.0f, -5.0f, 5.0f);
    scene_.addGroundLevel(0.0f);
    scene_.addCeiling(5.0f);
  } else {
    scene_.aabb() = AxisAlignedBoundingBox(Vector3f(-3.0f, -3.0f, 0.0f),
                                           Vector3f(3.0f, 3.0f, 3.0f));
  }

  switch (obstacle) {
    case Obstacle::kAxisAlignedPlane:
      // Plane at the origin pointing in the -x direction.
      scene_.addPrimitive(std::make_unique<primitives::Plane>(
          Vector3f(0.05, 0.05, 0.05), Vector3f(-1, 0, 0).normalized()));
      break;
    case Obstacle::kAngledPlane:
      // Plane at the origin pointing in the -x -y direction.
      scene_.addPrimitive(std::make_unique<primitives::Plane>(
          Vector3f(0.0, 0.0, 0.0), Vector3f(1, 1, 0).normalized()));
      break;
    case Obstacle::kSphereOrigin:
      // Create a scene that's just a sphere
      scene_.addPrimitive(
          std::make_unique<primitives::Sphere>(Vector3f(0.0, 0.0, 0.0), 2.0));
      break;
    case Obstacle::kBoxWithSphere:
      scene_.addPrimitive(std::make_unique<primitives::Sphere>(
          Vector3f(0.0f, 0.0f, 2.0f), 2.0f));
      break;
    case Obstacle::kBoxWithCube:
      scene_.addPrimitive(std::make_unique<primitives::Cube>(
          Vector3f(0.0f, 0.0f, 2.0f), Vector3f(2.0f, 2.0f, 2.0f)));
      break;
    default:
      break;
  };
}

bool EsdfIntegratorTest::outputFlatSliceEsdfAsPly(const EsdfLayer& layer,
                                                  const std::string& ply_path,
                                                  float height) {
  // Create a ply writer object.
  io::PlyWriter writer(ply_path);

  // Combine all the voxels in the mesh into a pointcloud.
  std::vector<Vector3f> points;
  std::vector<float> distances;

  const float block_size = layer.block_size();
  const float voxel_size = layer.voxel_size();

  auto lambda = [&points, &distances, &block_size, &voxel_size, &height](
                    const Index3D& block_index, const Index3D& voxel_index,
                    const EsdfVoxel* voxel) {
    if (voxel->observed) {
      Vector3f position = getCenterPostionFromBlockIndexAndVoxelIndex(
          block_size, block_index, voxel_index);
      if (position.z() - height < voxel_size && position.z() >= height) {
        points.push_back(position);
        float distance = voxel_size * std::sqrt(voxel->squared_distance_vox);
        if (voxel->is_inside) {
          distance = -distance;
        }
        distances.push_back(distance);
      }
    }
  };

  // Call above lambda on every voxel in the layer.
  callFunctionOnAllVoxels<EsdfVoxel>(layer, lambda);

  // Add the pointcloud to the ply writer.
  writer.setPoints(&points);
  writer.setIntensities(&distances);

  // Write out the ply.
  return writer.write();
}

bool EsdfIntegratorTest::outputFlatSliceTsdfAsPly(const TsdfLayer& layer,
                                                  const std::string& ply_path,
                                                  float height) {
  // Create a ply writer object.
  io::PlyWriter writer(ply_path);

  // Combine all the voxels in the mesh into a pointcloud.
  std::vector<Vector3f> points;
  std::vector<float> distances;

  const float block_size = layer.block_size();
  const float voxel_size = layer.voxel_size();

  auto lambda = [&points, &distances, &block_size, &voxel_size, &height](
                    const Index3D& block_index, const Index3D& voxel_index,
                    const TsdfVoxel* voxel) {
    if (voxel->weight > 1e-4f) {
      Vector3f position = getCenterPostionFromBlockIndexAndVoxelIndex(
          block_size, block_index, voxel_index);
      if (position.z() - height < voxel_size && position.z() >= height) {
        points.push_back(position);
        distances.push_back(voxel->distance);
      }
    }
  };

  // Call above lambda on every voxel in the layer.
  callFunctionOnAllVoxels<TsdfVoxel>(layer, lambda);

  // Add the pointcloud to the ply writer.
  writer.setPoints(&points);
  writer.setIntensities(&distances);

  // Write out the ply.
  return writer.write();
}

float EsdfIntegratorTest::compareEsdfToGt(const EsdfLayer& esdf_layer,
                                          const TsdfLayer& gt_layer,
                                          float error_threshold) {
  // Compare the layers
  int total_num_voxels_observed = 0;
  int num_voxels_over_threshold = 0;
  for (const Index3D& block_index : esdf_layer.getAllBlockIndices()) {
    const auto block_esdf = esdf_layer.getBlockAtIndex(block_index);
    const auto block_gt = gt_layer.getBlockAtIndex(block_index);
    if (!block_esdf || !block_gt) {
      continue;
    }
    for (int x = 0; x < VoxelBlock<TsdfVoxel>::kVoxelsPerSide; x++) {
      for (int y = 0; y < VoxelBlock<TsdfVoxel>::kVoxelsPerSide; y++) {
        for (int z = 0; z < VoxelBlock<TsdfVoxel>::kVoxelsPerSide; z++) {
          float distance =
              esdf_layer.voxel_size() *
              std::sqrt(block_esdf->voxels[x][y][z].squared_distance_vox);
          if (block_esdf->voxels[x][y][z].is_inside) {
            distance = -distance;
          }
          float diff = distance - block_gt->voxels[x][y][z].distance;

          if (block_esdf->voxels[x][y][z].observed) {
            ++total_num_voxels_observed;
            if (std::abs(diff) > error_threshold) {
              ++num_voxels_over_threshold;
            }
          }
        }
      }
    }
  }
  LOG(INFO) << "Num voxels observed: " << total_num_voxels_observed
            << " over threshold: " << num_voxels_over_threshold;

  return static_cast<float>(num_voxels_over_threshold) /
         total_num_voxels_observed;
}

float EsdfIntegratorTest::compareEsdfToEsdf(const EsdfLayer& esdf_layer,
                                            const EsdfLayer& gt_layer,
                                            float error_threshold) {
  // Compare the layers
  int total_num_voxels_observed = 0;
  int num_voxels_over_threshold = 0;
  for (const Index3D& block_index : esdf_layer.getAllBlockIndices()) {
    const auto block_esdf = esdf_layer.getBlockAtIndex(block_index);
    CHECK_NOTNULL(block_esdf.get());
    const auto block_gt = gt_layer.getBlockAtIndex(block_index);
    if (!block_esdf || !block_gt) {
      continue;
    }
    for (int x = 0; x < VoxelBlock<TsdfVoxel>::kVoxelsPerSide; x++) {
      for (int y = 0; y < VoxelBlock<TsdfVoxel>::kVoxelsPerSide; y++) {
        for (int z = 0; z < VoxelBlock<TsdfVoxel>::kVoxelsPerSide; z++) {
          float distance =
              esdf_layer.voxel_size() *
              std::sqrt(block_esdf->voxels[x][y][z].squared_distance_vox);
          if (block_esdf->voxels[x][y][z].is_inside) {
            distance = -distance;
          }
          float distance_gt =
              esdf_layer.voxel_size() *
              std::sqrt(block_gt->voxels[x][y][z].squared_distance_vox);
          if (block_gt->voxels[x][y][z].is_inside) {
            distance_gt = -distance_gt;
          }
          float diff = distance - distance_gt;

          if (block_esdf->voxels[x][y][z].observed) {
            ++total_num_voxels_observed;
            if (std::abs(diff) > error_threshold) {
              ++num_voxels_over_threshold;
            }
          }
        }
      }
    }
  }
  LOG(INFO) << "Num ESDF voxels observed: " << total_num_voxels_observed
            << " over threshold: " << num_voxels_over_threshold;

  return static_cast<float>(num_voxels_over_threshold) /
         total_num_voxels_observed;
}

bool EsdfIntegratorTest::validateEsdf(const EsdfLayer& esdf_layer,
                                      float max_squared_distance_vox) {
  constexpr float kTolerance = 1e-4;
  constexpr int kVoxelsPerSide = VoxelBlock<TsdfVoxel>::kVoxelsPerSide;
  for (const Index3D& block_index : esdf_layer.getAllBlockIndices()) {
    const auto block = esdf_layer.getBlockAtIndex(block_index);
    if (!block) {
      continue;
    }
    for (int x = 0; x < kVoxelsPerSide; x++) {
      for (int y = 0; y < kVoxelsPerSide; y++) {
        for (int z = 0; z < kVoxelsPerSide; z++) {
          const EsdfVoxel& voxel = block->voxels[x][y][z];

          if (!voxel.observed) {
            continue;
          }
          // If the voxel is a site, its parent must be 0, 0, 0 and vice versa.
          if (voxel.parent_direction == Index3D::Zero()) {
            if (!voxel.is_site &&
                voxel.squared_distance_vox < max_squared_distance_vox &&
                voxel.squared_distance_vox >= kTolerance) {
              LOG(ERROR) << "Wrong distance for zero parent voxel! "
                         << voxel.parent_direction.transpose() << " "
                         << voxel.squared_distance_vox << "/"
                         << max_squared_distance_vox;
              return false;
            }
          } else {
            // First check tht the distance matches.
            if (voxel.squared_distance_vox < max_squared_distance_vox &&
                voxel.squared_distance_vox -
                        voxel.parent_direction.squaredNorm() >=
                    kTolerance) {
              LOG(ERROR) << "Distance doesn't match parent direction.";
              return false;
            }

            // If a voxel has a parent direction, it should definitely point to
            // a site voxel.

            // Get the parent.
            Index3D parent_index = Index3D(x, y, z) + voxel.parent_direction;

            // Check if the voxel is within the same block.
            if (parent_index.x() < 0 || parent_index.x() >= kVoxelsPerSide ||
                parent_index.y() < 0 || parent_index.y() >= kVoxelsPerSide ||
                parent_index.z() < 0 || parent_index.z() >= kVoxelsPerSide) {
              // Then we need to get the block index.
              Index3D neighbor_block_index = block_index;
              Index3D neighbor_voxel_index = parent_index;

              // Find the parent index.
              while (parent_index.x() >= kVoxelsPerSide) {
                parent_index.x() -= kVoxelsPerSide;
                neighbor_block_index.x()++;
              }
              while (parent_index.y() >= kVoxelsPerSide) {
                parent_index.y() -= kVoxelsPerSide;
                neighbor_block_index.y()++;
              }
              while (parent_index.z() >= kVoxelsPerSide) {
                parent_index.z() -= kVoxelsPerSide;
                neighbor_block_index.z()++;
              }

              while (parent_index.x() < 0) {
                parent_index.x() += kVoxelsPerSide;
                neighbor_block_index.x()--;
              }
              while (parent_index.y() < 0) {
                parent_index.y() += kVoxelsPerSide;
                neighbor_block_index.y()--;
              }
              while (parent_index.z() < 0) {
                parent_index.z() += kVoxelsPerSide;
                neighbor_block_index.z()--;
              }

              EsdfBlock::ConstPtr neighbor_block =
                  esdf_layer.getBlockAtIndex(neighbor_block_index);
              if (!neighbor_block) {
                LOG(ERROR) << "Neighbor block does not exist!";
                return false;
              }
              const EsdfVoxel* neighbor_voxel =
                  &neighbor_block->voxels[parent_index.x()][parent_index.y()]
                                         [parent_index.z()];
              if (!neighbor_voxel->is_site) {
                LOG(ERROR) << "Parent in different block but is not a site!";
                LOG(ERROR) << "Block index: "
                           << neighbor_block_index.transpose()
                           << " voxel index: " << parent_index.transpose();
                return false;
              }
            } else {
              // Ok check if the parent index is a site.
              if (!block
                       ->voxels[parent_index.x()][parent_index.y()]
                               [parent_index.z()]
                       .is_site) {
                LOG(ERROR) << "Parent within the same block but is not a site!";
                return false;
              }
            }
          }
          if (voxel.is_site) {
            if (voxel.parent_direction != Index3D::Zero()) {
              LOG(ERROR) << "Site voxel has a parent!";
              return false;
            }
            if (voxel.squared_distance_vox >= kTolerance) {
              LOG(ERROR) << "Site voxel has non-zero distance! "
                         << voxel.squared_distance_vox;
              return false;
            }
          }
        }
      }
    }
  }
  return true;
}

TEST_P(EsdfIntegratorTest, SingleEsdfTestCPU) {
  // Create a scene that's just a plane.
  addParameterizedObstacleToScene(GetParam());

  // Generate a TSDF
  scene_.generateSdfFromScene(4 * voxel_size_, tsdf_layer_.get());
  // Get the full ground truth ESDF
  scene_.generateSdfFromScene(max_distance_, gt_sdf_layer_.get());

  // Actually run the ESDF generation.
  std::vector<Index3D> block_indices = tsdf_layer_->getAllBlockIndices();
  esdf_integrator_.integrateBlocksOnCPU(*tsdf_layer_, block_indices,
                                        esdf_layer_.get());

  // Compare the results to GT.
  EXPECT_LE(compareEsdfToGt(*esdf_layer_, *gt_sdf_layer_, voxel_size_),
            very_small_cutoff_);

  // Check if all parents point the correct directions, etc.
  EXPECT_TRUE(validateEsdf(
      *esdf_layer_, esdf_integrator_.max_squared_distance_vox(voxel_size_)));

  float slice_height = 1.0f;
  std::string obstacle_string = std::to_string(static_cast<int>(GetParam()));
  outputFlatSliceEsdfAsPly(
      *esdf_layer_, "test_esdf_cpu_" + obstacle_string + "_esdf_slice.ply",
      1.0f);
  outputFlatSliceTsdfAsPly(
      *tsdf_layer_, "test_esdf_cpu_" + obstacle_string + "_tsdf_slice.ply",
      1.0f);

  if (output_pointclouds_) {
    io::outputVoxelLayerToPly(*esdf_layer_,
                              "test_esdf_cpu_" + obstacle_string + "_esdf.ply");
    io::outputVoxelLayerToPly(*gt_sdf_layer_,
                              "test_esdf_cpu_" + obstacle_string + "_gt.ply");
    io::outputVoxelLayerToPly(*tsdf_layer_,
                              "test_esdf_cpu_" + obstacle_string + "_tsdf.ply");
  }
  std::cout << timing::Timing::Print();
}

TEST_P(EsdfIntegratorTest, SingleEsdfTestGPU) {
  // Create a scene that's just an object.
  addParameterizedObstacleToScene(GetParam());

  // Generate a TSDF
  scene_.generateSdfFromScene(4 * voxel_size_, tsdf_layer_.get());
  // Get the full ground truth ESDF
  scene_.generateSdfFromScene(max_distance_, gt_sdf_layer_.get());

  // Actually run the ESDF generation.
  std::vector<Index3D> block_indices = tsdf_layer_->getAllBlockIndices();
  esdf_integrator_.integrateBlocksOnGPU(*tsdf_layer_, block_indices,
                                        esdf_layer_.get());

  // Compare the results to GT.
  EXPECT_LE(compareEsdfToGt(*esdf_layer_, *gt_sdf_layer_, voxel_size_),
            very_small_cutoff_);

  // Check if all parents point the correct directions, etc.
  EXPECT_TRUE(validateEsdf(
      *esdf_layer_, esdf_integrator_.max_squared_distance_vox(voxel_size_)));

  if (output_pointclouds_) {
    std::string obstacle_string = std::to_string(static_cast<int>(GetParam()));
    io::outputVoxelLayerToPly(*esdf_layer_,
                              "test_esdf_gpu_" + obstacle_string + "_esdf.ply");
    io::outputVoxelLayerToPly(*gt_sdf_layer_,
                              "test_esdf_gpu_" + obstacle_string + "_gt.ply");
    io::outputVoxelLayerToPly(*tsdf_layer_,
                              "test_esdf_gpu_" + obstacle_string + "_tsdf.ply");
  }
  std::cout << timing::Timing::Print();
}

TEST_P(EsdfIntegratorTest, ComplexSceneWithTsdf) {
  constexpr float kTrajectoryRadius = 4.0f;
  constexpr float kTrajectoryHeight = 2.0f;
  constexpr int kNumTrajectoryPoints = 80;

  // Maximum distance to consider for scene generation.
  constexpr float kMaxDist = 15.0;

  Obstacle obstacle = GetParam();
  // Skip the non-box cases:
  if (obstacle != Obstacle::kBoxWithSphere &&
      obstacle != Obstacle::kBoxWithCube && obstacle != Obstacle::kBox) {
    return;
  }
  addParameterizedObstacleToScene(obstacle);

  // Get the ground truth SDF for it.
  scene_.generateSdfFromScene(max_distance_, gt_sdf_layer_.get());

  // Set up the integrator.
  tsdf_integrator_.max_integration_distance_m(kMaxDist);

  // Simulate a trajectory of the requisite amount of points, on the circle
  // around the sphere.
  const float radians_increment = 2 * M_PI / (kNumTrajectoryPoints);

  // Create a depth frame. We share this memory buffer for the entire
  // trajectory.
  DepthImage depth_image(camera_->height(), camera_->width(),
                         MemoryType::kUnified);

  for (size_t i = 0; i < kNumTrajectoryPoints; i++) {
    const float theta = radians_increment * i;
    // Convert polar to cartesian coordinates.
    Vector3f cartesian_coordinates(kTrajectoryRadius * std::cos(theta),
                                   kTrajectoryRadius * std::sin(theta),
                                   kTrajectoryHeight);
    // The camera has its z axis pointing towards the origin.
    Eigen::Quaternionf rotation_base(0.5, 0.5, 0.5, 0.5);
    Eigen::Quaternionf rotation_theta(
        Eigen::AngleAxisf(M_PI + theta, Vector3f::UnitZ()));

    // Construct a transform from camera to scene with this.
    Transform T_S_C = Transform::Identity();
    T_S_C.prerotate(rotation_theta * rotation_base);
    T_S_C.pretranslate(cartesian_coordinates);

    // Generate a depth image of the scene.
    scene_.generateDepthImageFromScene(*camera_, T_S_C, kMaxDist, &depth_image);

    // Integrate this depth image.
    tsdf_integrator_.integrateFrame(depth_image, T_S_C, *camera_,
                                    tsdf_layer_.get());
  }

  // Actually run the ESDF generation.
  esdf_integrator_.integrateLayer(*tsdf_layer_, esdf_layer_.get());

  // Compare the results to GT.
  // Allow within 1 voxel sizes.
  EXPECT_LE(compareEsdfToGt(*esdf_layer_, *gt_sdf_layer_, 4 * voxel_size_),
            0.20);

  EXPECT_TRUE(validateEsdf(
      *esdf_layer_, esdf_integrator_.max_squared_distance_vox(voxel_size_)));

  if (output_pointclouds_) {
    //  Output the layer for inspection.
    outputFlatSliceEsdfAsPly(*esdf_layer_, "test_esdf_slice_complex.ply", 1.0f);
    outputFlatSliceTsdfAsPly(*gt_sdf_layer_, "test_esdf_slice_complex_gt.ply",
                             1.0f);
    io::outputVoxelLayerToPly(*gt_sdf_layer_,
                              "test_esdf_slice_complex_gt_full.ply");
  }

  std::cout << timing::Timing::Print();
}

TEST_P(EsdfIntegratorTest, IncrementalTsdfAndEsdfWithObjectRemovalGPU) {
  constexpr float kTrajectoryRadius = 4.0f;
  constexpr float kTrajectoryHeight = 2.0f;
  constexpr int kNumTrajectoryPoints = 1;

  // Maximum distance to consider for scene generation.
  constexpr float kMaxDist = 15.0;

  // Create a batch layer to batch to.
  EsdfLayer esdf_layer_batch(voxel_size_, MemoryType::kUnified);

  Obstacle obstacle = GetParam();
  // Skip the non-box cases:
  if (obstacle != Obstacle::kBoxWithSphere &&
      obstacle != Obstacle::kBoxWithCube && obstacle != Obstacle::kBox) {
    return;
  }
  addParameterizedObstacleToScene(obstacle);

  // Set up the integrator.
  tsdf_integrator_.max_integration_distance_m(kMaxDist);

  // Simulate a trajectory of the requisite amount of points, on the circle
  // around the sphere.
  const float radians_increment = 2 * M_PI / (kNumTrajectoryPoints);

  // Create a depth frame. We share this memory buffer for the entire
  // trajectory.
  DepthImage depth_image(camera_->height(), camera_->width(),
                         MemoryType::kUnified);

  for (size_t i = 0; i < kNumTrajectoryPoints * 2; i++) {
    if (i == kNumTrajectoryPoints) {
      // Clear the scene.
      scene_.clear();
      addParameterizedObstacleToScene(Obstacle::kBox);
    }
    const float theta = radians_increment * i;
    // Convert polar to cartesian coordinates.
    Vector3f cartesian_coordinates(kTrajectoryRadius * std::cos(theta),
                                   kTrajectoryRadius * std::sin(theta),
                                   kTrajectoryHeight);
    // The camera has its z axis pointing towards the origin.
    Eigen::Quaternionf rotation_base(0.5, 0.5, 0.5, 0.5);
    Eigen::Quaternionf rotation_theta(
        Eigen::AngleAxisf(M_PI + theta, Vector3f::UnitZ()));

    // Construct a transform from camera to scene with this.
    Transform T_S_C = Transform::Identity();
    T_S_C.prerotate(rotation_theta * rotation_base);
    T_S_C.pretranslate(cartesian_coordinates);

    // Generate a depth image of the scene.
    scene_.generateDepthImageFromScene(*camera_, T_S_C, kMaxDist, &depth_image);

    // Integrate this depth image.
    std::vector<Index3D> updated_blocks;
    tsdf_integrator_.integrateFrame(depth_image, T_S_C, *camera_,
                                    tsdf_layer_.get(), &updated_blocks);

    // Run incremental ESDF generation.
    esdf_integrator_.integrateBlocksOnGPU(*tsdf_layer_, updated_blocks,
                                          esdf_layer_.get());
  }

  // Run batch ESDF generation to compare to.
  esdf_integrator_.integrateLayer(*tsdf_layer_, &esdf_layer_batch);

  // Compare results to each other. Should really be basically identical.
  EXPECT_LE(compareEsdfToEsdf(*esdf_layer_, esdf_layer_batch, voxel_size_),
            small_cutoff_);

  EXPECT_TRUE(validateEsdf(
      *esdf_layer_, esdf_integrator_.max_squared_distance_vox(voxel_size_)));

  if (output_pointclouds_) {
    float slice_height = 3.75f;
    std::string obstacle_string = std::to_string(static_cast<int>(obstacle));
    outputFlatSliceEsdfAsPly(
        *esdf_layer_, "test_incremental_" + obstacle_string + "_esdf.ply",
        slice_height);
    outputFlatSliceEsdfAsPly(
        esdf_layer_batch,
        "test_incremental_" + obstacle_string + "_esdf_batch.ply",
        slice_height);
    io::outputVoxelLayerToPly(
        *esdf_layer_, "test_incremental_" + obstacle_string + "_esdf_full.ply");
  }
  std::cout << timing::Timing::Print();
}

TEST_P(EsdfIntegratorTest, IncrementalEsdf2DWithObjectRemoval) {
  // Create a batch layer to batch to.
  EsdfLayer esdf_layer_batch(voxel_size_, MemoryType::kUnified);

  Obstacle obstacle = GetParam();
  // Skip the non-box cases:
  if (obstacle != Obstacle::kBoxWithSphere &&
      obstacle != Obstacle::kBoxWithCube && obstacle != Obstacle::kBox) {
    return;
  }
  addParameterizedObstacleToScene(obstacle);

  AxisAlignedBoundingBox aabb(Vector3f(-5.0f, -5.0f, 1.0f - voxel_size_ / 2.0f),
                              Vector3f(5.0f, 5.0f, 1.0f + voxel_size_ / 2.0f));
  scene_.aabb() = aabb;

  // Get the ground truth SDF for it.
  scene_.generateSdfFromScene(max_distance_, tsdf_layer_.get());

  for (size_t i = 0; i < 2; i++) {
    if (i == 1) {
      tsdf_layer_->clear();
      // Clear the scene.
      scene_.clear();
      addParameterizedObstacleToScene(Obstacle::kBox);
      scene_.aabb() = aabb;
      scene_.generateSdfFromScene(max_distance_, tsdf_layer_.get());
    }

    // Get all blocks from the tsdf layer.
    std::vector<Index3D> updated_blocks;
    updated_blocks = tsdf_layer_->getAllBlockIndices();

    // Run incremental ESDF generation.
    esdf_integrator_.integrateBlocksOnGPU(*tsdf_layer_, updated_blocks,
                                          esdf_layer_.get());
  }

  // Run batch ESDF generation to compare to.
  esdf_integrator_.integrateLayer(*tsdf_layer_, &esdf_layer_batch);

  // Compare results to each other. Should really be basically identical.
  EXPECT_LE(compareEsdfToEsdf(*esdf_layer_, esdf_layer_batch, voxel_size_),
            small_cutoff_);

  EXPECT_TRUE(validateEsdf(
      *esdf_layer_, esdf_integrator_.max_squared_distance_vox(voxel_size_)));
  if (output_pointclouds_) {
    float slice_height = 1.0f;
    std::string obstacle_string = std::to_string(static_cast<int>(obstacle));
    outputFlatSliceEsdfAsPly(
        *esdf_layer_, "test_2d_" + obstacle_string + "_esdf.ply", slice_height);
    outputFlatSliceEsdfAsPly(esdf_layer_batch,
                             "test_2d_" + obstacle_string + "_esdf_batch.ply",
                             slice_height);
    outputFlatSliceTsdfAsPly(*tsdf_layer_,
                             "test_2d_" + obstacle_string + "_tsdf_batch.ply",
                             slice_height);
  }
  std::cout << timing::Timing::Print();
}

TEST_P(EsdfIntegratorTest, IncrementalEsdfSliceWithObjectRemovalGPU) {
  // Create a batch layer to batch to.
  EsdfLayer esdf_layer_batch(voxel_size_, MemoryType::kUnified);

  Obstacle obstacle = GetParam();
  // Skip the non-box cases:
  if (obstacle != Obstacle::kBoxWithSphere &&
      obstacle != Obstacle::kBoxWithCube && obstacle != Obstacle::kBox) {
    return;
  }

  float min_z = 1.0f;
  float max_z = 2.0f;
  float output_z = 1.5f;
  AxisAlignedBoundingBox aabb(
      Vector3f(-5.0f, -5.0f, output_z - voxel_size_ / 2.0f),
      Vector3f(5.0f, 5.0f, output_z + voxel_size_ / 2.0f));
  float tsdf_truncation_distance = 4 * voxel_size_;

  addParameterizedObstacleToScene(obstacle);

  scene_.aabb() = aabb;

  // Get the ground truth SDF for it.
  scene_.generateSdfFromScene(tsdf_truncation_distance, tsdf_layer_.get());

  for (size_t i = 0; i < 2; i++) {
    if (i == 1) {
      tsdf_layer_->clear();
      // Clear the scene.
      scene_.clear();
      addParameterizedObstacleToScene(Obstacle::kBox);
      scene_.aabb() = aabb;
      scene_.generateSdfFromScene(tsdf_truncation_distance, tsdf_layer_.get());
    }

    // Get all blocks from the tsdf layer.
    std::vector<Index3D> updated_blocks;
    updated_blocks = tsdf_layer_->getAllBlockIndices();

    // Run incremental ESDF generation.
    esdf_integrator_.integrateSliceOnGPU(*tsdf_layer_, updated_blocks, min_z,
                                         max_z, output_z, esdf_layer_.get());
  }

  // Run batch ESDF generation to compare to.
  esdf_integrator_.integrateLayer(*tsdf_layer_, &esdf_layer_batch);

  // Compare results to each other. Should really be basically identical.
  EXPECT_LE(compareEsdfToEsdf(*esdf_layer_, esdf_layer_batch, voxel_size_),
            small_cutoff_);

  EXPECT_TRUE(validateEsdf(
      *esdf_layer_, esdf_integrator_.max_squared_distance_vox(voxel_size_)));
  if (output_pointclouds_) {
    float slice_height = output_z;
    std::string obstacle_string = std::to_string(static_cast<int>(obstacle));
    io::outputVoxelLayerToPly(
        *esdf_layer_, "test_slice_" + obstacle_string + "_esdf_full.ply");
    outputFlatSliceEsdfAsPly(
        esdf_layer_batch, "test_slice_" + obstacle_string + "_esdf_batch.ply",
        slice_height);
    io::outputVoxelLayerToPly(
        *tsdf_layer_, "test_slice_" + obstacle_string + "_tsdf_batch.ply");
  }
  std::cout << timing::Timing::Print();
}

TEST_P(EsdfIntegratorTest, IncrementalEsdfWithObjectRemoval) {
  // Create a batch layer to batch to.
  EsdfLayer esdf_layer_batch(voxel_size_, MemoryType::kUnified);

  Obstacle obstacle = GetParam();
  // Skip the non-box cases:
  if (obstacle != Obstacle::kBoxWithSphere &&
      obstacle != Obstacle::kBoxWithCube && obstacle != Obstacle::kBox) {
    return;
  }
  addParameterizedObstacleToScene(obstacle);

  // Get the ground truth SDF for it.
  scene_.generateSdfFromScene(max_distance_, tsdf_layer_.get());

  for (size_t i = 0; i < 2; i++) {
    if (i == 1) {
      // Clear the scene.
      scene_.clear();
      addParameterizedObstacleToScene(Obstacle::kBox);

      scene_.generateSdfFromScene(max_distance_, tsdf_layer_.get());
    }

    // Get all blocks from the tsdf layer.
    std::vector<Index3D> updated_blocks;
    updated_blocks = tsdf_layer_->getAllBlockIndices();

    // Run incremental ESDF generation.
    esdf_integrator_.integrateBlocksOnGPU(*tsdf_layer_, updated_blocks,
                                          esdf_layer_.get());
  }

  // Run batch ESDF generation to compare to.
  esdf_integrator_.integrateLayer(*tsdf_layer_, &esdf_layer_batch);

  // Compare results to each other. Should really be basically identical.
  EXPECT_LE(compareEsdfToEsdf(*esdf_layer_, esdf_layer_batch, voxel_size_),
            small_cutoff_);

  EXPECT_TRUE(validateEsdf(
      *esdf_layer_, esdf_integrator_.max_squared_distance_vox(voxel_size_)));

  if (output_pointclouds_) {
    float slice_height = 3.5f;
    std::string obstacle_string = std::to_string(static_cast<int>(obstacle));
    outputFlatSliceEsdfAsPly(*esdf_layer_,
                             "test_object_" + obstacle_string + "_esdf.ply",
                             slice_height);
    outputFlatSliceEsdfAsPly(
        esdf_layer_batch, "test_object_" + obstacle_string + "_esdf_batch.ply",
        slice_height);
    io::outputVoxelLayerToPly(
        *esdf_layer_, "test_object_" + obstacle_string + "_esdf_full.ply");
  }
  std::cout << timing::Timing::Print();
}

INSTANTIATE_TEST_CASE_P(
    ParameterizedEsdfTests, EsdfIntegratorTest,
    ::testing::Values(Obstacle::kAxisAlignedPlane, Obstacle::kAngledPlane,
                      Obstacle::kSphereOrigin, Obstacle::kBox,
                      Obstacle::kBoxWithSphere, Obstacle::kBoxWithCube));

int main(int argc, char** argv) {
  warmupCuda();
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::InstallFailureSignalHandler();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
