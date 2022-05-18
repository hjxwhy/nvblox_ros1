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
#include "nvblox/datasets/image_loader.h"
#include "nvblox/datasets/parse_3dmatch.h"
#include "nvblox/integrators/projective_tsdf_integrator.h"
#include "nvblox/io/mesh_io.h"
#include "nvblox/io/ply_writer.h"
#include "nvblox/mesh/mesh_block.h"
#include "nvblox/mesh/mesh_integrator.h"
#include "nvblox/primitives/scene.h"

#include "nvblox/utils/timing.h"

using namespace nvblox;

constexpr float kFloatEpsilon = 1e-4;

class MeshTest : public ::testing::Test {
 protected:
  void SetUp() override {
    timing::Timing::Reset();
    std::srand(0);

    sdf_layer_.reset(new TsdfLayer(voxel_size_, MemoryType::kUnified));
    mesh_layer_.reset(
        new MeshLayer(sdf_layer_->block_size(), MemoryType::kUnified));

    block_size_ = mesh_layer_->block_size();

    // Make the scene 6x6x3 meters big.
    scene_.aabb() = AxisAlignedBoundingBox(Vector3f(-3.0f, -3.0f, 0.0f),
                                           Vector3f(3.0f, 3.0f, 3.0f));
  }

  float block_size_;
  float voxel_size_ = 0.10;

  TsdfLayer::Ptr sdf_layer_;
  MeshLayer::Ptr mesh_layer_;

  // The actual integrator.
  MeshIntegrator mesh_integrator_;

  // A simulation scene.
  primitives::Scene scene_;
};

TEST_F(MeshTest, DirectionFromNeighborTest) {
  for (int i = 0; i < 8; i++) {
    Index3D dir = marching_cubes::directionFromNeighborIndex(i);
    EXPECT_GE(dir.minCoeff(), 0);
    EXPECT_LE(dir.maxCoeff(), 1);

    int index = marching_cubes::neighborIndexFromDirection(dir);
    EXPECT_EQ(i, index);
  }
  // Make sure 0 maps to 0.
  EXPECT_EQ(marching_cubes::neighborIndexFromDirection(Index3D::Zero()), 0);
}

TEST_F(MeshTest, BlankMap) {
  std::vector<Index3D> block_indices_mesh = mesh_layer_->getAllBlockIndices();
  std::vector<Index3D> block_indices_sdf = sdf_layer_->getAllBlockIndices();

  ASSERT_EQ(block_indices_mesh.size(), 0);
  ASSERT_EQ(block_indices_sdf.size(), 0);

  // Integrate a blank map.
  EXPECT_TRUE(mesh_integrator_.integrateMeshFromDistanceField(
      *sdf_layer_, mesh_layer_.get()));

  // Should still be 0.
  block_indices_mesh = mesh_layer_->getAllBlockIndices();
  ASSERT_EQ(block_indices_mesh.size(), 0);
}

TEST_F(MeshTest, PlaneMesh) {
  // Create a scene that's just a plane.
  // Plane at the origin pointing in the -x direction.
  scene_.addPrimitive(std::make_unique<primitives::Plane>(
      Vector3f(0.0, 0.0, 0.0), Vector3f(-1, 0, 0)));

  scene_.generateSdfFromScene(4 * voxel_size_, sdf_layer_.get());

  EXPECT_TRUE(mesh_integrator_.integrateMeshFromDistanceField(
      *sdf_layer_, mesh_layer_.get(), DeviceType::kCPU));

  std::vector<Index3D> block_indices_mesh = mesh_layer_->getAllBlockIndices();
  std::vector<Index3D> block_indices_sdf = sdf_layer_->getAllBlockIndices();

  EXPECT_GT(block_indices_sdf.size(), 0);
  EXPECT_LE(block_indices_mesh.size(), block_indices_sdf.size());
  EXPECT_GT(block_indices_mesh.size(), 0);

  // Make sure there's no empty mesh blocks.
  for (const Index3D& block_index : block_indices_mesh) {
    MeshBlock::ConstPtr mesh_block = mesh_layer_->getBlockAtIndex(block_index);

    EXPECT_GT(mesh_block->vertices.size(), 0);
    EXPECT_GT(mesh_block->normals.size(), 0);
    EXPECT_GT(mesh_block->triangles.size(), 0);

    // Make sure everything is the same size.
    EXPECT_EQ(mesh_block->vertices.size(), mesh_block->normals.size());
    EXPECT_EQ(mesh_block->vertices.size(), mesh_block->triangles.size());

    std::vector<Vector3f> vertices = mesh_block->getVertexVectorOnCPU();
    std::vector<Vector3f> normals = mesh_block->getNormalVectorOnCPU();

    // Make sure that the actual points are correct.
    for (size_t i = 0; i < vertices.size(); i++) {
      const Vector3f& vertex = vertices[i];
      const Vector3f& normal = normals[i];

      // Make sure the points on the plane are correct.
      EXPECT_NEAR(vertex.x(), 0.0, kFloatEpsilon);
      EXPECT_NEAR(normal.x(), -1.0, kFloatEpsilon);
      EXPECT_NEAR(normal.y(), 0.0, kFloatEpsilon);
      EXPECT_NEAR(normal.z(), 0.0, kFloatEpsilon);
    }
  }

  io::outputMeshLayerToPly(*mesh_layer_, "test_mesh_cpu.ply");

  std::cout << timing::Timing::Print();
}

TEST_F(MeshTest, ComplexScene) {
  scene_.addPrimitive(std::make_unique<primitives::Plane>(
      Vector3f(0.0, 0.0, 0.0), Vector3f(-1, 0, 0)));

  scene_.addPrimitive(std::make_unique<primitives::Plane>(
      Vector3f(2.1, 0.1, 0.1), Vector3f(0, -1, 0)));

  scene_.addPrimitive(
      std::make_unique<primitives::Sphere>(Vector3f(-2, -2, 0), 2.0));

  scene_.generateSdfFromScene(4 * voxel_size_, sdf_layer_.get());

  EXPECT_TRUE(mesh_integrator_.integrateMeshFromDistanceField(
      *sdf_layer_, mesh_layer_.get(), DeviceType::kCPU));

  std::vector<Index3D> block_indices_mesh = mesh_layer_->getAllBlockIndices();
  EXPECT_GT(block_indices_mesh.size(), 0);

  io::outputMeshLayerToPly(*mesh_layer_, "test_mesh_complex.ply");
  std::cout << timing::Timing::Print();
}

TEST_F(MeshTest, GPUPlaneTest) {
  // Create a scene that's just a plane.
  // Plane at the origin pointing in the -x direction.
  scene_.addPrimitive(std::make_unique<primitives::Plane>(
      Vector3f(0.00, 0.0, 0.0), Vector3f(-1, 0, 0)));

  scene_.generateSdfFromScene(4 * voxel_size_, sdf_layer_.get());

  // Create a second mesh layer for the CPU.
  BlockLayer<MeshBlock> cpu_mesh(block_size_, MemoryType::kUnified);
  EXPECT_TRUE(mesh_integrator_.integrateMeshFromDistanceField(
      *sdf_layer_, &cpu_mesh, DeviceType::kCPU));

  // Integrate from GPU.
  EXPECT_TRUE(mesh_integrator_.integrateMeshFromDistanceField(
      *sdf_layer_, mesh_layer_.get(), DeviceType::kGPU));

  std::vector<Index3D> block_indices_mesh = mesh_layer_->getAllBlockIndices();
  std::vector<Index3D> block_indices_sdf = sdf_layer_->getAllBlockIndices();
  std::vector<Index3D> block_indices_mesh_cpu = cpu_mesh.getAllBlockIndices();

  EXPECT_GT(block_indices_sdf.size(), 0);
  EXPECT_LE(block_indices_mesh.size(), block_indices_sdf.size());
  EXPECT_GT(block_indices_mesh.size(), 0);
  EXPECT_GE(block_indices_mesh.size(), block_indices_mesh_cpu.size());

  // With the GPU integration we might actually have empty mesh blocks.
  for (const Index3D& block_index : block_indices_mesh) {
    MeshBlock::ConstPtr mesh_block = mesh_layer_->getBlockAtIndex(block_index);
    MeshBlock::ConstPtr mesh_block_cpu = cpu_mesh.getBlockAtIndex(block_index);
    if (mesh_block_cpu == nullptr) {
      EXPECT_EQ(mesh_block->vertices.size(), 0);
      EXPECT_EQ(mesh_block->normals.size(), 0);
      EXPECT_EQ(mesh_block->triangles.size(), 0);
    }

    // Make sure everything is the same size.
    EXPECT_EQ(mesh_block->vertices.size(), mesh_block->normals.size());
    EXPECT_EQ(mesh_block->vertices.size(), mesh_block->triangles.size());

    // Make sure we have the same size as on the CPU as well.
    if (mesh_block_cpu != nullptr) {
      EXPECT_EQ(mesh_block->vertices.size(), mesh_block_cpu->vertices.size());
    }
    std::vector<Vector3f> vertices = mesh_block->getVertexVectorOnCPU();
    std::vector<Vector3f> normals = mesh_block->getNormalVectorOnCPU();

    // Make sure that the actual points are correct.
    for (size_t i = 0; i < vertices.size(); i++) {
      const Vector3f& vertex = vertices[i];
      const Vector3f& normal = normals[i];

      // Make sure the points on the plane are correct.
      EXPECT_NEAR(vertex.x(), 0.0, kFloatEpsilon);
      EXPECT_NEAR(normal.x(), -1.0, kFloatEpsilon);
      EXPECT_NEAR(normal.y(), 0.0, kFloatEpsilon);
      EXPECT_NEAR(normal.z(), 0.0, kFloatEpsilon);
    }
  }
  io::outputMeshLayerToPly(*mesh_layer_, "test_mesh_gpu.ply");

  std::cout << timing::Timing::Print();
}

TEST_F(MeshTest, IncrementalMesh) {
  constexpr float kSphereRadius = 2.0f;
  constexpr float kTrajectoryRadius = 4.0f;
  constexpr float kTrajectoryHeight = 2.0f;
  constexpr int kNumTrajectoryPoints = 80;

  // Maximum distance to consider for scene generation.
  constexpr float kMaxDist = 10.0;
  constexpr float kMinWeight = 2.0;

  mesh_integrator_.min_weight() = kMinWeight;

  // Create a camera.
  Camera camera(300, 300, 320, 240, 640, 480);

  // Scene is bounded to -5, -5, 0 to 5, 5, 5.
  scene_.aabb() = AxisAlignedBoundingBox(Vector3f(-5.0f, -5.0f, 0.0f),
                                         Vector3f(5.0f, 5.0f, 5.0f));
  // Create a scene with a ground plane, ceiling, and a sphere.
  scene_.addGroundLevel(0.0f);
  scene_.addCeiling(5.0f);
  scene_.addPrimitive(
      std::make_unique<primitives::Sphere>(Vector3f(0.0f, 0.0f, 2.0f), 2.0f));
  // Add bounding planes at 5 meters. Basically makes it sphere in a box.
  scene_.addPlaneBoundaries(-5.0f, 5.0f, -5.0f, 5.0f);

  // Create an integrator.
  ProjectiveTsdfIntegrator integrator;
  integrator.max_integration_distance_m(10.0);

  // Simulate a trajectory of the requisite amount of points, on the circle
  // around the sphere.
  const float radians_increment = 2 * M_PI / (kNumTrajectoryPoints);

  // Create a depth frame. We share this memory buffer for the entire
  // trajectory.
  DepthImage depth_frame(camera.height(), camera.width(), MemoryType::kUnified);

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
    scene_.generateDepthImageFromScene(camera, T_S_C, kMaxDist, &depth_frame);

    // Integrate this depth image.
    std::vector<Index3D> updated_blocks;

    integrator.integrateFrame(depth_frame, T_S_C, camera, sdf_layer_.get(),
                              &updated_blocks);

    // Integrate the mesh.
    mesh_integrator_.integrateBlocksGPU(*sdf_layer_, updated_blocks,
                                        mesh_layer_.get());
  }

  // Create a comparison mesh.
  BlockLayer<MeshBlock>::Ptr batch_mesh_layer(
      new BlockLayer<MeshBlock>(block_size_, MemoryType::kUnified));

  // Compute the batch mesh.
  mesh_integrator_.integrateMeshFromDistanceField(*sdf_layer_,
                                                  batch_mesh_layer.get());

  io::outputMeshLayerToPly(*mesh_layer_, "test_mesh_inc.ply");
  io::outputMeshLayerToPly(*batch_mesh_layer, "test_mesh_batch.ply");

  // For each block in the batch mesh, make sure we have roughly the same
  // number of points in the incremental mesh.
  std::vector<Index3D> all_blocks = batch_mesh_layer->getAllBlockIndices();

  for (const Index3D& block : all_blocks) {
    MeshBlock::ConstPtr batch_block = batch_mesh_layer->getBlockAtIndex(block);
    MeshBlock::ConstPtr inc_block = mesh_layer_->getBlockAtIndex(block);

    ASSERT_NE(inc_block.get(), nullptr);
    EXPECT_EQ(batch_block->vertices.size(), inc_block->vertices.size());
  }

  std::cout << timing::Timing::Print();
}

TEST_F(MeshTest, RepeatabilityTest) {
  const std::string base_path = "../tests/data/3dmatch";
  constexpr int seq_id = 1;
  DepthImage depth_image;
  ColorImage color_image;
  EXPECT_TRUE(datasets::load16BitDepthImage(
      datasets::threedmatch::getPathForDepthImage(base_path, seq_id, 0),
      &depth_image));
  EXPECT_TRUE(datasets::load8BitColorImage(
      datasets::threedmatch::getPathForColorImage(base_path, seq_id, 0),
      &color_image));
  EXPECT_EQ(depth_image.width(), color_image.width());
  EXPECT_EQ(depth_image.height(), color_image.height());

  // Parse 3x3 camera intrinsics matrix from 3D Match format: space-separated.
  Eigen::Matrix3f camera_intrinsic_matrix;
  EXPECT_TRUE(datasets::threedmatch::parseCameraFromFile(
      datasets::threedmatch::getPathForCameraIntrinsics(base_path),
      &camera_intrinsic_matrix));
  const auto camera = Camera::fromIntrinsicsMatrix(
      camera_intrinsic_matrix, depth_image.width(), depth_image.height());

  // Integrate depth
  sdf_layer_ = std::make_shared<TsdfLayer>(voxel_size_, MemoryType::kUnified);
  ProjectiveTsdfIntegrator tsdf_integrator;
  tsdf_integrator.integrateFrame(depth_image, Transform::Identity(), camera,
                                 sdf_layer_.get());

  // Generate the mesh (twice)
  MeshLayer mesh_layer_1(block_size_, MemoryType::kUnified);
  MeshLayer mesh_layer_2(block_size_, MemoryType::kUnified);
  EXPECT_TRUE(mesh_integrator_.integrateMeshFromDistanceField(*sdf_layer_,
                                                              &mesh_layer_1));
  EXPECT_TRUE(mesh_integrator_.integrateMeshFromDistanceField(*sdf_layer_,
                                                              &mesh_layer_2));

  // Check that the generated vertices are the same.
  auto block_indices_1 = mesh_layer_1.getAllBlockIndices();
  auto block_indices_2 = mesh_layer_2.getAllBlockIndices();
  EXPECT_EQ(block_indices_1.size(), block_indices_2.size());
  for (int block_idx = 0; block_idx < block_indices_1.size(); block_idx++) {
    const Index3D& idx_1 = block_indices_1[block_idx];
    const Index3D& idx_2 = block_indices_2[block_idx];
    EXPECT_TRUE((idx_1.array() == idx_2.array()).all());
    MeshBlock::ConstPtr block_1 = mesh_layer_1.getBlockAtIndex(idx_1);
    MeshBlock::ConstPtr block_2 = mesh_layer_2.getBlockAtIndex(idx_2);
    EXPECT_EQ(block_1->vertices.size(), block_2->vertices.size());

    auto threed_less = [](const Vector3f& p_1, const Vector3f& p_2) -> bool {
      if (p_1.x() != p_2.x()) {
        return p_1.x() < p_2.x();
      }
      if (p_1.y() != p_2.y()) {
        return p_1.y() < p_2.y();
      }
      return p_1.z() < p_2.z();
    };
    auto vertex_vector_1 = block_1->getVertexVectorOnCPU();
    auto vertex_vector_2 = block_2->getVertexVectorOnCPU();
    std::sort(vertex_vector_1.begin(), vertex_vector_1.end(), threed_less);
    std::sort(vertex_vector_2.begin(), vertex_vector_2.end(), threed_less);
    for (int i = 0; i < vertex_vector_1.size(); i++) {
      EXPECT_TRUE(
          (vertex_vector_1[i].array() == vertex_vector_2[i].array()).all());
    }
  }
}

int main(int argc, char** argv) {
  warmupCuda();
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::InstallFailureSignalHandler();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
