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

#include "nvblox/core/common_names.h"
#include "nvblox/core/layer.h"
#include "nvblox/core/types.h"
#include "nvblox/core/voxels.h"

#include "nvblox/tests/blox.h"
#include "nvblox/tests/blox_utils.h"
#include "nvblox/tests/utils.h"

using namespace nvblox;

Index3D getRandomIndex3DInRange(const int min, const int max) {
  return Index3D(test_utils::randomIntInRange(min, max),
                 test_utils::randomIntInRange(min, max),
                 test_utils::randomIntInRange(min, max));
}

TEST(LayerTest, InsertionAndRetrieval) {
  // Make sure this is deterministic.
  std::srand(0);

  // Empty layer
  constexpr float KTestBlockSize = 0.1;
  BlockLayer<IndexBlock> layer(KTestBlockSize, MemoryType::kUnified);

  // Get some distict block locations
  constexpr int kNumTestItems = 100;
  constexpr int kMinimumIndexValue = -100;
  constexpr int kMaximumIndexValue = 100;

  Index3DSet unique_block_indices;
  for (int i = 0; i < kNumTestItems; i++) {
    while (!unique_block_indices
                .insert(getRandomIndex3DInRange(kMinimumIndexValue,
                                                kMaximumIndexValue))
                .second) {
      continue;
    }
  }

  // Allocate blocks at these indices (and store index in block for checking
  // later)
  for (auto it = unique_block_indices.begin(); it != unique_block_indices.end();
       it++) {
    EXPECT_FALSE(layer.getBlockAtIndex(*it));
    IndexBlock::Ptr block_ptr = layer.allocateBlockAtIndex(*it);
    block_ptr->data = *it;
  }

  // Allocate blocks at these positions
  for (auto it = unique_block_indices.begin(); it != unique_block_indices.end();
       it++) {
    IndexBlock::ConstPtr block_ptr = layer.getBlockAtIndex(*it);
    // Check it's there
    EXPECT_NE(block_ptr, nullptr);
    // Check it has the data we put in.
    EXPECT_EQ(block_ptr->data, *it);
  }
}

TEST(LayerTest, EmptyLayer) {
  constexpr float KTestBlockSize = 0.1;
  BlockLayer<IndexBlock> layer(KTestBlockSize, MemoryType::kUnified);

  IndexBlock::Ptr index_block_ptr = layer.getBlockAtIndex(Index3D(4, 50, -10));

  EXPECT_FALSE(index_block_ptr);
  EXPECT_EQ(index_block_ptr.get(), nullptr);
}

TEST(LayerTest, MinCornerBasedIndexing) {
  // Check that indexing is performed with block origin at minimum corner and
  // spanning block_size. E.g. from [0,0,0], [1,1,1], exclusive on the top side.

  constexpr float KTestBlockSize = 0.1;
  BlockLayer<DummyBlock> layer(KTestBlockSize, MemoryType::kUnified);

  const Vector3f kPostion3DEpsilson(0.001f, 0.001f, 0.001f);
  Vector3f position_low(0, 0, 0);
  Vector3f position_high =
      KTestBlockSize * Vector3f::Ones() - kPostion3DEpsilson;

  // Put something in on the low side of the block's range
  DummyBlock::Ptr block_low_ptr = layer.allocateBlockAtPosition(position_low);
  block_low_ptr->data = true;

  // Check we get the same block back on the high side.
  DummyBlock::ConstPtr block_high_ptr =
      layer.allocateBlockAtPosition(position_high);
  EXPECT_NE(block_high_ptr, nullptr);
  EXPECT_EQ(block_low_ptr, block_high_ptr);
  EXPECT_TRUE(block_high_ptr->data);
}

TEST(VoxelLayerTest, CopyVoxelsToHost) {
  constexpr float voxel_size_m = 1.0;
  TsdfLayer tsdf_layer(voxel_size_m, MemoryType::kDevice);
  auto block_ptr = tsdf_layer.allocateBlockAtIndex(Index3D(0, 0, 0));

  test_utils::setTsdfBlockVoxelsInSequence(block_ptr);

  // Generating the voxel center positions
  std::vector<Vector3f> positions_L;
  for (int x = 0; x < TsdfBlock::kVoxelsPerSide; x++) {
    for (int y = 0; y < TsdfBlock::kVoxelsPerSide; y++) {
      for (int z = 0; z < TsdfBlock::kVoxelsPerSide; z++) {
        positions_L.push_back(Index3D(x, y, z).cast<float>() +
                              0.5 * Vector3f::Ones());
      }
    }
  }

  std::vector<TsdfVoxel> voxels;
  std::vector<bool> flags;
  tsdf_layer.getVoxels(positions_L, &voxels, &flags);

  for (int i = 0; i < voxels.size(); i++) {
    EXPECT_TRUE(flags[i]);

    EXPECT_EQ(voxels[i].distance, static_cast<float>(i));
    EXPECT_EQ(voxels[i].weight, static_cast<float>(i));
  }

  // Now try some edge cases

  // Just inside the block from {0.0f, 0.0f, 0.0f}
  constexpr float kEps = 1e-5;
  const Vector3f kVecEps = kEps * Vector3f::Ones();
  tsdf_layer.getVoxels({kVecEps}, &voxels, &flags);
  EXPECT_EQ(flags.size(), 1);
  EXPECT_EQ(voxels.size(), 1);
  EXPECT_TRUE(flags[0]);
  EXPECT_EQ(voxels[0].distance, 0.0f);
  EXPECT_EQ(voxels[0].weight, 0.0f);

  // Just inside the block from it's far boundary {8.0f, 8.0f, 8.0f}
  tsdf_layer.getVoxels({Vector3f(8.0f, 8.0f, 8.0f) - kVecEps}, &voxels, &flags);
  EXPECT_TRUE(flags[0]);
  EXPECT_EQ(voxels[0].distance, 511.0f);
  EXPECT_EQ(voxels[0].weight, 511.0f);

  // Just outside the block from {0.0f, 0.0f, 0.0f}
  tsdf_layer.getVoxels({-kVecEps}, &voxels, &flags);
  EXPECT_FALSE(flags[0]);

  // Just outside the block from it's far boundary {8.0f, 8.0f, 8.0f}
  tsdf_layer.getVoxels({Vector3f(8.0f, 8.0f, 8.0f) + kVecEps}, &voxels, &flags);
  EXPECT_FALSE(flags[0]);
}

TEST(LayerTest, MoveOperations) {
  constexpr float voxel_size_m = 1.0;

  // BlockLayer (MeshLayer being the representative)
  MeshLayer mesh_layer_1(voxel_size_m, MemoryType::kDevice);
  mesh_layer_1.allocateBlockAtIndex(Index3D(0, 0, 0));

  EXPECT_TRUE(mesh_layer_1.isBlockAllocated(Index3D(0, 0, 0)));

  MeshLayer mesh_layer_2 = std::move(mesh_layer_1);

  EXPECT_FALSE(mesh_layer_1.isBlockAllocated(Index3D(0, 0, 0)));
  EXPECT_TRUE(mesh_layer_2.isBlockAllocated(Index3D(0, 0, 0)));

  MeshLayer mesh_layer_3(std::move(mesh_layer_2));

  EXPECT_FALSE(mesh_layer_2.isBlockAllocated(Index3D(0, 0, 0)));
  EXPECT_TRUE(mesh_layer_3.isBlockAllocated(Index3D(0, 0, 0)));

  // VoxelBlockLayer (TsdfLayer being the representative)
  TsdfLayer tsdf_layer_1(voxel_size_m, MemoryType::kDevice);
  tsdf_layer_1.allocateBlockAtIndex(Index3D(0, 0, 0));

  TsdfLayer tsdf_layer_2 = std::move(tsdf_layer_1);

  EXPECT_FALSE(tsdf_layer_1.isBlockAllocated(Index3D(0, 0, 0)));
  EXPECT_TRUE(tsdf_layer_2.isBlockAllocated(Index3D(0, 0, 0)));

  TsdfLayer tsdf_layer_3(std::move(tsdf_layer_2));

  EXPECT_FALSE(tsdf_layer_2.isBlockAllocated(Index3D(0, 0, 0)));
  EXPECT_TRUE(tsdf_layer_3.isBlockAllocated(Index3D(0, 0, 0)));
}

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}