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
#include "nvblox/core/common_names.h"
#include "nvblox/core/image.h"
#include "nvblox/core/layer.h"
#include "nvblox/core/types.h"
#include "nvblox/integrators/projective_tsdf_integrator.h"

namespace nvblox {

class ProjectiveTsdfIntegratorCPU : public ProjectiveTsdfIntegrator {
 public:
  ProjectiveTsdfIntegratorCPU() : ProjectiveTsdfIntegrator() {}
  virtual ~ProjectiveTsdfIntegratorCPU() {}

 protected:
  // We override the GPU version of this function and run it on the CPU.
  void updateBlocks(const std::vector<Index3D>& block_indices,
                    const DepthImage& depth_frame, const Transform& T_L_C,
                    const Camera& camera, const float truncation_distance_m,
                    TsdfLayer* layer) override;
};

}  // namespace nvblox
