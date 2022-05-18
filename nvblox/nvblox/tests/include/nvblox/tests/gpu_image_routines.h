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
#include "nvblox/core/color.h"
#include "nvblox/core/image.h"
#include "nvblox/core/types.h"

namespace nvblox {
namespace test_utils {

void setImageConstantOnGpu(const float value, DepthImage* image_ptr);
void setImageConstantOnGpu(const Color value, ColorImage* image_ptr);

void getDifferenceImageOnGPU(const ColorImage& image_1,
                             const ColorImage& image_2,
                             ColorImage* diff_image_ptr);
void getDifferenceImageOnGPU(const DepthImage& image_1,
                             const DepthImage& image_2,
                             DepthImage* diff_image_ptr);

}  // namespace test_utils
}  // namespace nvblox
