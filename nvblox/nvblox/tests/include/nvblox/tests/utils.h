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
#pragma once

#include "nvblox/core/blox.h"
#include "nvblox/core/color.h"
#include "nvblox/core/layer.h"
#include "nvblox/core/types.h"
#include "nvblox/core/voxels.h"

namespace nvblox {
namespace test_utils {

float randomFloatInRange(float f_min, float f_max);

float randomIntInRange(int i_min, int i_max);

float randomSign();

Index3D getRandomIndex3dInRange(const int min, const int max);

Vector3f getRandomVector3fInRange(const float min, const float max);

Vector3f getRandomVector3fInRange(const Vector3f& min, const Vector3f& max);

Vector3f getRandomUnitVector3f();

Color randomColor();

}  // namespace test_utils
}  // namespace nvblox
