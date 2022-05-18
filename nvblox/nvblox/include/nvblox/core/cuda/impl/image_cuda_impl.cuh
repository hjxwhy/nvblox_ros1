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

#include "nvblox/core/cuda/error_check.cuh"

namespace nvblox {
namespace cuda {

template<typename T>
void copy(const int rows, const int cols, const T* from, T* to) {
  checkCudaErrors(
      cudaMemcpy(to, from, rows * cols * sizeof(T), cudaMemcpyDefault));
}

template<typename T>
void toGPU(const T* image, const int rows, const int cols) {
  int device;
  checkCudaErrors(cudaGetDevice(&device));
  checkCudaErrors(cudaMemPrefetchAsync(image, rows * cols * sizeof(T), device));
}

} // namespace cuda
} // namespace nvblox
