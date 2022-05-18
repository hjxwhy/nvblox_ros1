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
#include "nvblox/tests/blox.h"

#include "nvblox/core/layer.h"
#include "nvblox/gpu_hash/cuda/impl/gpu_layer_view_impl.cuh"

namespace nvblox {

using IndexBlockLayer = BlockLayer<IndexBlock>;
template class GPULayerView<IndexBlock>;

using DummyBlockLayer = BlockLayer<DummyBlock>;
template class GPULayerView<DummyBlock>;

} // nvblox
