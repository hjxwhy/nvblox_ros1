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

#include <cuda_runtime.h>
#include <glog/logging.h>
#include <algorithm>
#include <cstring>
#include <type_traits>

#include "nvblox/core/cuda/error_check.cuh"

namespace nvblox {

// Constructors and destructors.
template <typename T>
unified_vector<T>::unified_vector(MemoryType memory_type)
    : memory_type_(memory_type),
      buffer_(nullptr),
      buffer_size_(0),
      buffer_capacity_(0) {
  static_assert(std::is_default_constructible<T>::value,
                "Need to have a default constructor to use unified_vector.");
  static_assert(std::is_trivially_destructible<T>::value,
                "Need to have a trivial destructor to use unified_vector.");
}

template <typename T>
unified_vector<T>::unified_vector(size_t size, MemoryType memory_type)
    : unified_vector(memory_type) {
  resize(size);
}

template <typename T>
unified_vector<T>::unified_vector(size_t size, const T& initial,
                                  MemoryType memory_type)
    : unified_vector(memory_type) {
  CHECK_NE(static_cast<int>(memory_type_),
           static_cast<int>(MemoryType::kDevice))
      << "Can't set initial values for device memory. Use copy instead.";
  resize(size);
  for (size_t i = 0; i < size; i++) {
    buffer_[i] = initial;
  }
}

// Copy constructor.
template <typename T>
unified_vector<T>::unified_vector(const unified_vector<T>& other,
                                  MemoryType memory_type)
    : unified_vector(memory_type) {
  resize(other.size());

  // Do a cuda memcpy on this, which will hopefully figure out the 2 data types
  // correctly.
  checkCudaErrors(cudaMemcpy(buffer_, other.data(), sizeof(T) * other.size(),
                             cudaMemcpyDefault));
}

template <typename T>
unified_vector<T>::unified_vector(const std::vector<T>& other,
                                  MemoryType memory_type)
    : unified_vector(memory_type) {
  resize(other.size());

  checkCudaErrors(cudaMemcpy(buffer_, other.data(), sizeof(T) * other.size(),
                             cudaMemcpyDefault));
}

// Move constructor.
template <typename T>
unified_vector<T>::unified_vector(unified_vector<T>&& other)
    : memory_type_(other.memory_type_),
      buffer_(other.buffer_),
      buffer_size_(other.buffer_size_),
      buffer_capacity_(other.buffer_capacity_) {
  other.buffer_ = nullptr;
  other.buffer_size_ = 0;
  other.buffer_capacity_ = 0;
}

template <typename T>
unified_vector<T>::~unified_vector() {
  clear();
}

// Operators
template <typename T>
T& unified_vector<T>::operator[](size_t index) {
  return buffer_[index];
}

template <typename T>
const T& unified_vector<T>::operator[](size_t index) const {
  return buffer_[index];
}

// Copy assignment
template <typename T>
unified_vector<T>& unified_vector<T>::operator=(
    const unified_vector<T>& other) {
  resize(other.size());
  checkCudaErrors(cudaMemcpy(buffer_, other.data(), sizeof(T) * other.size(),
                             cudaMemcpyDefault));
  return *this;
}

// Move assignment
template <typename T>
unified_vector<T>& unified_vector<T>::operator=(unified_vector<T>&& other) {
  clear();
  buffer_ = other.buffer_;
  buffer_size_ = other.buffer_size_;
  buffer_capacity_ = other.buffer_capacity_;
  other.buffer_ = nullptr;
  other.buffer_size_ = 0;
  other.buffer_capacity_ = 0;
  return *this;
}

template <typename T>
unified_vector<T>& unified_vector<T>::operator=(const std::vector<T>& other) {
  resize(other.size());
  checkCudaErrors(cudaMemcpy(buffer_, other.data(), sizeof(T) * other.size(),
                             cudaMemcpyDefault));
  return *this;
}

template <typename T>
std::vector<T> unified_vector<T>::toVector() const {
  std::vector<T> vect(buffer_size_);
  checkCudaErrors(cudaMemcpy(vect.data(), buffer_, sizeof(T) * buffer_size_,
                             cudaMemcpyDefault));
  return vect;
}

// Get those raw pointers.
template <typename T>
T* unified_vector<T>::data() {
  return buffer_;
}

template <typename T>
const T* unified_vector<T>::data() const {
  return buffer_;
}

// Hint to move the memory to the GPU.
template <typename T>
void unified_vector<T>::toGPU() {
  if (buffer_ == nullptr || buffer_capacity_ == 0) {
    return;
  }
  int device = 0;
  checkCudaErrors(cudaGetDevice(&device));
  checkCudaErrors(
      cudaMemPrefetchAsync(buffer_, buffer_capacity_ * sizeof(T), device));
}

template <typename T>
void unified_vector<T>::toCPU() {
  if (buffer_ == nullptr || buffer_capacity_ == 0) {
    return;
  }
  checkCudaErrors(cudaMemPrefetchAsync(buffer_, buffer_capacity_ * sizeof(T),
                                       cudaCpuDeviceId));
}

// Accessors
template <typename T>
size_t unified_vector<T>::capacity() const {
  return buffer_capacity_;
}

template <typename T>
size_t unified_vector<T>::size() const {
  return buffer_size_;
}

template <typename T>
bool unified_vector<T>::empty() const {
  return buffer_size_ == 0;
}

// Changing the size.
template <typename T>
void unified_vector<T>::reserve(size_t capacity) {
  if (buffer_capacity_ < capacity) {
    // Create a new buffer.
    T* new_buffer = nullptr;

    if (memory_type_ == MemoryType::kUnified) {
      checkCudaErrors(cudaMallocManaged(&new_buffer, sizeof(T) * capacity,
                                        cudaMemAttachGlobal));
    } else if (memory_type_ == MemoryType::kDevice) {
      checkCudaErrors(cudaMalloc(&new_buffer, sizeof(T) * capacity));
    } else {
      checkCudaErrors(cudaMallocHost(&new_buffer, sizeof(T) * capacity));
    }

    if (buffer_ != nullptr) {
      // Copy the old values to the new buffer.
      checkCudaErrors(cudaMemcpy(new_buffer, buffer_, sizeof(T) * buffer_size_,
                                 cudaMemcpyDefault));

      // Delete the old buffer.
      if (memory_type_ == MemoryType::kHost) {
        checkCudaErrors(cudaFreeHost(reinterpret_cast<void*>(buffer_)));
      } else {
        checkCudaErrors(cudaFree(reinterpret_cast<void*>(buffer_)));
      }
    }
    buffer_ = new_buffer;
    buffer_capacity_ = capacity;
  }
}

template <typename T>
void unified_vector<T>::resize(size_t size) {
  // Absolute no-op.
  if (buffer_size_ == size) {
    return;
  } else {
    reserve(size);
    buffer_size_ = size;
  }
}

template <typename T>
void unified_vector<T>::clear() {
  if (buffer_ != nullptr) {
    if (memory_type_ == MemoryType::kHost) {
      checkCudaErrors(cudaFreeHost(reinterpret_cast<void*>(buffer_)));
    } else {
      checkCudaErrors(cudaFree(reinterpret_cast<void*>(buffer_)));
    }
  }
  buffer_ = nullptr;
  buffer_size_ = 0;
  buffer_capacity_ = 0;
}

template <typename T>
void unified_vector<T>::push_back(const T& value) {
  CHECK_NE(static_cast<int>(memory_type_),
           static_cast<int>(MemoryType::kDevice))
      << "Can't use push_back on device memory.";

  if (buffer_capacity_ < buffer_size_ + 1) {
    // Do the vector thing and reserve double.
    reserve(std::max(buffer_size_ * 2, buffer_size_ + 1));
  }
  // Add the new value to the end.
  buffer_[buffer_size_] = value;
  buffer_size_++;
}

template <typename T>
typename unified_vector<T>::iterator unified_vector<T>::begin() {
  return iterator(&buffer_[0]);
}

template <typename T>
typename unified_vector<T>::iterator unified_vector<T>::end() {
  return iterator(&buffer_[buffer_size_]);
}

template <typename T>
typename unified_vector<T>::const_iterator unified_vector<T>::cbegin() const {
  return const_iterator(&buffer_[0]);
}

template <typename T>
typename unified_vector<T>::const_iterator unified_vector<T>::cend() const {
  return const_iterator(&buffer_[buffer_size_]);
}

template <typename T>
void unified_vector<T>::setZero() {
  // It is safe to use cudaMemset since the memory is ALWAYS allocated with
  // cudaMalloc.
  checkCudaErrors(cudaMemset(buffer_, 0, buffer_size_ * sizeof(T)));
}

}  // namespace nvblox
