include(FetchContent)
FetchContent_Declare(
  ext_stdgpu
  PREFIX stdgpu
  GIT_REPOSITORY https://github.com/stotko/stdgpu.git
  GIT_TAG        e10f6f3ccc9902d693af4380c3bcd188ec34a2e6
  UPDATE_COMMAND ""
)

# stdgpu build options
set(STDGPU_BUILD_SHARED_LIBS ON)
set(STDGPU_BUILD_EXAMPLES OFF)
set(STDGPU_BUILD_TESTS OFF)
set(STDGPU_ENABLE_CONTRACT_CHECKS OFF)

set(STDGPU_BACKEND_DIRECTORY "cuda") # DO we need this?

# Download the files
FetchContent_MakeAvailable(ext_stdgpu)

# Grabbing the compute capability through functions defined by stdgpu
# https://github.com/stotko/stdgpu/tree/master/cmake/cuda

set(CMAKE_MODULE_PATH_OLD ${CMAKE_MODULE_PATH})
# Temporary replace CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH "${stdgpu_SOURCE_DIR}/cmake/cuda")

include("${stdgpu_SOURCE_DIR}/cmake/cuda/set_device_flags.cmake")

stdgpu_set_device_flags(STDGPU_DEVICE_FLAGS)
stdgpu_cuda_set_architecture_flags(STDGPU_CUDA_ARCHITECTURE_FLAGS)
if(STDGPU_CUDA_ARCHITECTURE_FLAGS)
    if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.18)
        set(CMAKE_CUDA_ARCHITECTURES ${STDGPU_CUDA_ARCHITECTURE_FLAGS})
    else()
        string(APPEND CMAKE_CUDA_FLAGS "${STDGPU_CUDA_ARCHITECTURE_FLAGS}")
        message(STATUS "Building with modified CMAKE_CUDA_FLAGS : ${CMAKE_CUDA_FLAGS}")
    endif()
else()
    message(WARNING "Falling back to default CCs : ${CMAKE_CUDA_ARCHITECTURES}")
endif()

# Restore CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH_OLD})
