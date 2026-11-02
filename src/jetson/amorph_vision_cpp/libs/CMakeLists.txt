# 第三方依赖：OpenCV + Eigen3 + PCL + OpenMP + TensorRT + TensorRT-YOLO + 自定义插件。
#
# 本文件对外只暴露一个接口目标 `${PROJECT_NAME}_libs`，外层直接链接它即可：
#
#     add_subdirectory(libs)
#     target_link_libraries(<target> ${PROJECT_NAME}_libs)
#
# 不使用 libs/TensorRT-YOLO 自带的顶层 CMakeLists，原因：
#   - 它强制要求 -DTRT_PATH，且拼死 ${TRT_PATH}/include 与 ${TRT_PATH}/lib，
#     不接受 Debian multiarch / JetPack 的散装布局；
#   - 带自己的 install 规则，会把 trtyolo.so 装进 ament 前缀；
#   - 会 FORCE 覆盖 CMAKE_CUDA_ARCHITECTURES。
# 因此这里直接编译它的源码。

# ------------------------------------------------------------------------------
# 系统依赖
# ------------------------------------------------------------------------------
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# PCL 的 io 组件会去查找 VTK，而 VTK 的 config 会级联 find_package 几十个依赖，
# 一次 configure 光这就吃掉 ~90s（VTK 被重复查找 3 次）。本项目只用 PCL 的
# PLY 读取与点云算法，都不需要 VTK。
set(CMAKE_DISABLE_FIND_PACKAGE_VTK ON)
find_package(PCL REQUIRED COMPONENTS common io filters features search kdtree)

# 不用 find_package(CUDAToolkit)：它的 FindCUDAToolkit 每次 configure 都要跑几十次
# find_library / find_program，在本机（WSL，CMAKE_PREFIX_PATH 含几十个 install 目录）
# 要 20s 以上。含 .cu 源文件的目标由 CMake 自动链接 CUDA runtime
# （CUDA_RUNTIME_LIBRARY 默认为 Static），不需要 CUDA::cudart。

# ------------------------------------------------------------------------------
# TensorRT 探测
#
# 支持两种安装形态，路径全部来自缓存变量或本文件，不改动系统目录：
#   1) 标准布局：-DTRT_PATH=/opt/tensorrt，其下有 include/ 与 lib/
#   2) Debian/Ubuntu multiarch 或 JetPack 布局：头在 /usr/include/<triplet>，
#      库在 /usr/lib/<triplet>（本机即 /usr/include/x86_64-linux-gnu）
# ------------------------------------------------------------------------------
set(TRT_PATH "" CACHE PATH "TensorRT 根目录（含 include/ 与 lib/）；留空则自动探测系统安装")

if(TRT_PATH)
  set(_trt_include_hints "${TRT_PATH}/include")
  set(_trt_library_hints "${TRT_PATH}/lib" "${TRT_PATH}/lib64")
else()
  set(_trt_include_hints
    /usr/include/x86_64-linux-gnu
    /usr/include/aarch64-linux-gnu
    /usr/local/include
    /usr/include
  )
  set(_trt_library_hints
    /usr/lib/x86_64-linux-gnu
    /usr/lib/aarch64-linux-gnu
    /usr/local/lib
    /usr/lib64
    /usr/lib
  )
endif()

find_path(TRT_INCLUDE_DIR NvInfer.h HINTS ${_trt_include_hints} REQUIRED)
find_library(TRT_NVINFER_LIB NAMES nvinfer HINTS ${_trt_library_hints} REQUIRED)
find_library(TRT_NVINFER_PLUGIN_LIB NAMES nvinfer_plugin HINTS ${_trt_library_hints} REQUIRED)
find_library(TRT_NVONNXPARSER_LIB NAMES nvonnxparser HINTS ${_trt_library_hints})

message(STATUS "TensorRT include: ${TRT_INCLUDE_DIR}")
message(STATUS "TensorRT libs   : ${TRT_NVINFER_LIB}")

file(STRINGS "${TRT_INCLUDE_DIR}/NvInferVersion.h" _trt_major_line
  REGEX "^#define NV_TENSORRT_MAJOR ([0-9]+)")
string(REGEX REPLACE "^#define NV_TENSORRT_MAJOR ([0-9]+).*" "\\1" TRT_VERSION_MAJOR "${_trt_major_line}")
if(NOT TRT_VERSION_MAJOR MATCHES "^[0-9]+$")
  # 较新的头文件用 TRT_MAJOR_ENTERPRISE 间接定义
  file(STRINGS "${TRT_INCLUDE_DIR}/NvInferVersion.h" _trt_ent_line
    REGEX "^#define TRT_MAJOR_ENTERPRISE ([0-9]+)")
  string(REGEX REPLACE "^#define TRT_MAJOR_ENTERPRISE ([0-9]+).*" "\\1" TRT_VERSION_MAJOR "${_trt_ent_line}")
endif()
message(STATUS "TensorRT version: ${TRT_VERSION_MAJOR}.x")

# ------------------------------------------------------------------------------
# trtyolo：推理引擎封装（静态库）
#
# trtyolo.hpp 不引用 TensorRT 头文件，因此 TRT 的 include 目录保持 PRIVATE，
# 不向消费方传播。
# ------------------------------------------------------------------------------
set(TRT_YOLO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/TensorRT-YOLO")
if(NOT EXISTS "${TRT_YOLO_DIR}/modules/trtyolo/infer/trtyolo.hpp")
  message(FATAL_ERROR "TensorRT-YOLO 子模块缺失，请执行 git submodule update --init")
endif()

file(GLOB TRTYOLO_SOURCES CONFIGURE_DEPENDS
  "${TRT_YOLO_DIR}/modules/trtyolo/core/*.cpp"
  "${TRT_YOLO_DIR}/modules/trtyolo/utils/*.cpp"
  "${TRT_YOLO_DIR}/modules/trtyolo/infer/*.cpp"
  "${TRT_YOLO_DIR}/modules/trtyolo/infer/*.cu"
)

add_library(trtyolo STATIC ${TRTYOLO_SOURCES})
set_target_properties(trtyolo PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  CUDA_RESOLVE_DEVICE_SYMBOLS ON
  POSITION_INDEPENDENT_CODE ON
  CXX_VISIBILITY_PRESET hidden
)
target_include_directories(trtyolo
  PUBLIC "${TRT_YOLO_DIR}/modules/trtyolo/infer"
  PRIVATE "${TRT_YOLO_DIR}/modules/trtyolo" "${TRT_INCLUDE_DIR}"
)
target_link_libraries(trtyolo PUBLIC
  ${TRT_NVINFER_LIB}
  ${TRT_NVINFER_PLUGIN_LIB}
  ${TRT_NVONNXPARSER_LIB}
)

# ------------------------------------------------------------------------------
# custom_plugins：Segment / Pose / OBB 的 TensorRT 自定义插件
#
# 只在生成 engine 时由 trtexec --staticPlugins 加载，因此不安装、不进 ament 前缀。
# 产物路径：<build>/libs/libcustom_plugins.so
# ------------------------------------------------------------------------------
option(AMORPH_BUILD_TRT_PLUGINS "构建 TensorRT-YOLO 自定义插件库（生成 engine 用）" ON)
if(AMORPH_BUILD_TRT_PLUGINS)
  file(GLOB CUSTOM_PLUGIN_SOURCES CONFIGURE_DEPENDS
    "${TRT_YOLO_DIR}/modules/plugin/common/*.cpp"
    "${TRT_YOLO_DIR}/modules/plugin/common/*.cu"
    "${TRT_YOLO_DIR}/modules/plugin/efficientIdxNMSPlugin/*.cpp"
    "${TRT_YOLO_DIR}/modules/plugin/efficientIdxNMSPlugin/*.cu"
    "${TRT_YOLO_DIR}/modules/plugin/efficientRotatedNMSPlugin/*.cpp"
    "${TRT_YOLO_DIR}/modules/plugin/efficientRotatedNMSPlugin/*.cu"
  )
  add_library(custom_plugins SHARED ${CUSTOM_PLUGIN_SOURCES})
  set_target_properties(custom_plugins PROPERTIES
    OUTPUT_NAME custom_plugins
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
    CXX_VISIBILITY_PRESET hidden
  )
  target_include_directories(custom_plugins PRIVATE
    "${TRT_YOLO_DIR}/modules/plugin"
    "${TRT_INCLUDE_DIR}"
  )
  target_link_libraries(custom_plugins PRIVATE
    ${TRT_NVINFER_LIB}
    ${TRT_NVINFER_PLUGIN_LIB}
  )
endif()


find_package(OpenMP REQUIRED COMPONENTS CXX)
message(STATUS "OpenMP: ${OpenMP_CXX_FLAGS}")

# pfr 是 header-only，只用它的 INTERFACE 目标 Boost::pfr。
# 它的 test/ 依赖本机没有的 Boost::core / Boost::container_hash，这里临时关掉。
set(_amorph_build_testing_saved ${BUILD_TESTING})
set(BUILD_TESTING OFF)
add_subdirectory(pfr)
set(BUILD_TESTING ${_amorph_build_testing_saved})
unset(_amorph_build_testing_saved)

# ------------------------------------------------------------------------------
# 对外接口目标
#
# 所有第三方依赖都在这里转发给消费方：trtyolo 以 PUBLIC 方式链接 CUDA runtime 与
# TensorRT 库；OpenCV / PCL 走的是变量而不是 IMPORTED target，需要手动转发
# include 目录与编译定义。
# ------------------------------------------------------------------------------
add_library(${PROJECT_NAME}_libs INTERFACE)
target_link_libraries(${PROJECT_NAME}_libs INTERFACE
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  trtyolo
  OpenMP::OpenMP_CXX
  Boost::pfr
)
target_include_directories(${PROJECT_NAME}_libs SYSTEM INTERFACE
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(${PROJECT_NAME}_libs INTERFACE
  ${PCL_DEFINITIONS}
)
