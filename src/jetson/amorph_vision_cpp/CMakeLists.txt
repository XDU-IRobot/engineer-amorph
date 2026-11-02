cmake_minimum_required(VERSION 3.16)
project(amorph_vision_cpp)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 目标 GPU 架构：默认同时编 Jetson Orin (87) 与桌面 Ampere/Ada (89)。
# 只在一台机器上跑时可 -DAMORPH_CUDA_ARCHITECTURES=87 缩短编译时间。
set(AMORPH_CUDA_ARCHITECTURES "87;89" CACHE STRING "CUDA 架构列表，传给 CMAKE_CUDA_ARCHITECTURES")

enable_language(CUDA)
set(CMAKE_CUDA_ARCHITECTURES ${AMORPH_CUDA_ARCHITECTURES})

find_package(ament_cmake_auto REQUIRED)

# ------------------------------------------------------------------------------
# ROS 依赖
# ------------------------------------------------------------------------------
ament_auto_find_build_dependencies()

# ------------------------------------------------------------------------------
# 第三方依赖（OpenCV / Eigen3 / PCL / OpenMP / pfr / TensorRT / TensorRT-YOLO）
# 全部在 libs/ 里维护，对外只暴露接口目标 ${PROJECT_NAME}_libs。
# ------------------------------------------------------------------------------
add_subdirectory(libs)

# ------------------------------------------------------------------------------
# 本包的库与节点
# ------------------------------------------------------------------------------
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES CONFIGURE_DEPENDS src/*.cc)
list(FILTER ${PROJECT_NAME}_SOURCES EXCLUDE REGEX "src/(main|debug_infer|debug_pose)\\.cc$")

ament_auto_add_library(${PROJECT_NAME}_core SHARED ${${PROJECT_NAME}_SOURCES})

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
# ament_auto_add_library 用的是 plain 签名，这里必须保持一致
target_link_libraries(${PROJECT_NAME}_core ${PROJECT_NAME}_libs)

ament_auto_add_executable(${PROJECT_NAME}_node src/main.cc)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)

# 离线调试：对单张图片跑完整推理，打印检测结果，便于在不启动 ROS 的情况下验证 engine
ament_auto_add_executable(${PROJECT_NAME}_debug_infer src/debug_infer.cc)
target_link_libraries(${PROJECT_NAME}_debug_infer ${PROJECT_NAME}_core)

# 离线调试：对单帧图片+深度跑完整位姿估计（YOLO → PCA → ICP）
ament_auto_add_executable(${PROJECT_NAME}_debug_pose src/debug_pose.cc)
target_link_libraries(${PROJECT_NAME}_debug_pose ${PROJECT_NAME}_core)

# 只安装运行时需要的资产。best.pt / best-trtyolo.onnx 是生成 engine 的中间产物，
# 运行时用不到，装进 share 只会白占空间。engine 与 TensorRT 版本绑定且被 gitignore，
# 本机没生成时跳过即可（节点会在启动时报错并提示生成方法）。
install(FILES resource/energy_unit.ply
  DESTINATION share/${PROJECT_NAME}/resource
)
install(FILES resource/best.engine
  DESTINATION share/${PROJECT_NAME}/resource
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 仓库统一用 clang-format（Google 风格、120 列），与 ament 的 uncrustify 冲突；
  # 本包没有 Python 代码；libs/ 下是第三方子模块，也不应由本包 lint。
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_cppcheck
    ament_cmake_copyright
    ament_cmake_flake8
    ament_cmake_pep257
  )
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_geometry test/test_geometry.cc)
  if(TARGET test_geometry)
    target_link_libraries(test_geometry ${PROJECT_NAME}_core)
    target_compile_definitions(test_geometry PRIVATE
      AMORPH_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resource")
  endif()

  ament_add_gtest(test_kalman_tuning test/test_kalman_tuning.cc)
  if(TARGET test_kalman_tuning)
    target_link_libraries(test_kalman_tuning ${PROJECT_NAME}_core)
  endif()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
