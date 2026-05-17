cmake_minimum_required(VERSION 3.8)
project(amorph_custom_client)

# C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 依赖 ──
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Qt5 MOC — 不启用 AUTOMOC，改用手动 qt5_wrap_cpp（与 rviz_default_plugins 一致）
# AUTOMOC 生成的 mocs_compilation.cpp 为空 → vtable 无 key function → dlopen 找不到符号
qt5_wrap_cpp(MOC_FILES
  include/amorph_custom_client/operator_panel.hpp
)

# Qt5 UIC — 从 .ui 文件生成 ui_operator_panel_ui.h
qt5_wrap_ui(UI_FILES
  ui/operator_panel.ui
)
# 确保编译 .cpp 前先生成 ui_xxx.h（cmake 不会自动追踪 #include 依赖）
add_custom_target(amorph_generate_ui DEPENDS ${UI_FILES})

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ── 参数库（generate_parameter_library） ──
generate_parameter_library(pick_energy_unit_task_params
  config/pick_energy_unit_task_params.yaml
)

# ── 共享库（插件） ──
include_directories(
  include
  include/amorph_custom_client/mtc
  ${CMAKE_CURRENT_BINARY_DIR}
)

file(GLOB_RECURSE SOURCES
  src/*.cpp
  src/*.cc
)

ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCES} ${MOC_FILES})
add_dependencies(${PROJECT_NAME} amorph_generate_ui)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  pick_energy_unit_task_params
)

# 覆盖 ament 默认的 -fvisibility=hidden，插件 .so 需要导出 vtable 供 dlopen 解析
set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_VISIBILITY_PRESET default
  VISIBILITY_INLINES_HIDDEN OFF
)

# 让 pluginlib 能找到此类
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
