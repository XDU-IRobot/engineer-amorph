cmake_minimum_required(VERSION 3.8)
project(amorph_client_gateway)

set(CMAKE_CXX_STANDARD 20)
add_definitions(-std=c++20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(backward_ros REQUIRED)

# GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# Protobuf
find_package(Protobuf REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_subdirectory(libs)

include_directories(include/amorph_client_gateway)

# -- Protobuf code generation ------------------------------------------------
set(PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
set(PROTO_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}")
file(MAKE_DIRECTORY ${PROTO_GEN_DIR})

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_DIR}/custom_byte_block.proto ${PROTO_DIR}/custom_control.proto)

# Workaround: protobuf_generate_cpp puts generated files relative to current src dir;
# we want them under proto_gen/ for isolation.
set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS} PROPERTIES GENERATED TRUE)

include_directories(${PROTO_GEN_DIR})
include_directories(${PROTOBUF_INCLUDE_DIRS})

##############
# VT Receiver node
###############
ament_auto_add_library(vt_receiver_component SHARED
  src/fragment_parser.cc
  src/udp_video_receiver_node.cc
  src/video_decoder.cc
)
target_link_libraries(vt_receiver_component
  ${PROJECT_NAME}_libs
  ${GSTREAMER_LIBRARIES}
  ${OpenCV_LIBRARIES}
  cv_bridge::cv_bridge
)
target_include_directories(vt_receiver_component PRIVATE
  ${GSTREAMER_INCLUDE_DIRS}
)
target_compile_options(vt_receiver_component PRIVATE
  ${GSTREAMER_CFLAGS_OTHER}
)
rclcpp_components_register_node(vt_receiver_component
  PLUGIN "amorph::UdpVideoReceiverNode"
  EXECUTABLE vt_receiver_node
)

##############
# MQTT Gateway node
###############
ament_auto_add_library(mqtt_gateway_component SHARED
  src/mqtt_gateway_node.cc
  ${PROTO_SRCS}
)
target_link_libraries(mqtt_gateway_component
  ${PROJECT_NAME}_libs
  ${PROTOBUF_LIBRARIES}
)
target_include_directories(mqtt_gateway_component PRIVATE
  ${PROTO_GEN_DIR}
)
# Suppress protobuf deprecated warnings from generated code
set_source_files_properties(${PROTO_SRCS} PROPERTIES COMPILE_FLAGS "-Wno-deprecated-declarations")
rclcpp_components_register_node(mqtt_gateway_component
  PLUGIN "amorph::MqttGatewayNode"
  EXECUTABLE mqtt_gateway_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
