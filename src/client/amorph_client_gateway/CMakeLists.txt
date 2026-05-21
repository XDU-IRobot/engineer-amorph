cmake_minimum_required(VERSION 3.8)
project(amorph_client_gateway)

# GStreamer — not a ROS package, must be found manually
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)

# Protobuf — needed only for code generation (linking via libprotobuf-dev in package.xml)
find_package(Protobuf REQUIRED)

# ament_auto handles all <depend> entries in package.xml automatically
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Third-party MQTT C++ library
add_subdirectory(libs)

# -- Protobuf code generation ------------------------------------------------
set(PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
set(PROTO_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}")
file(MAKE_DIRECTORY ${PROTO_GEN_DIR})

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_DIR}/custom_byte_block.proto ${PROTO_DIR}/custom_control.proto)
set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS} PROPERTIES GENERATED TRUE)

##############
# VT Receiver node  (ament_auto handles: rclcpp, cv_bridge, OpenCV, image_transport, camera_info_manager, backward_ros, ...)
###############
ament_auto_add_library(vt_receiver_component SHARED
  src/fragment_parser.cc
  src/udp_video_receiver_node.cc
  src/video_decoder.cc
)
target_link_libraries(vt_receiver_component
  ${PROJECT_NAME}_libs
  ${GSTREAMER_LIBRARIES}
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
# MQTT Gateway node  (ament_auto handles: rclcpp, protobuf, ...)
###############
ament_auto_add_library(mqtt_gateway_component SHARED
  src/mqtt_gateway_node.cc
  src/mqtt_client.cc
  src/custom_byte_block_handler.cc
  ${PROTO_SRCS}
)
target_link_libraries(mqtt_gateway_component
  ${PROJECT_NAME}_libs
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

# Install calibration YAML so package://amorph_client_gateway/config/ works
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
