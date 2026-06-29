cmake_minimum_required(VERSION 3.8)
project(spot_multicam_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Register the message file with rosidl.
# rosidl_generate_interfaces() generates Python and C++ bindings
# for CaptureMetadata.msg automatically during colcon build.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CaptureMetadata.msg"
  "action/CaptureAction.action"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
