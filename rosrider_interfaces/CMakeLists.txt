cmake_minimum_required(VERSION 3.8)
project(rosrider_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Diagnostics.msg"
  "srv/SysCtl.srv"
  "srv/PidCtl.srv"
  "srv/LedCtl.srv"
  "srv/SetInt.srv"
  "srv/SetFloat.srv"
  "srv/DriveCtl.srv"
  "srv/SetRtc.srv"
)

ament_package()
