cmake_minimum_required(VERSION 3.8)
project(w5500_msg)

# Compiler options (future-proof)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ------------------------------------------------------------------------------
# Message generation
# ------------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ForceF3.msg"
  "msg/ForceF4.msg"
  DEPENDENCIES builtin_interfaces
)

# ------------------------------------------------------------------------------
# Testing
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ------------------------------------------------------------------------------
# Export
# ------------------------------------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)
ament_package()
