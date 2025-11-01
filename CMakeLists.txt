cmake_minimum_required(VERSION 3.8)
project(vector_field_simul)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

include_directories(include include/vector_field_simul)
add_executable(run_vector_field 
  src/field_node.cpp
  src/algorithm.cpp
)
ament_target_dependencies(run_vector_field 
  rclcpp
  geometry_msgs
  example_interfaces
  nav_msgs)

target_link_libraries(run_vector_field "${cpp_typesupport_target}")

install(
  TARGETS run_vector_field
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY launch description worlds config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
