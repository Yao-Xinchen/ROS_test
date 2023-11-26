cmake_minimum_required(VERSION 3.8)
project(dm_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(test_interface REQUIRED)

include_directories(include)

add_executable(dm_test_active
  src/dm_test_active.cpp
  src/dm_driver.cpp)
ament_target_dependencies(dm_test_active
  rclcpp test_interface)

add_executable(dm_test_passive
  src/dm_test_passive.cpp
  src/dm_driver.cpp)
ament_target_dependencies(dm_test_passive
  rclcpp test_interface)

install(TARGETS dm_test_active dm_test_passive
  DESTINATION lib/${PROJECT_NAME})

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
