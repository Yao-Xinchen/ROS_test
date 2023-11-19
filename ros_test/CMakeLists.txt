cmake_minimum_required(VERSION 3.5)
project(ros_test)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(test_interface REQUIRED)

add_executable(add_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_server rclcpp example_interfaces)

add_executable(add_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_client rclcpp example_interfaces)

add_executable(basic_server src/basic_server.cpp)
ament_target_dependencies(basic_server rclcpp test_interface)

add_executable(basic_client src/basic_client.cpp)
ament_target_dependencies(basic_client rclcpp test_interface)

install(TARGETS
  add_server
  add_client
  basic_server
  basic_client
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
