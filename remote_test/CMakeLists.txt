cmake_minimum_required(VERSION 3.8)
project(remote_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(serial_driver REQUIRED)

include_directories(include)

add_executable(remote_test
  src/uart_driver.cpp
  src/remote_test.cpp
  )
ament_target_dependencies(remote_test
  serial_driver
  )

add_executable(uart_test
  src/uart_test.cpp
  src/uart_driver.cpp
  )
ament_target_dependencies(uart_test
  serial_driver
  )

install(TARGETS
  uart_test
  remote_test
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
