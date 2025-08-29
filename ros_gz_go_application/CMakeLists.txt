cmake_minimum_required(VERSION 3.10)
project(ros_gz_go_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclc REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find Go
find_program(GO_EXECUTABLE go)
if(NOT GO_EXECUTABLE)
    message(FATAL_ERROR "Go not found")
endif()

# Collect all .go files
file(GLOB_RECURSE GO_FILES "${CMAKE_SOURCE_DIR}/*.go")

# go generate target with expected outputs
add_custom_command(
    OUTPUT ${CMAKE_SOURCE_DIR}/cgo-flags.env ${CMAKE_SOURCE_DIR}/msgs
    COMMAND ${GO_EXECUTABLE} generate .
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    DEPENDS ${CMAKE_SOURCE_DIR}/go.mod ${CMAKE_SOURCE_DIR}/go.sum ${CMAKE_SOURCE_DIR}/generate.go ${CMAKE_SOURCE_DIR}/tools.go
    COMMENT "Running go generate"
)

add_custom_target(generate_go
    DEPENDS ${CMAKE_SOURCE_DIR}/cgo-flags.env ${CMAKE_SOURCE_DIR}/msgs
)

# Build Go project using build.sh
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/goros2
    COMMAND ${SH_EXECUTABLE} ${CMAKE_SOURCE_DIR}/build.sh ${GO_EXECUTABLE} ${CMAKE_BINARY_DIR}/goros2
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Building Go CGO project"
    DEPENDS generate_go ${CMAKE_SOURCE_DIR}/build.sh ${CMAKE_SOURCE_DIR}/go.mod ${GO_FILES}
)

add_custom_target(
  build_go ALL
  DEPENDS ${CMAKE_BINARY_DIR}/goros2
)

install(
  PROGRAMS ${CMAKE_BINARY_DIR}/goros2
  DESTINATION lib/${PROJECT_NAME}
)

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
