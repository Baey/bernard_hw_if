cmake_minimum_required(VERSION 3.8)
project(bernard_hw_if)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(CMAKE_INSTALL_RPATH $ORIGIN)

set(MAKE_TESTS OFF)
set(CANDLE_BUILD_STATIC OFF)
add_subdirectory(CANdle-SDK)

# Actuators node
add_executable(actuators_node
 src/actuators_node.cpp
 src/utils.cpp
)
ament_target_dependencies(actuators_node rclcpp std_msgs sensor_msgs)
target_link_libraries(actuators_node candle)
target_include_directories(actuators_node PUBLIC ./CANdle-SDK/candlelib/include ${CMAKE_SOURCE_DIR}/include)

install(TARGETS
  actuators_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
