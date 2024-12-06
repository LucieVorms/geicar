cmake_minimum_required(VERSION 3.9.5)
project(space_environment)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -pedantic)
endif()

set(req_deps "rclcpp" "sensor_msgs" "interfaces")

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)

ament_auto_find_build_dependencies(REQUIRED ${req_deps})

include_directories(include)

# build composition node
add_executable(space_environment_node src/space_environment.cpp)
ament_target_dependencies(space_environment_node ${req_deps})

install(
        TARGETS space_environment_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
