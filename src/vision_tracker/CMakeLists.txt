cmake_minimum_required(VERSION 3.5)
project(vision_tracker)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(aircraft_msgs REQUIRED)

include_directories(include)

add_library(vision_tracker_mode SHARED
    src/vision_tracker_mode.cpp
)

ament_target_dependencies(vision_tracker_mode
    rclcpp
    px4_msgs
    aircraft_msgs
    px4_ros2_cpp
    Eigen3
)

add_executable(vision_tracker
    src/main.cpp
)

target_link_libraries(vision_tracker
    vision_tracker_mode
)

ament_target_dependencies(vision_tracker
    rclcpp
    px4_msgs
    aircraft_msgs
    px4_ros2_cpp
    Eigen3
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS vision_tracker_mode
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(
    TARGETS vision_tracker
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(vision_tracker_mode)

ament_package()
