cmake_minimum_required(VERSION 3.5)
project(aircraft_controller)

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
find_package(vision_tracker REQUIRED)

include_directories(include)

add_executable(mission_runner
    src/mission_runner.cpp
    src/actions/vtol_actions.cpp
    src/trajectory/fw_trajectory_executor.cpp
)


ament_target_dependencies(mission_runner
    rclcpp
    px4_msgs
    aircraft_msgs
	vision_tracker
    px4_ros2_cpp
    Eigen3
)

install(
    TARGETS mission_runner
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
