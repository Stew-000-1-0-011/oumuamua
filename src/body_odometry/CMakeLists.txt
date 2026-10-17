cmake_minimum_required(VERSION 3.22)
project(body_odometry CXX)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 26)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(body_odometry_node src/body_odometry_node.cpp)
target_link_libraries(body_odometry_node
	PRIVATE
		rclcpp::rclcpp
		${geometry_msgs_TARGETS}
		${nav_msgs_TARGETS}
		tf2_ros::tf2_ros
)

install(TARGETS body_odometry_node DESTINATION lib/${PROJECT_NAME})

ament_package()
