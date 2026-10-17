cmake_minimum_required(VERSION 3.22)
project(gicp_localizer CXX)

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
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(small_gicp REQUIRED)

add_executable(gicp_localizer src/gicp_localizer.cpp)
target_link_libraries(gicp_localizer
	PRIVATE
		rclcpp::rclcpp
		${geometry_msgs_TARGETS}
		${sensor_msgs_TARGETS}
		tf2::tf2
		tf2_ros::tf2_ros
		Eigen3::Eigen
		OpenMP::OpenMP_CXX
		small_gicp::small_gicp
)

install(TARGETS gicp_localizer DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
