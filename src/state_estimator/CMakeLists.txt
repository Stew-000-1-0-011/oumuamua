cmake_minimum_required(VERSION 3.22)
project(state_estimator CXX)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 26)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sotoba_ros REQUIRED)
find_package(Eigen3 REQUIRED)
# sotoba と一緒に入っている Sophus (1.24 以上)
find_package(Sophus REQUIRED)

# --- フィルタと信念の変換 (ROS非依存) ---
add_library(state_estimator_core STATIC
	src/filter.cpp
	src/belief_geometry.cpp
)
target_include_directories(state_estimator_core
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)
target_link_libraries(state_estimator_core PUBLIC Eigen3::Eigen Sophus::Sophus)
set_target_properties(state_estimator_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- ノード ---
add_executable(state_estimator_node
	src/state_estimator_node.cpp
)
target_link_libraries(state_estimator_node
	PRIVATE
		state_estimator_core
		rclcpp::rclcpp
		${geometry_msgs_TARGETS}
		${nav_msgs_TARGETS}
		${std_srvs_TARGETS}
		${sotoba_ros_TARGETS}
		tf2::tf2
		tf2_ros::tf2_ros
)

install(TARGETS state_estimator_node
	DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
	add_executable(state_estimator_test test/estimator_test.cpp)
	target_link_libraries(state_estimator_test PRIVATE state_estimator_core)
	ament_add_test(state_estimator
		COMMAND "$<TARGET_FILE:state_estimator_test>"
		GENERATE_RESULT_FOR_RETURN_CODE_ZERO
	)
endif()

ament_package()
