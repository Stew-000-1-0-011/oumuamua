cmake_minimum_required(VERSION 3.22)
# rosidl は C のターゲットも作るので C も有効にする
project(path_follower C CXX)

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
find_package(holonomic_tracker REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- アクション ---
rosidl_generate_interfaces(${PROJECT_NAME}
	"action/FollowPath.action"
)
rosidl_get_typesupport_target(path_follower_interfaces_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_export_dependencies(rosidl_default_runtime)

# --- 経路の読み込みと速度計画 (ROS非依存) ---
add_library(path_follower_core STATIC
	src/trajectory.cpp
)
target_include_directories(path_follower_core
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)
set_target_properties(path_follower_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- ノード ---
add_executable(path_follower_node
	src/path_follower_node.cpp
)
target_link_libraries(path_follower_node
	PRIVATE
		path_follower_core
		${path_follower_interfaces_target}
		rclcpp::rclcpp
		rclcpp_action::rclcpp_action
		${geometry_msgs_TARGETS}
		${nav_msgs_TARGETS}
		${std_srvs_TARGETS}
		${holonomic_tracker_TARGETS}
)

install(TARGETS path_follower_node
	DESTINATION lib/${PROJECT_NAME}
)
# 経路ファイルを作るスクリプト
install(PROGRAMS scripts/make_path.py
	DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
	add_executable(path_follower_test test/trajectory_test.cpp)
	target_link_libraries(path_follower_test PRIVATE path_follower_core)
	ament_add_test(path_follower
		COMMAND "$<TARGET_FILE:path_follower_test>"
		GENERATE_RESULT_FOR_RETURN_CODE_ZERO
	)
endif()

ament_package()
