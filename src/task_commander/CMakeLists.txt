cmake_minimum_required(VERSION 3.22)
# rosidl は C のターゲットも作るので C も有効にする
project(task_commander C CXX)

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
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gripper REQUIRED)
find_package(path_follower REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- アクション ---
rosidl_generate_interfaces(${PROJECT_NAME}
	"action/RunTask.action"
)
rosidl_get_typesupport_target(task_commander_interfaces_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_export_dependencies(rosidl_default_runtime)

# --- 姿勢と間隔の計算 (ROS非依存) ---
add_library(task_commander_geometry STATIC
	src/geometry.cpp
	src/pickup.cpp
	src/pickup_execution.cpp
)
target_include_directories(task_commander_geometry
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)
set_target_properties(task_commander_geometry PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- ノード ---
add_executable(task_commander_node
	src/task_commander_node.cpp
)
target_link_libraries(task_commander_node
	PRIVATE
		task_commander_geometry
		${task_commander_interfaces_target}
		rclcpp::rclcpp
		rclcpp_action::rclcpp_action
		${std_msgs_TARGETS}
		${std_srvs_TARGETS}
		${nav_msgs_TARGETS}
		${gripper_TARGETS}
		${path_follower_TARGETS}
)

install(TARGETS task_commander_node
	DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
	find_package(Python3 REQUIRED COMPONENTS Interpreter)
	add_executable(pickup_probe test/pickup_probe.cpp)
	target_link_libraries(pickup_probe PRIVATE task_commander_geometry)
	ament_add_test(pickup_geometry_fixture
		COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/pickup_preview.py"
			"$<TARGET_FILE:pickup_probe>" "${CMAKE_CURRENT_BINARY_DIR}/pickup-fixture"
		TIMEOUT 90
		GENERATE_RESULT_FOR_RETURN_CODE_ZERO
	)
	ament_add_test(pickup_ros
		COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/pickup_ros_test.py"
			"$<TARGET_FILE:task_commander_node>"
			"${CMAKE_CURRENT_SOURCE_DIR}/../oumuamua_bringup/config"
		TIMEOUT 90
		GENERATE_RESULT_FOR_RETURN_CODE_ZERO
	)
	foreach(pickup_test_name pickup pickup_execution)
		add_executable(${pickup_test_name}_test test/${pickup_test_name}_test.cpp)
		target_link_libraries(${pickup_test_name}_test PRIVATE task_commander_geometry)
		ament_add_test(${pickup_test_name}
			COMMAND "$<TARGET_FILE:${pickup_test_name}_test>"
			GENERATE_RESULT_FOR_RETURN_CODE_ZERO
		)
	endforeach()
	add_executable(task_commander_geometry_test test/geometry_test.cpp)
	target_link_libraries(task_commander_geometry_test PRIVATE task_commander_geometry)
	ament_add_test(task_commander_geometry
		COMMAND "$<TARGET_FILE:task_commander_geometry_test>"
		GENERATE_RESULT_FOR_RETURN_CODE_ZERO
	)
endif()

ament_package()
