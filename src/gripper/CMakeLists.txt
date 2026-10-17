cmake_minimum_required(VERSION 3.22)
# rosidl は C のターゲットも作るので C も有効にする
project(gripper C CXX)

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
find_package(robomas_plugins REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- アクションとメッセージ ---
rosidl_generate_interfaces(${PROJECT_NAME}
	"action/MoveLift.action"
	"msg/GripperState.msg"
	DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(gripper_interfaces_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_export_dependencies(rosidl_default_runtime)

# --- 昇降の計算 (ROS非依存) ---
add_library(gripper_lift STATIC
	src/lift.cpp
)
target_include_directories(gripper_lift
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)
set_target_properties(gripper_lift PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- ノード ---
add_executable(gripper_node
	src/gripper_node.cpp
)
target_link_libraries(gripper_node
	PRIVATE
		gripper_lift
		${gripper_interfaces_target}
		rclcpp::rclcpp
		rclcpp_action::rclcpp_action
		${std_msgs_TARGETS}
		${std_srvs_TARGETS}
		${robomas_plugins_TARGETS}
)

install(TARGETS gripper_node
	DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
	# フィードバックの読み取り・回転の数え上げ・指令の動かし方のテスト (ROS不要)
	add_executable(gripper_lift_test test/lift_test.cpp)
	target_link_libraries(gripper_lift_test PRIVATE gripper_lift)
	ament_add_test(gripper_lift
		COMMAND "$<TARGET_FILE:gripper_lift_test>"
		GENERATE_RESULT_FOR_RETURN_CODE_ZERO
	)
endif()

ament_package()
