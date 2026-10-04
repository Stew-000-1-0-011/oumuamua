cmake_minimum_required(VERSION 3.22)
project(oumuamua_sim CXX)

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
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
# フィールドの形状は sotoba_node と同じものを使う
find_package(sotoba_ros REQUIRED)

# --- 平面運動 (ROS非依存) ---
add_library(oumuamua_sim_plant STATIC
	src/plant.cpp
)
target_include_directories(oumuamua_sim_plant
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)

# --- ノード ---
add_executable(robot_sim
	src/robot_sim.cpp
)
target_link_libraries(robot_sim
	PRIVATE
		oumuamua_sim_plant
		sotoba_ros::sotoba_ros_objects
		rclcpp::rclcpp
		${geometry_msgs_TARGETS}
		${nav_msgs_TARGETS}
		${sensor_msgs_TARGETS}
		${std_srvs_TARGETS}
		tf2_ros::tf2_ros
)

install(TARGETS robot_sim
	DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
	add_executable(oumuamua_sim_plant_test test/plant_test.cpp)
	target_link_libraries(oumuamua_sim_plant_test PRIVATE oumuamua_sim_plant)
	ament_add_test(oumuamua_sim_plant
		COMMAND "$<TARGET_FILE:oumuamua_sim_plant_test>"
		GENERATE_RESULT_FOR_RETURN_CODE_ZERO
	)
endif()

ament_package()
