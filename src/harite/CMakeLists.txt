cmake_minimum_required(VERSION 3.22)
project(harite CXX)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 26)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robomas_plugins REQUIRED)

add_executable(harite_node src/harite_node.cpp)
target_include_directories(harite_node PRIVATE include)
target_link_libraries(harite_node PRIVATE
	rclcpp::rclcpp ${std_msgs_TARGETS} ${std_srvs_TARGETS} ${robomas_plugins_TARGETS}
)
install(TARGETS harite_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
	find_package(ament_cmake_test REQUIRED)
	add_executable(harite_velocity_test test/velocity_test.cpp)
	target_include_directories(harite_velocity_test PRIVATE include)
	ament_add_test(harite_velocity
		COMMAND "$<TARGET_FILE:harite_velocity_test>"
		GENERATE_RESULT_FOR_RETURN_CODE_ZERO
	)
endif()

ament_package()
