cmake_minimum_required(VERSION 3.22)
project(oumuamua_bringup NONE)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
	find_package(Python3 REQUIRED COMPONENTS Interpreter)
	ament_add_test(pickup_parameters
		COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/pickup_parameters_test.py"
		GENERATE_RESULT_FOR_RETURN_CODE_ZERO
	)
endif()

install(DIRECTORY config launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
