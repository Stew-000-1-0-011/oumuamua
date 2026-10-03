cmake_minimum_required(VERSION 3.22)
project(oumuamua_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
