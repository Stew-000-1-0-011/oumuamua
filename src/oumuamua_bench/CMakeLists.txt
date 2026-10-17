cmake_minimum_required(VERSION 3.22)
project(oumuamua_bench NONE)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
	scripts/bench_run.py
	scripts/bench_summary.py
	DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
