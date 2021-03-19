cmake_minimum_required(VERSION 3.16.3)

project(controllerCpp)

add_subdirectory(dlib/dlib dlib_build)
add_subdirectory(armadillo-10.3.0 arma_build)

add_executable(${PROJECT_NAME}
	main.cpp 
	Controller.cpp
	Integrator.h
	CommandFilter1.cpp
	CommandFilter2.cpp
	Saturator.cpp
	SimulinkPlantLinux.cpp
	Simulation.cpp)
	

	
target_link_libraries(${PROJECT_NAME} dlib::dlib armadillo)
	

	
	
	

