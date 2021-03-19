cmake_minimum_required(VERSION 3.16.3)

project(controllerCpp)

include(FetchContent)

FetchContent_Declare(
dlibFiles
URL http://dlib.net/files/dlib-19.21.tar.bz2
SOURCE_DIR external/dlib
)


FetchContent_Declare(
armadilloFiles
URL http://sourceforge.net/projects/arma/files/armadillo-10.3.0.tar.xz
SOURCE_DIR external/armadillo
)


if(NOT dlibFiles_POPULATED)
	FetchContent_Populate(dlibDownload)
endif()

if(NOT armadilloFiles_POPULATED)
	FetchContent_Populate(armadilloDownload)
endif()

add_subdirectory(build/external/dlib/dlib dlib_build)
add_subdirectory(build/external/armadillo arma_build)

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
	

	
	
	

