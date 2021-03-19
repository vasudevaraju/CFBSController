cmake_minimum_required(VERSION 3.16.3)

project(controllerCpp)

include(FetchContent)

# Download dlib and armadillo
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

# Populate dlib and armadillo libraries in the directory build/external
if(NOT dlibFiles_POPULATED)
	FetchContent_Populate(dlibFiles)
endif()

if(NOT armadilloFiles_POPULATED)
	FetchContent_Populate(armadilloFiles)
endif()

# CMake the required armadillo and dlib subdirectories as given in their README
add_subdirectory(build/external/dlib/dlib dlib_build)
add_subdirectory(build/external/armadillo arma_build)

# Provide the executable files
# Change the SimulinkPlantLinux.cpp to SimulinkPlantWin.Cpp when compiling in Windows
add_executable(${PROJECT_NAME}
	main.cpp 
	Controller.cpp
	Integrator.h
	CommandFilter1.cpp
	CommandFilter2.cpp
	Saturator.cpp
	SimulinkPlantLinux.cpp
	Simulation.cpp)
	
# Add the build/external directory to the include path of the compiler
target_include_directories(${PROJECT_NAME} PRIVATE build/external)

# Add the compiled armadillo and dlib libraries to the linker path
target_link_libraries(${PROJECT_NAME} dlib::dlib armadillo)
	

	
	
	

