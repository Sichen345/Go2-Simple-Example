cmake_minimum_required(VERSION 3.0.2)
project(BaseEnvironment)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT}) 
find_package(raisim CONFIG REQUIRED)   

include_directories(${Eigen3_INCLUDE_DIRS})

set(CONTROL_LIBRARIES
	raisim::raisim
	pthread)

set(CONTROL_SOURCES
	src/Robot/YourRobot.cpp
	src/Robot/Simulation.cpp)

add_executable(control-ex ${CONTROL_SOURCES})
target_link_libraries(control-ex ${CONTROL_LIBRARIES})
