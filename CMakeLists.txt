# cmake version to be used
cmake_minimum_required(VERSION 3.0)

# project name
project(Simulation)

set (SRC_DIR "src")

# header files location
include_directories( ${SRC_DIR}/include )

# kdl usage
FIND_PACKAGE(orocos_kdl)
include_directories (${orocos_kdl_INCLUDE_DIRS})

# enable C++11 standard
set(CMAKE_CXX_STANDARD 11)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

add_executable (simulation ${SRC_DIR}/SimpleFK.cpp ${SRC_DIR}/Utils.cpp)
target_link_libraries (simulation ${orocos_kdl_LIBRARIES})
