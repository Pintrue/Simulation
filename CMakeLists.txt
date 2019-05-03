# cmake version to be used
cmake_minimum_required(VERSION 3.0)

# project name
project(Simulation)

set (SRC_DIR "src")
set (C "src/robot_reinforcement_learning/C")

# header files location
# include_directories( ${SRC_DIR}/include )

# kdl usage
FIND_PACKAGE(orocos_kdl)
include_directories (${orocos_kdl_INCLUDE_DIRS})

# enable C++11 standard
set (CMAKE_CXX_STANDARD 11)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror -pedantic -O3")
add_compile_definitions(C_AS_LIB=1)

add_executable (simulation ${SRC_DIR}/C_api.cpp ${SRC_DIR}/Sim.cpp
                ${SRC_DIR}/model/KinematicsModel.cpp
                ${SRC_DIR}/model/Trajectory.cpp
                ${SRC_DIR}/model/AngularKinematics.cpp
                ${SRC_DIR}/utils/Utils.cpp
                ${C}/main.c
                ${C}/matrix_op.c
                ${C}/tests.c
                ${C}/utils.c
                ${C}/rl.c
                ${C}/layers.c
                ${C}/model.c
                ${C}/rl_ddpg.c
                ${C}/rl_utils.c)
target_link_libraries (simulation ${orocos_kdl_LIBRARIES})
