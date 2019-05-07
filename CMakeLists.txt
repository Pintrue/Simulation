# cmake version to be used
cmake_minimum_required(VERSION 3.0)

# project name
project(Simulation)

set (SRC_DIR "src")
set (C "src/robot_reinforcement_learning/C")
set (GRA "src/GUI")
set(QT_OPENGL_SRCS
	${GRA}/logo.cpp
	${GRA}/window.cpp
	${GRA}/glwidget.cpp
    ${GRA}/mainwindow.cpp
    ${GRA}/main.cpp
)
set(QT_OPENGL_FILES
	${GRA}/logo.cpp
	${GRA}/QtWindow.cpp
	${GRA}/GLWidgets.cpp
    ${GRA}/QtMainWindow.cpp
    ${GRA}/main.cpp
)

# header files location
# include_directories( ${SRC_DIR}/include )

# kdl usage
FIND_PACKAGE(orocos_kdl)
include_directories (${orocos_kdl_INCLUDE_DIRS})

# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)

# QT5 usage
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
# FIND_PACKAGE(Qt5Widgets CONFIG REQUIRED)
SET( QT_USE_QTGUI TRUE )
SET( QT_USE_QTOPENGL TRUE )
#INCLUDE(${QT5_USE_FILE})

# enable C++11 standard
set (CMAKE_CXX_STANDARD 11)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror -pedantic -O3")
add_compile_definitions(C_AS_LIB=1)

add_executable (simulation ${QT_OPENGL_FILES} ${SRC_DIR}/C_api.cpp ${SRC_DIR}/Sim.cpp
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
target_link_libraries (simulation ${orocos_kdl_LIBRARIES} Qt5::Widgets)
