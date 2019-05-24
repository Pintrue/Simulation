# cmake version to be used
cmake_minimum_required(VERSION 2.8)

# A few env variable
set(ENV{PATH} "/usr/local/opt/qt/bin:$ENV{PATH}")
# project name
project(Simulation)

OPTION(GUI "Build with Graphical User Interface" OFF)
OPTION(LINUX "Build under linux env" OFF)

set(SRC_DIR "src")
set(C "src/robot_reinforcement_learning/C")
set(GRA "src/GUI")

set(QT_OPENGL_FILES
	${GRA}/Drawable.cpp
	${GRA}/GLGraphics.cpp
	${GRA}/QtWindow.cpp
	${GRA}/GLWidgets.cpp
    ${GRA}/QtMainWindow.cpp
	${GRA}/UIUtils.cpp
)

set(KM_FILES
	${SRC_DIR}/Sim.cpp
    ${SRC_DIR}/model/KinematicsModel.cpp
    ${SRC_DIR}/model/Trajectory.cpp
    ${SRC_DIR}/model/AngularKinematics.cpp
	${SRC_DIR}/model/Constraints.cpp
    ${SRC_DIR}/utils/Utils.cpp
)

# kdl usage
find_package(orocos_kdl)
include_directories(${orocos_kdl_INCLUDE_DIRS})


# enable C++11 standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

#enable C11 standard
set(CMAKE_C_STANDARD 11)

if(GUI STREQUAL "ON")
	# Instruct CMake to run moc automatically when needed
	set(CMAKE_AUTOMOC ON)

	# Create code from a list of Qt designer ui files
	set(CMAKE_AUTOUIC ON)

	# QT5 usage
	find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
	set(QT_USE_QTGUI TRUE)
	set(QT_USE_QTOPENGL TRUE)
	
	add_executable(simulation ${GRA}/GUIMain.cpp
					${QT_OPENGL_FILES}
					${KM_FILES}
	)

	target_link_libraries(simulation ${orocos_kdl_LIBRARIES} Qt5::Widgets "-framework OpenGL")

else()
	if (LINUX STREQUAL "ON")
		set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11 -Wall -Werror -pedantic -DMKL_ILP64 -m64 -I${MKLROOT}/include -O3")
		set(SIM_FILES
			${SRC_DIR}/C_api.cpp
			${KM_FILES}

		)
	else()
		# Instruct CMake to run moc automatically when needed
		set(CMAKE_AUTOMOC ON)

		# Create code from a list of Qt designer ui files
		set(CMAKE_AUTOUIC ON)

		# QT5 usage
		find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
		set(QT_USE_QTGUI TRUE)
		set(QT_USE_QTOPENGL TRUE)
		
		add_compile_definitions(RENDER=1)
		set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11 -Wall -Werror -pedantic -O3")
		set(SIM_FILES
			${SRC_DIR}/C_api.cpp
			${KM_FILES}
			${QT_OPENGL_FILES}

		)
	endif()
	add_compile_definitions(C_AS_LIB=1)
	add_executable(simulation
					${SIM_FILES}
                	${C}/main.c
                	${C}/matrix_op.c
                	${C}/tests.c
                	${C}/utils.c
                	${C}/rl.c
                	${C}/layers.c
                	${C}/model.c
                	${C}/rl_ddpg.c
                	${C}/rl_utils.c
					${C}/socket_utils.c
					${C}/optimizer.c
					${C}/model_utils.c
					${C}/rl_ddpg_her.c
					${C}/normalizer.c
					${C}/test_agent.c
					${C}/rl_ddpg_her_demo.c
					${C}/rl_ddpg_her_sim.c
					${SRC_DIR}/model/FwdKM.c
					${SRC_DIR}/model/InvKM.c
	)
	if (LINUX STREQUAL "OFF")
		target_link_libraries(simulation ${orocos_kdl_LIBRARIES} Qt5::Widgets "-framework OpenGL" $ENV{MKLROOT}/lib/libmkl_intel_ilp64.a $ENV{MKLROOT}/lib/libmkl_sequential.a $ENV{MKLROOT}/lib/libmkl_core.a -lpthread -lm -ldl)
	else()
		target_link_libraries(simulation ${orocos_kdl_LIBRARIES} $ENV{MKLROOT}/lib/intel64/libmkl_intel_ilp64.a $ENV{MKLROOT}/lib/intel64/libmkl_sequential.a $ENV{MKLROOT}/lib/intel64/libmkl_core.a -lpthread -lm -ldl)
	endif()
endif()
unset(GUI)