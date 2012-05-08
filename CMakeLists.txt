cmake_minimum_required(VERSION 2.4)
PROJECT (MRP_homework)

SET (CMAKE_MODULE_PATH "/usr/local/dcs/share/cmake/Modules")
#SET (CMAKE_MODULE_PATH "/usr/local/dcs/share/cmake/Modules:/usr/share/cmake-2.8/Modules")
INCLUDE (UsePlayerC++)
find_package(GLUT)
find_package(OpenGL)

PLAYER_ADD_PLAYERCPP_CLIENT (safegoto SOURCES safegoto.cpp robot.cpp LINKFLAGS)
PLAYER_ADD_PLAYERCPP_CLIENT (mapper SOURCES robot.cpp headlessRobot.cpp mapper.cpp LINKFLAGS)
PLAYER_ADD_PLAYERCPP_CLIENT (navigator SOURCES robot.cpp planning.cpp navigator.cpp LINKFLAGS)
target_link_libraries(mapper ${GLUT_LIBRARY} ${OPENGL_LIBRARY} X11 Xext)
