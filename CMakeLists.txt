cmake_minimum_required(VERSION 2.4)
PROJECT (MRP_homework)

SET (CMAKE_MODULE_PATH "/usr/local/dcs/share/cmake/Modules/")
INCLUDE (UsePlayerC++)

PLAYER_ADD_PLAYERCPP_CLIENT (safegoto SOURCES safegoto.cpp robot.cpp LINKFLAGS)
