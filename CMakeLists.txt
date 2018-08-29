cmake_minimum_required(VERSION 3.12)
project(mini-ai-cup-3)

set(CMAKE_CXX_STANDARD 11)
set(SOURCE_FILES main.cpp s_olution.cpp c_onstants.cpp s_olver.cpp GameState.cpp GameState.h)
file(GLOB CHIPMUNK_SOURCES chipmunk/src/*.c)
#add_definitions(-DDEBUG)
#set(CMAKE_CXX_FLAGS "-O2")
add_executable(mini-ai-cup-3 ${SOURCE_FILES} ${CHIPMUNK_SOURCES})