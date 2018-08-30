cmake_minimum_required(VERSION 3.12)
project(mini-ai-cup-3)

# Add DEBUG directive
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")


set(CMAKE_CXX_STANDARD 11)
set(SOURCE_FILES main.cpp Solution.cpp Constants.cpp Solver.cpp GameState.cpp Randomizer.cpp)
file(GLOB CHIPMUNK_SOURCES chipmunk/src/*.c)
add_executable(mini-ai-cup-3 ${SOURCE_FILES} ${CHIPMUNK_SOURCES})

target_compile_definitions(mini-ai-cup-3 PRIVATE LOCAL_RUN)