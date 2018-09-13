cmake_minimum_required(VERSION 3.12)
project(mini-ai-cup-3)

SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

add_subdirectory(3rdparty/csimplesocket)
add_subdirectory(chipmunk_src)

# Add DEBUG directive
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")
set(CMAKE_CXX_STANDARD 11)
set(SOURCE_FILES solution/main.cpp solution/Solution.cpp solution/Constants.cpp solution/Solver.cpp solution/random/Randomizer.cpp solution/simulation/Simulation.cpp solution/simulation/Car.cpp solution/simulation/Car.h solution/simulation/Map.cpp solution/simulation/Map.h solution/simulation/Deadline.cpp)
#file(GLOB CHIPMUNK_SOURCES chipmunk_src/src/*.c)

add_executable(mini-ai-cup-3 ${SOURCE_FILES})# ${CHIPMUNK_SOURCES})
target_link_libraries(mini-ai-cup-3 csimplesocket chipmunk)
#target_compile_definitions(mini-ai-cup-3 PRIVATE LOCAL_RUN REWIND_VIEWER CALC_TIME_BANK)
target_compile_definitions(mini-ai-cup-3 PRIVATE OPTIMIZATION_RUN CALC_TIME_BANK)
#target_compile_definitions(mini-ai-cup-3 PRIVATE CALC_TIME_BANK)
#target_compile_options(mini-ai-cup-3 PRIVATE -ffast-math)

# python ./miniaicups/madcars/Runners/localrunner.py -f ./mini-ai-cup-3/cmake-build-release/mini-ai-cup-3 -s ./smartguy
# python ./miniaicups/madcars/Runners/localrunner.py -f ./mini-ai-cup-3/cmake-build-release/mini-ai-cup-3 -s ./ver_1_0 -m PillMap,Bus PillCarcassMap,Bus IslandHoleMap,Bus PillHubbleMap,Bus PillHillMap,Bus IslandMap,Bus IslandHoleMap,Bus