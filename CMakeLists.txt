cmake_minimum_required(VERSION 3.12)
project(mini-ai-cup-3)

add_subdirectory(csimplesocket)

# Add DEBUG directive
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")
set(CMAKE_CXX_STANDARD 11)
set(SOURCE_FILES main.cpp Solution.cpp Constants.cpp Solver.cpp random/Randomizer.cpp simulation/Simulation.cpp simulation/Car.cpp simulation/Car.h simulation/Map.cpp simulation/Map.h simulation/Deadline.cpp)
file(GLOB CHIPMUNK_SOURCES chipmunk/src/*.c)

add_executable(mini-ai-cup-3 ${SOURCE_FILES} ${CHIPMUNK_SOURCES})
target_link_libraries(mini-ai-cup-3 csimplesocket)
target_compile_definitions(mini-ai-cup-3 PRIVATE LOCAL_RUN REWIND_VIEWER)
#target_compile_options(mini-ai-cup-3 PRIVATE -ffast-math)