cmake_minimum_required(VERSION 3.5)

#add_library(chipmunk INTERFACE)
##target_link_libraries(chipmunk INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/bin/libchipmunk.so)
#target_link_libraries(chipmunk INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/bin/chipmunk.dll)
#target_include_directories(chipmunk INTERFACE include)


project(chipmunk LANGUAGES C)

file(GLOB chimpunk_sources "src/*.c")

add_library(chipmunk SHARED ${chimpunk_sources})
set_property(TARGET chipmunk PROPERTY C_STANDARD 99)
target_compile_options(chipmunk PRIVATE -ffast-math)
target_include_directories(chipmunk PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)