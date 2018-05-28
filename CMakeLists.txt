cmake_minimum_required(VERSION 3.8)
project(opengl123)

set(CMAKE_CXX_STANDARD 11)

find_library(OPENGL opengl)
find_library(GLEW glew)
find_library(GLUT glut)

include_directories(/usr/local/include/)
include_directories(./external/glm)
link_libraries(${OPENGL} ${GLUT} ${GLEW})

set(SOURCE_FILES cloth_collision.cpp)

add_executable(opengl123 ${SOURCE_FILES})
