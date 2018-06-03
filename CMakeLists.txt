cmake_minimum_required(VERSION 3.8)
project(opengl123)

set(CMAKE_CXX_STANDARD 11)

find_library(OPENGL opengl)
find_library(GLUT glut)

include_directories(/usr/local/include/)
include_directories(./external/glm)
link_libraries(${OPENGL} ${GLUT})

set(SOURCE_FILES
        collision_detect/Matrix.h
        collision_detect/vec.h
        collision_detect/rootparitycollisiontest.h
        collision_detect/rootparitycollisiontest.cpp
#        collision_detect/main.cpp

        cloth_collision.cpp
        BVH.cpp BVH.h
        BVH_Node.h BVH_Node.cpp
        )

add_executable(opengl123 ${SOURCE_FILES})
