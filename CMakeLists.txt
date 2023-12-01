#Template for INFOH502
#Heavily inspired from https://github.com/Polytonic/Glitter/blob/master/CMakeLists.txt

#project setup
project(infoh502-cpp)

#set the minimal version of cmake and the minimal version of the c++ language
cmake_minimum_required(VERSION 3.20 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 14)     
set(CMAKE_VERBOSE_MAKEFILE ON)


find_package(OpenGL REQUIRED)

#for glad library
add_library( glad STATIC 3rdParty/glad/src/glad.c)
set(GLAD_INCLUDE "3rdParty/glad/include")
target_include_directories(glad PUBLIC ${GLAD_INCLUDE})

#for GLFW
option(GLFW_BUILD_DOCS OFF)
option(GLFW_BUILD_EXAMPLES OFF)
option(GLFW_BUILD_TESTS OFF)

#Search for glfw in case it is installed
find_package(glfw3 QUIET)
if (NOT glfw3_FOUND)
    #if glfw is not found, we will use the sources from the submodules
    add_subdirectory(3rdParty/glfw)
    include_directories(3rdParty/glfw/include/)
endif()


find_package(Bullet)
if(NOT Bullet_FOUND)
    option(BUILD_BULLET2_DEMOS OFF)
    option(BUILD_CPU_DEMOS OFF)
    option(BUILD_EXTRAS OFF)
    option(BUILD_OPENGL3_DEMOS OFF)
    option(BUILD_UNIT_TESTS OFF)
    add_subdirectory(3rdParty/bullet)
    include_directories(3rdParty/bullet/src)
else()
    include_directories(${BULLET_INCLUDE_DIR}) 
endif()


include_directories(3rdParty/glad/include/
                    3rdParty/glm/
                    3rdParty/stb/
                    3rdParty/bullet/)




set(SOURCES_EX_08 "ex08.cpp" "camera.h" "shader.h" "object.h")
set(SOURCES_Main "main.cpp" "camera.h" "shader.h" "object.h")


#These commands are there to specify the path to the folder containing the object and textures files as macro
#With these you can just use PATH_TO_OBJECTS and PATH_TO_TEXTURE in your c++ code and the compiler will replace it by the correct expression
add_compile_definitions(PATH_TO_OBJECTS="${CMAKE_CURRENT_SOURCE_DIR}/objects")
add_compile_definitions(PATH_TO_TEXTURE="${CMAKE_CURRENT_SOURCE_DIR}/textures")

add_executable(${PROJECT_NAME}_ex08 ${SOURCES_EX_08})
target_link_libraries(${PROJECT_NAME}_ex08 PUBLIC OpenGL::GL glfw glad)


add_executable(${PROJECT_NAME}_Main ${SOURCES_Main})
target_link_libraries(${PROJECT_NAME}_Main PUBLIC glad OpenGL::GL glfw LinearMath  BulletDynamics BulletCollision)
