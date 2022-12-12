cmake_minimum_required(VERSION 3.6)

set (CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(DifferentiablePhysicsEngine)

#sources
file(GLOB SOURCE 
        "DifferentiablePhysicsEngine/*.cpp"
)

file(GLOB SOURCES
    ${SOURCE}
)

#headers
file(GLOB HEADERS 
        "DifferentiablePhysicsEngine/*.h"
)

file(GLOB HEADERS
    ${HEADERS}
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/DifferentiablePhysicsEngine"
         PREFIX "DifferentiablePhysicsEngine"
         FILES ${SOURCES} ${HEADERS})

add_subdirectory(pybind11)
pybind11_add_module(DifferentiablePhysicsEngine ${SOURCES} ${HEADERS} "${CMAKE_CURRENT_SOURCE_DIR}/DifferentiablePhysicsEngine/PythonInterface.cpp")

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.10.5/json.tar.xz)
FetchContent_MakeAvailable(json)

target_link_libraries(DifferentiablePhysicsEngine PUBLIC nlohmann_json::nlohmann_json)

# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
target_compile_definitions(DifferentiablePhysicsEngine
                           PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})

# 

set(PYTHON_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/Python310/include" CACHE PATH "") 
set(PYTHON_LIB "${CMAKE_CURRENT_SOURCE_DIR}/Python310/libs/python310.lib" CACHE FILEPATH "")
set(PYBIND11_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/pybind11/include/pybind11" CACHE PATH "") 

target_include_directories(
        DifferentiablePhysicsEngine
        PUBLIC
        DifferentiablePhysicsEngine/
        libs/
        libs/Eigen/
)

target_include_directories(
        DifferentiablePhysicsEngine 
        PUBLIC
        ${PYTHON_INCLUDE}
        ${PYBIND11_INCLUDE}
)

target_link_libraries(DifferentiablePhysicsEngine PUBLIC
        ${PYTHON_LIB}
)

#project(DifferentiablePhysicsEngine VERSION 1.0.1 DESCRIPTION "mylib description")
#add_library(DifferentiablePhysicsEngine SHARED ${CMAKE_CURRENT_SOURCE_DIR}/DifferentiablePhysicsEngine/dllmain.cpp)

#python -m pip install "../DifferentiablePhysicsEngine"