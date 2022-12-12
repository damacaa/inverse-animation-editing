cmake_minimum_required(VERSION 3.6)

set (CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project(UnityDLL)

#sources
file(GLOB SOURCE 
        "UnityDLL/*.cpp"
)

file(GLOB SOURCES
    ${SOURCE}
)

#headers
file(GLOB HEADERS 
        "UnityDLL/*.h"
)

file(GLOB HEADERS
    ${HEADERS}
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/UnityDLL"
         PREFIX "UnityDLL"
         FILES ${SOURCES} ${HEADERS})

add_subdirectory(pybind11)
pybind11_add_module(UnityDLL ${SOURCES} ${HEADERS} "${CMAKE_CURRENT_SOURCE_DIR}/UnityDLL/PythonInterface.cpp")

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.10.5/json.tar.xz)
FetchContent_MakeAvailable(json)

target_link_libraries(UnityDLL PUBLIC nlohmann_json::nlohmann_json)

# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
target_compile_definitions(UnityDLL
                           PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})

# 

set(PYTHON_INCLUDE "C:/Users/Dani/AppData/Local/Programs/Python/Python310/include" CACHE PATH "") 
set(PYTHON_LIB "C:/Users/Dani/AppData/Local/Programs/Python/Python310/libs/python310.lib" CACHE FILEPATH "")
set(PYBIND11_INCLUDE "D:/Projects/MassSpringSimulator/UnityDLL/include/pybind11/include" CACHE PATH "") 

target_include_directories(
        UnityDLL
        PUBLIC
        UnityDLL/
        libs/
        libs/Eigen/
)

target_include_directories(
        UnityDLL 
        PUBLIC
        ${PYTHON_INCLUDE}
        ${PYBIND11_INCLUDE}
)

target_link_libraries(UnityDLL PUBLIC
        ${PYTHON_LIB}
)

#project(UnityDLL VERSION 1.0.1 DESCRIPTION "mylib description")
#add_library(UnityDLL SHARED ${CMAKE_CURRENT_SOURCE_DIR}/UnityDLL/dllmain.cpp)

#python -m pip install "d:/Projects/MassSpringSimulator/UnityDLL"