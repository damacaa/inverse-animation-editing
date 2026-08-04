cmake_minimum_required(VERSION 3.18)

# Allow FetchContent'd dependencies with old cmake_minimum_required (e.g. Eigen 3.4)
# to configure under CMake >= 4.0.
if(CMAKE_VERSION VERSION_GREATER_EQUAL 4.0)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()

project(InverseAnimationEditing LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# dpe_core is linked into shared libraries (Python module, Unity bridge)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(DPE_BUILD_PYTHON "Build the Python bindings (pybind11 module)" ON)
option(DPE_BUILD_UNITY_BRIDGE "Build the Unity C API bridge (shared library)" ON)
option(DPE_BUILD_TESTS "Build the unit tests (requires GoogleTest)" ON)

# ---------------------------------------------------------------------------
# Dependencies (fetched at configure time, nothing vendored in the repo)
# ---------------------------------------------------------------------------
include(FetchContent)

FetchContent_Declare(Eigen
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
  URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)

FetchContent_Declare(nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
  URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
)

if(DPE_BUILD_PYTHON)
  FetchContent_Declare(pybind11
    URL https://github.com/pybind/pybind11/archive/refs/tags/v3.0.1.tar.gz
    URL_HASH SHA256=741633da746b7c738bb71f1854f957b9da660bcd2dce68d71949037f0969d0ca
  )
endif()

if(DPE_BUILD_PYTHON)
  FetchContent_MakeAvailable(Eigen nlohmann_json pybind11)
else()
  FetchContent_MakeAvailable(Eigen nlohmann_json)
endif()

# ---------------------------------------------------------------------------
# Core physics engine (engine-independent static library)
# ---------------------------------------------------------------------------
add_library(dpe_core STATIC
  engine/Collider.cpp
  engine/DebugHelper.cpp
  engine/Face.cpp
  engine/Fixer.cpp
  engine/Node.cpp
  engine/Object.cpp
  engine/PhysicsManager.cpp
  engine/Spring.cpp
)

target_include_directories(dpe_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/engine)
target_link_libraries(dpe_core PUBLIC Eigen3::Eigen nlohmann_json::nlohmann_json)

# ---------------------------------------------------------------------------
# Python bindings: importable as "import DifferentiablePhysicsEngine"
# ---------------------------------------------------------------------------
if(DPE_BUILD_PYTHON)
  pybind11_add_module(DifferentiablePhysicsEngine bindings/python/PythonInterface.cpp)
  target_link_libraries(DifferentiablePhysicsEngine PRIVATE dpe_core)
  set_target_properties(DifferentiablePhysicsEngine PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python)
endif()

# ---------------------------------------------------------------------------
# Unity bridge: shared library exposing the C API used via P/Invoke
# ---------------------------------------------------------------------------
if(DPE_BUILD_UNITY_BRIDGE)
  add_library(dpe_unity SHARED bindings/unity/unity_bridge.cpp)
  target_link_libraries(dpe_unity PRIVATE dpe_core)
  # Unity resolves DllImport("DifferentiablePhysicsEngine") to
  # DifferentiablePhysicsEngine.dll / libDifferentiablePhysicsEngine.so
  set_target_properties(dpe_unity PROPERTIES
    OUTPUT_NAME DifferentiablePhysicsEngine
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/unity
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/unity)

  # Deploy next to the Unity project so the editor picks it up
  add_custom_command(TARGET dpe_unity POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
      $<TARGET_FILE:dpe_unity>
      ${CMAKE_SOURCE_DIR}/UnityProject/Assets/$<TARGET_FILE_NAME:dpe_unity>)
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(DPE_BUILD_TESTS)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    URL_HASH SHA256=7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  enable_testing()

  add_executable(dpe_tests test/test_physics.cpp)
  target_link_libraries(dpe_tests PRIVATE dpe_core gtest_main)
  include(GoogleTest)
  gtest_discover_tests(dpe_tests)
endif()

# ---------------------------------------------------------------------------
# Viewer (Vulkan + GLFW + ImGui standalone app)
# ---------------------------------------------------------------------------
include(${CMAKE_CURRENT_SOURCE_DIR}/viewer/CMakeLists.txt)
