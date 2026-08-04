option(DPE_BUILD_VIEWER "Build the standalone Vulkan visualization app" ON)

if(DPE_BUILD_VIEWER)
  find_package(Vulkan REQUIRED)
  # GLFW — windowing + input
  FetchContent_Declare(glfw
    URL https://github.com/glfw/glfw/archive/refs/tags/3.4.tar.gz
    URL_HASH SHA256=c038d34200234d071fae9345bc455e4a8f2f544ab60150765d7704e08f3dac01)
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_WAYLAND ON CACHE BOOL "" FORCE)
  set(GLFW_BUILD_X11 OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(glfw)

  # Dear ImGui — parameter tuning panel
  FetchContent_Declare(imgui
    URL https://github.com/ocornut/imgui/archive/refs/tags/v1.91.9.tar.gz
    URL_HASH SHA256=3872a5f90df78fced023c1945f4466b654fd74573370b77b17742149763a7a7c)
  # ImGui is a single-header library without CMake support natively;
  # we handle it with a simple interface target.
  FetchContent_GetProperties(imgui)
  if(NOT imgui_POPULATED)
    FetchContent_Populate(imgui)
    add_library(imgui INTERFACE)
    target_include_directories(imgui INTERFACE ${imgui_SOURCE_DIR})
    target_sources(imgui INTERFACE
      ${imgui_SOURCE_DIR}/imgui.cpp
      ${imgui_SOURCE_DIR}/imgui_draw.cpp
      ${imgui_SOURCE_DIR}/imgui_tables.cpp
      ${imgui_SOURCE_DIR}/imgui_widgets.cpp
      ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
      ${imgui_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp
    )
  endif()

  # Compile GLSL shaders to SPIR-V at build time
  find_program(GLSLC glslc REQUIRED)
  set(SHADER_DIR ${CMAKE_SOURCE_DIR}/viewer/shaders)
  function(compile_shader TARGET SRC DST)
    add_custom_command(
      OUTPUT ${DST}
      COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/shaders
      COMMAND ${GLSLC} -fshader-stage=${TARGET} ${SRC} -o ${DST}
      DEPENDS ${SRC}
      COMMENT "Compiling ${SRC} → ${DST}")
  endfunction()

  compile_shader(vert ${SHADER_DIR}/mesh.vert ${CMAKE_BINARY_DIR}/shaders/mesh.vert.spv)
  compile_shader(frag ${SHADER_DIR}/mesh.frag ${CMAKE_BINARY_DIR}/shaders/mesh.frag.spv)

  # Viewer executable
  add_executable(dpe_viewer
    viewer/main.cpp
    viewer/renderer.cpp
    viewer/scene_loader.cpp
    viewer/sim_worker.cpp
    ${CMAKE_BINARY_DIR}/shaders/mesh.vert.spv
    ${CMAKE_BINARY_DIR}/shaders/mesh.frag.spv
  )
  target_link_libraries(dpe_viewer PRIVATE dpe_core glfw imgui Vulkan::Vulkan)
  target_include_directories(dpe_viewer PRIVATE viewer)
  target_compile_definitions(dpe_viewer PRIVATE
    SHADER_DIR="${CMAKE_BINARY_DIR}/shaders")
  source_group("Shaders" FILES
    ${CMAKE_BINARY_DIR}/shaders/mesh.vert.spv
    ${CMAKE_BINARY_DIR}/shaders/mesh.frag.spv)
endif()
