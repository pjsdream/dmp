cmake_minimum_required(VERSION 3.1.0)
project(dmp_renderer)

# C++17
set(CMAKE_CXX_STANDARD 17)

# C++ thread
find_package(Threads)

# fPIC
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# eigen
find_package(Eigen3 REQUIRED)

# assimp
find_package(assimp REQUIRED)

# lodepng
set(LODEPNG_SOURCE_FILES lib/lodepng/lodepng.h lib/lodepng/lodepng.cpp)

# Qt5
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Find the Qt library
IF (APPLE)
  set(QT_PATH ~/Qt/5.9.2/clang_64)
  set(Qt5Widgets_DIR ${QT_PATH}/lib/cmake/Qt5Widgets)
  set(Qt5OpenGL_DIR ${QT_PATH}/lib/cmake/Qt5OpenGL)
ENDIF (APPLE)
IF (UNIX AND NOT APPLE)
  set(QT_PATH /playpen/jaesungp/Qt/5.9.2/gcc_64)
  set(Qt5Widgets_DIR ${QT_PATH}/lib/cmake/Qt5Widgets)
  set(Qt5OpenGL_DIR ${QT_PATH}/lib/cmake/Qt5OpenGL)
ENDIF (UNIX AND NOT APPLE)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
add_definitions(${Qt5Widgets_DEFINITIONS})
add_definitions(${Qt5OpenGL_DEFINITIONS})

# DMP core
link_directories(${CMAKE_BINARY_DIR}/core)
set(DMP_CORE_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/core/include)
set(DMP_CORE_LIBRARIES dmp_core)

include_directories(
  include
  lib
  ${EIGEN3_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
  ${DMP_CORE_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
)

set(DMP_RENDERER_SOURCE_FILES
  # lodepng
  ${LODEPNG_SOURCE_FILES}

  include/renderer/gl_base.h
  include/renderer/camera/camera.h
  include/renderer/renderer_window.h
  include/renderer/shader/shader.h
  include/renderer/shader/light_shader.h
  include/renderer/scene/scene_manager.h
  include/renderer/scene/scene_node.h
  include/renderer/light/light_manager.h
  include/renderer/resource/resource.h
  include/renderer/resource/resource_manager.h
  include/renderer/resource/resource_mesh.h
  include/renderer/resource/resource_texture.h
  include/renderer/resource/mesh_loader.h
  include/renderer/resource/texture_loader.h

  src/camera/camera.cpp
  src/renderer_window.cpp
  src/shader/shader.cpp
  src/shader/light_shader.cpp
  src/scene/scene_manager.cpp
  src/scene/scene_node.cpp
  src/light/light_manager.cpp
  src/resource/resource.cpp
  src/resource/resource_manager.cpp
  src/resource/resource_mesh.cpp
  src/resource/resource_texture.cpp
  src/resource/mesh_loader.cpp
  src/resource/texture_loader.cpp

  src/main.cpp include/renderer/request/request_subscriber.h include/renderer/resource/raw_mesh.h)

add_executable(renderer ${DMP_RENDERER_SOURCE_FILES})
target_link_libraries(renderer
  Threads::Threads
  ${Qt5Widgets_LIBRARIES}
  ${Qt5OpenGL_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  ${DMP_CORE_LIBRARIES}
  stdc++fs
  )

add_executable(test_renderer
  src/test_renderer.cpp

  # Requests
  include/renderer/request/request.h
  include/renderer/request/request_mesh.h
  include/renderer/renderer_ostream.h

  src/renderer_ostream.cpp
  )

target_link_libraries(test_renderer
  ${DMP_CORE_LIBRARIES}
  )
