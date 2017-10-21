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
)

set(DMP_RENDERER_SOURCE_FILES
  # source files
  )

add_library(dmp_renderer ${DMP_RENDERER_SOURCE_FILES})
target_link_libraries(dmp_renderer
  Threads::Threads
  ${Qt5Widgets_LIBRARIES}
  ${Qt5OpenGL_LIBRARIES}
  ${DMP_CORE_LIBRARIES}
  stdc++fs
  )

#add_executable(renderer src/renderer.cpp)
#target_link_libraries(renderer dmp_renderer)
