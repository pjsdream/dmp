cmake_minimum_required(VERSION 3.1.0)
project(dmp)

# C++14
set(CMAKE_CXX_STANDARD 14)

# fPIC
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# eigen
find_package(Eigen3 REQUIRED)

# assimp
find_package(assimp 3.3.1 REQUIRED)

# yaml-cpp
find_package(yaml-cpp 0.5.3 REQUIRED)

# Qt5
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Find the Qt library
set(Qt5Widgets_DIR ${QT_PATH})
set(Qt5OpenGL_DIR ${QT_PATH})
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
add_definitions(${Qt5Widgets_DEFINITIONS})
add_definitions(${Qt5OpenGL_DEFINITIONS})

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
)

set(DMP_SOURCE_FILES
  include/dmp/gui/renderer.h
  include/dmp/gui/shader.h
  include/dmp/gui/renderer_impl.h
  include/dmp/gui/gl_base.h
  src/dmp.cpp
  src/gui/renderer.cpp
  src/gui/shader.cpp
  src/gui/renderer_impl.cpp
  src/gui/gl_base.cpp)

add_executable(dmp ${DMP_SOURCE_FILES})
target_link_libraries(dmp
  ${catkin_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${Qt5OpenGL_LIBRARIES}
  )
