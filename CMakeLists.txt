cmake_minimum_required(VERSION 3.1.0)
project(dmp)

# C++14
set(CMAKE_CXX_STANDARD 14)

# fPIC
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# eigen
find_package(Eigen3)

# assimp
find_package(assimp 3.3.1)

# yaml-cpp
find_package(yaml-cpp 0.5.3)

# Qt5
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Find the Qt library
set(QT_PATH /Users/jaesungp/Qt/5.9.1/ios)
set(Qt5Widgets_DIR ${QT_PATH}/lib/cmake/Qt5Widgets)
set(Qt5OpenGL_DIR ${QT_PATH}/lib/cmake/Qt5OpenGL)
message(${Qt5Widgets_DIR})
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
  include/dmp/rendering/renderer.h
  include/dmp/rendering/shader/shader.h
  include/dmp/rendering/renderer_impl.h
  include/dmp/rendering/gl_base.h
  src/dmp.cpp
  src/rendering/renderer.cpp
  src/rendering/shader/shader.cpp
  src/rendering/renderer_impl.cpp
  src/rendering/gl_base.cpp include/dmp/rendering/scene/scene_manager.h src/rendering/scene/scene_manager.cpp src/rendering/scene/scene_node.cpp include/dmp/rendering/scene/scene_node.h)

add_executable(dmp ${DMP_SOURCE_FILES})
target_link_libraries(dmp
  ${catkin_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${Qt5OpenGL_LIBRARIES}
  )
