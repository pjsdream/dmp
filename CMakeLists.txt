cmake_minimum_required(VERSION 3.1.0)
project(dmp)

set(CMAKE_BUILD_TYPE Debug)

# C++17
set(CMAKE_CXX_STANDARD 17)

# C++ thread
find_package(Threads)

# fPIC
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# eigen
find_package(Eigen3)

# assimp
find_package(assimp)

# yaml-cpp
find_package(yaml-cpp 0.5.3)

# Qt5
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Find the Qt library
IF (APPLE)
  set(QT_PATH ~/Qt/5.9.1/clang_64)
  set(Qt5Widgets_DIR ${QT_PATH}/lib/cmake/Qt5Widgets)
  set(Qt5OpenGL_DIR ${QT_PATH}/lib/cmake/Qt5OpenGL)
ENDIF (APPLE)
IF (LINUX)
  set(Qt5Widgets_DIR ${QT_PATH})
  set(Qt5OpenGL_DIR ${QT_PATH})
ENDIF (LINUX)
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
  include/dmp/rendering/gl_base.h
  src/dmp.cpp
  src/rendering/renderer.cpp
  src/rendering/shader/shader.cpp
  src/rendering/gl_base.cpp
  include/dmp/rendering/scene/scene_manager.h
  src/rendering/scene/scene_manager.cpp
  src/rendering/scene/scene_node.cpp
  include/dmp/rendering/scene/scene_node.h
  include/dmp/rendering/scene/scene_mesh_object.h
  include/dmp/rendering/scene/scene_object.h
  src/rendering/scene/scene_mesh_object.cpp
  src/rendering/scene/scene_object.cpp
  include/dmp/rendering/resource/resource_manager.h
  src/rendering/resource/resource_manager.cpp
  src/rendering/resource/resource.cpp
  src/rendering/resource/resource_mesh.cpp
  include/dmp/rendering/resource/resource.h
  include/dmp/rendering/resource/resource_mesh.h
  src/rendering/shader/light_shader.cpp include/dmp/rendering/shader/light_shader.h include/dmp/rendering/scene/scene_edge.h src/rendering/scene/scene_edge.cpp src/rendering/request/request_manager.cpp include/dmp/rendering/request/request_manager.h include/dmp/rendering/request/request.h src/rendering/request/request.cpp include/dmp/json/json.h src/json/json.cpp include/dmp/rendering/tf/tf_manager.h include/dmp/rendering/tf/tf.h src/rendering/tf/tf_manager.cpp src/rendering/tf/tf.cpp)

add_executable(dmp ${DMP_SOURCE_FILES})
target_link_libraries(dmp
  Threads::Threads
  ${catkin_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${Qt5OpenGL_LIBRARIES}
  )
