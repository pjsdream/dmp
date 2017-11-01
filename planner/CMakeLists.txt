cmake_minimum_required(VERSION 3.1.0)
project(dmp_planner)

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

# tinyxml2
set(TINYXML2_SOURCE_FILES lib/tinyxml2/tinyxml2.h lib/tinyxml2/tinyxml2.cpp)

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

# DMP renderer
set(DMP_RENDERER_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/renderer/include)

include_directories(
  include
  lib
  ${EIGEN3_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
  ${DMP_CORE_INCLUDE_DIRS}
  ${DMP_RENDERER_INCLUDE_DIRS}
)

set(DMP_PLANNER_SOURCE_FILES
  # tinyxml2
  ${TINYXML2_SOURCE_FILES}

  # source files
  include/planner/robot/planning_robot_joint.h
  include/planner/robot/planning_robot_link.h
  include/planner/robot/planning_robot_model.h
  include/planner/robot/robot_configuration.h
  include/planner/robot/robot_state.h
  include/planner/robot/robot_joint.h
  include/planner/robot/robot_link.h
  include/planner/robot/robot_model.h
  include/planner/robot/robot_model_loader.h
  include/planner/robot/tree_robot_joint.h
  include/planner/robot/tree_robot_link.h
  include/planner/robot/tree_robot_model.h
  include/planner/shape/aabb.h
  include/planner/shape/bounding_volume_factory.h
  include/planner/shape/cube.h
  include/planner/shape/cylinder.h
  include/planner/shape/distance_query.h
  include/planner/shape/shape.h
  include/planner/shape/sphere.h
  include/planner/motion/motion.h
  include/planner/motion/motion_loader.h

  # TODO: make raw_mesh struct common
  ../renderer/include/renderer/resource/raw_mesh.h

  src/robot/planning_robot_joint.cpp
  src/robot/planning_robot_link.cpp
  src/robot/planning_robot_model.cpp
  src/robot/robot_configuration.cpp
  src/robot/robot_state.cpp
  src/robot/robot_joint.cpp
  src/robot/robot_link.cpp
  src/robot/robot_model.cpp
  src/robot/robot_model_loader.cpp
  src/robot/tree_robot_joint.cpp
  src/robot/tree_robot_link.cpp
  src/robot/tree_robot_model.cpp
  src/shape/aabb.cpp
  src/shape/bounding_volume_factory.cpp
  src/shape/cube.cpp
  src/shape/cylinder.cpp
  src/shape/distance_query.cpp
  src/shape/shape.cpp
  src/shape/sphere.cpp
  src/motion/motion.cpp
  src/motion/motion_loader.cpp
)

add_library(dmp_planner ${DMP_PLANNER_SOURCE_FILES})
target_link_libraries(dmp_planner
  Threads::Threads
  ${ASSIMP_LIBRARIES}
  ${DMP_CORE_LIBRARIES}
  stdc++fs
  )

add_executable(planner src/planner.cpp)
target_link_libraries(planner
  dmp_planner
  ${Qt5Widgets_LIBRARIES}
  )
