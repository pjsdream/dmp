cmake_minimum_required(VERSION 3.1.0)
project(dmp)

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

# yaml-cpp (not using curently)
# find_package(yaml-cpp 0.5.3)

# tinyxml2
set(TINYXML2_SOURCE_FILES lib/tinyxml2/tinyxml2.h lib/tinyxml2/tinyxml2.cpp)

# lodepng
set(LODEPNG_SOURCE_FILES lib/lodepng/lodepng.h lib/lodepng/lodepng.cpp)

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
IF (UNIX AND NOT APPLE)
  set(QT_PATH /playpen/jaesungp/Qt/5.9.1/gcc_64)
  set(Qt5Widgets_DIR ${QT_PATH}/lib/cmake/Qt5Widgets)
  set(Qt5OpenGL_DIR ${QT_PATH}/lib/cmake/Qt5OpenGL)
ENDIF (UNIX AND NOT APPLE)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
add_definitions(${Qt5Widgets_DEFINITIONS})
add_definitions(${Qt5OpenGL_DEFINITIONS})

include_directories(
  include
  lib
  ${EIGEN3_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
  #${YAML_CPP_INCLUDE_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
)

set(DMP_SOURCE_FILES
  # tinyxml2
  ${TINYXML2_SOURCE_FILES}

  # lodepng
  ${LODEPNG_SOURCE_FILES}

  # source files
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
  include/dmp/rendering/resource/resource_manager.h
  src/rendering/resource/resource_manager.cpp
  src/rendering/resource/resource.cpp
  src/rendering/resource/resource_mesh.cpp
  include/dmp/rendering/resource/resource.h
  include/dmp/rendering/resource/resource_mesh.h
  src/rendering/shader/light_shader.cpp
  include/dmp/rendering/shader/light_shader.h
  include/dmp/rendering/scene/scene_edge.h
  src/rendering/scene/scene_edge.cpp
  src/rendering/request/request_manager.cpp
  include/dmp/rendering/request/request_manager.h
  include/dmp/rendering/request/request.h
  src/rendering/request/request.cpp
  include/dmp/json/json.h src/json/json.cpp
  src/planning/planner.cpp include/dmp/planning/planner.h
  include/dmp/robot/robot_model.h
  src/robot/robot_model.cpp
  include/dmp/robot/robot_model_loader.h
  src/robot/robot_model_loader.cpp
  include/dmp/rendering/resource/resource_texture.h
  src/rendering/resource/resource_texture.cpp
  include/dmp/rendering/resource/resource_dynamic_mesh.h
  src/rendering/resource/resource_dynamic_mesh.cpp
  include/dmp/rendering/request/request_frame.h src/rendering/request/request_frame.cpp include/dmp/rendering/request/request_mesh.h src/rendering/request/request_mesh.cpp
  include/dmp/rendering/camera/camera.h src/rendering/camera/camera.cpp include/dmp/rendering/light/light.h include/dmp/rendering/material/material.h src/rendering/light/light.cpp src/rendering/light/light_manager.cpp include/dmp/rendering/light/light_manager.h include/dmp/rendering/request/request_light.h src/rendering/request/request_light.cpp include/dmp/robot/robot_link.h include/dmp/robot/robot_joint.h src/robot/robot_joint.cpp src/robot/robot_link.cpp include/dmp/utils/mesh_loader.h include/dmp/utils/texture_loader.h src/utils/texture_loader.cpp src/utils/mesh_loader.cpp include/dmp/planning/environment/environment.h include/dmp/planning/environment/environment_loader.h include/dmp/shape/shape.h include/dmp/shape/sphere.h include/dmp/shape/cube.h src/shape/shape.cpp src/shape/cube.cpp src/shape/sphere.cpp include/dmp/rendering/request/request_custom_mesh.h src/rendering/request/request_custom_mesh.cpp include/dmp/rendering/request/request_custom_texture.h src/rendering/request/request_custom_texture.cpp src/planning/environment/environment.cpp src/planning/environment/environment_loader.cpp include/dmp/planning/environment/object.h include/dmp/planning/environment/interactable_object.h include/dmp/planning/environment/obstacle.h include/dmp/json/json_loader.h src/json/json_loader.cpp src/planning/environment/obstacle.cpp src/planning/environment/object.cpp include/dmp/shape/cylinder.h src/shape/cylinder.cpp include/dmp/planning/motion/motion_loader.h src/planning/motion/motion_loader.cpp src/planning/motion/motion.cpp include/dmp/planning/motion/motion.h include/dmp/planning/trajectory/trajectory.h include/dmp/planning/trajectory/trajectory_point.h src/planning/trajectory/trajectory.cpp src/planning/trajectory/trajectory_point.cpp include/dmp/planning/planning_option.h src/planning/planning_option.cpp include/dmp/shape/distance_query.h src/shape/distance_query.cpp include/dmp/shape/aabb.h src/shape/aabb.cpp)

add_executable(dmp ${DMP_SOURCE_FILES})
target_link_libraries(dmp
  Threads::Threads
  ${catkin_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  #${YAML_CPP_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${Qt5OpenGL_LIBRARIES}
  )
