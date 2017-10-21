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
  include/dmp/rendering/request/request_custom_mesh.h
  include/dmp/rendering/request/request_custom_texture.h
  include/dmp/rendering/gl_base.h
  include/dmp/rendering/scene/scene_manager.h
  include/dmp/rendering/scene/scene_node.h
  include/dmp/rendering/resource/resource_manager.h
  include/dmp/rendering/resource/resource.h
  include/dmp/rendering/resource/resource_mesh.h
  include/dmp/rendering/shader/light_shader.h
  include/dmp/rendering/scene/scene_edge.h
  include/dmp/rendering/request/request.h
  include/dmp/rendering/resource/resource_texture.h
  include/dmp/rendering/resource/resource_dynamic_mesh.h
  include/dmp/rendering/request/request_frame.h
  include/dmp/rendering/request/request_mesh.h
  include/dmp/rendering/camera/camera.h
  include/dmp/rendering/light/light.h
  include/dmp/rendering/material/material.h
  include/dmp/rendering/light/light_manager.h
  include/dmp/rendering/request/request_light.h
  include/dmp/shape/shape.h
  include/dmp/shape/sphere.h
  include/dmp/shape/cube.h
  include/dmp/shape/distance_query.h
  include/dmp/shape/aabb.h
  include/dmp/shape/bounding_volume_factory.h
  include/dmp/shape/cylinder.h
  include/dmp/planning/environment/object.h
  include/dmp/planning/environment/interactable_object.h
  include/dmp/planning/environment/obstacle.h
  include/dmp/planning/environment/environment.h
  include/dmp/planning/environment/environment_loader.h
  include/dmp/planning/cost/cost.h
  include/dmp/planning/cost/cost_smoothness.h
  include/dmp/planning/cost/cost_collision.h
  include/dmp/planning/objective/objective.h
  include/dmp/planning/objective/objective_sequence.h
  include/dmp/planning/objective/objective_reach_to_grip.h
  include/dmp/planning/objective/objective_grip.h
  include/dmp/planning/objective/objective_reach_to_place.h
  include/dmp/planning/motion/motion.h
  include/dmp/planning/planning_option.h
  include/dmp/planning/motion/motion_loader.h
  include/dmp/planning/planner.h
  include/dmp/json/json_loader.h
  include/dmp/json/json.h
  include/dmp/robot/tree_robot_model.h
  include/dmp/robot/robot_model_loader.h
  include/dmp/utils/mesh_loader.h
  include/dmp/utils/texture_loader.h
  include/dmp/robot/tree_robot_link.h
  include/dmp/robot/tree_robot_joint.h
  include/dmp/trajectory/trajectory.h
  include/dmp/trajectory/trajectory_point.h
  include/dmp/robot/robot_model.h
  include/dmp/utils/vector_eigen.h
  include/dmp/robot/robot_joint.h
  include/dmp/robot/robot_link.h
  include/dmp/comm/node.h
  include/dmp/comm/publisher.h
  include/dmp/comm/subscriber.h
  include/dmp/comm/message_queue.h
  include/dmp/controlling/controller.h
  include/dmp/controlling/controller_option.h
  include/dmp/utils/rate.h
  include/dmp/robot/robot_state.h
  include/dmp/common.h
  include/dmp/trajectory/cubic_spline_trajectory.h
  include/dmp/trajectory/cubic_spline.h
  include/dmp/utils/timer.h
  include/dmp/utils/stopwatch.h
  include/dmp/robot/robot_configuration.h
  include/dmp/comm/manager.h
  include/dmp/robot/planning_robot_model.h
  include/dmp/robot/planning_robot_joint.h
  include/dmp/robot/planning_robot_link.h
  include/dmp/comm/serializer.h
  include/dmp/comm/deserializer.h
  src/rendering/renderer.cpp
  src/rendering/shader/shader.cpp
  src/rendering/gl_base.cpp
  src/rendering/scene/scene_manager.cpp
  src/rendering/scene/scene_node.cpp
  src/rendering/resource/resource_manager.cpp
  src/rendering/resource/resource.cpp
  src/rendering/resource/resource_mesh.cpp
  src/rendering/shader/light_shader.cpp
  src/rendering/scene/scene_edge.cpp
  src/rendering/request/request.cpp
  src/rendering/resource/resource_texture.cpp
  src/rendering/resource/resource_dynamic_mesh.cpp
  src/rendering/request/request_frame.cpp
  src/rendering/request/request_mesh.cpp
  src/rendering/camera/camera.cpp
  src/rendering/light/light.cpp
  src/rendering/light/light_manager.cpp
  src/rendering/request/request_light.cpp
  src/rendering/request/request_custom_mesh.cpp
  src/rendering/request/request_custom_texture.cpp
  src/shape/distance_query.cpp
  src/shape/aabb.cpp
  src/shape/bounding_volume_factory.cpp
  src/shape/shape.cpp
  src/shape/cube.cpp
  src/shape/sphere.cpp
  src/shape/cylinder.cpp
  src/json/json.cpp
  src/robot/tree_robot_model.cpp
  src/robot/robot_model_loader.cpp
  src/robot/tree_robot_joint.cpp
  src/robot/tree_robot_link.cpp
  src/utils/texture_loader.cpp
  src/utils/mesh_loader.cpp
  src/json/json_loader.cpp
  src/planning/planner.cpp
  src/planning/environment/environment.cpp
  src/planning/environment/environment_loader.cpp
  src/planning/environment/obstacle.cpp
  src/planning/environment/object.cpp
  src/planning/motion/motion_loader.cpp
  src/planning/motion/motion.cpp
  src/planning/planning_option.cpp
  src/planning/objective/objective.cpp
  src/planning/objective/objective_sequence.cpp
  src/planning/objective/objective_reach_to_grip.cpp
  src/planning/objective/objective_grip.cpp
  src/planning/objective/objective_reach_to_place.cpp
  src/planning/cost/cost.cpp
  src/planning/cost/cost_smoothness.cpp
  src/planning/cost/cost_collision.cpp
  src/planning/environment/interactable_object.cpp
  src/trajectory/trajectory.cpp
  src/trajectory/trajectory_point.cpp
  src/trajectory/cubic_spline.cpp
  src/trajectory/cubic_spline_trajectory.cpp
  src/robot/robot_model.cpp
  src/robot/robot_joint.cpp
  src/robot/robot_link.cpp
  src/controlling/controller.cpp
  src/controlling/controller_option.cpp
  src/utils/rate.cpp
  src/robot/robot_state.cpp
  src/utils/timer.cpp
  src/utils/stopwatch.cpp
  src/robot/robot_configuration.cpp
  src/robot/planning_robot_link.cpp
  src/robot/planning_robot_joint.cpp
  src/robot/planning_robot_model.cpp
  src/comm/manager.cpp
  src/comm/node.cpp
  src/comm/serializer.cpp
  src/comm/deserializer.cpp
)

add_library(dmp ${DMP_SOURCE_FILES})
target_link_libraries(dmp
  Threads::Threads
  ${catkin_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  #${YAML_CPP_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${Qt5OpenGL_LIBRARIES}
  stdc++fs
  )

add_executable(test_dmp src/test/test_dmp.cpp)
target_link_libraries(test_dmp dmp)

add_executable(test_comm src/test/test_comm.cpp)
target_link_libraries(test_comm dmp)

add_executable(test_serialize src/test/test_serialize.cpp)
target_link_libraries(test_serialize dmp)
