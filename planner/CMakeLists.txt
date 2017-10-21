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

# lodepng
set(LODEPNG_SOURCE_FILES lib/lodepng/lodepng.h lib/lodepng/lodepng.cpp)

# DMP core
link_directories(${CMAKE_BINARY_DIR}/core)
set(DMP_CORE_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/core/include)
set(DMP_CORE_LIBRARIES dmp_core)

include_directories(
  include
  lib
  ${EIGEN3_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
)

set(DMP_PLANNER_SOURCE_FILES
  # tinyxml2
  ${TINYXML2_SOURCE_FILES}

  # lodepng
  ${LODEPNG_SOURCE_FILES}

  # source files
)

add_library(dmp_planner ${DMP_PLANNER_SOURCE_FILES})
target_link_libraries(dmp_planner
  Threads::Threads
  ${ASSIMP_LIBRARIES}
  ${DMP_CORE_LIBRARIES}
  stdc++fs
  )

#add_executable(planner src/planner.cpp)
#target_link_libraries(planner dmp_planner)
