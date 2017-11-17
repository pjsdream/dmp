cmake_minimum_required(VERSION 3.1.0)
project(dmp_hmp)

# C++17
set(CMAKE_CXX_STANDARD 17)

# C++ thread
find_package(Threads)

# fPIC
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# eigen
find_package(Eigen3 REQUIRED)

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
  ${DMP_CORE_INCLUDE_DIRS}
  ${DMP_RENDERER_INCLUDE_DIRS}
)

set(DMP_PCD_SOURCE_FILES
  # source files
  )

add_library(dmp_hmp ${DMP_HMP_SOURCE_FILES})
target_link_libraries(dmp_hmp
  Threads::Threads
  ${DMP_CORE_LIBRARIES}
  stdc++fs
  )

add_executable(test_hmp src/test_hmp.cpp)
target_link_libraries(test_hmp
  dmp_hmp
  )
