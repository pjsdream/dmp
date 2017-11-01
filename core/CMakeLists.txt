cmake_minimum_required(VERSION 3.1.0)
project(dmp_core)

# C++17
set(CMAKE_CXX_STANDARD 17)

# C++ thread
find_package(Threads)

# fPIC
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# eigen
find_package(Eigen3 REQUIRED)

# cppzmq
find_package(cppzmq REQUIRED)
message(${cppzmq_INCLUDE_DIR})

include_directories(
  include
  lib
  ${EIGEN3_INCLUDE_DIRS}
)

set(DMP_CORE_SOURCE_FILES
  # source files
  include/core/comm/serializer.h
  include/core/comm/deserializer.h
  include/core/comm/subscriber.h
  include/core/comm/publisher.h
  include/core/comm/context.h
  include/core/comm/zmq_serializer.h
  include/core/comm/zmq_deserializer.h
  include/core/utils/timer.h
  include/core/utils/vector_eigen.h
  include/core/json/json.h
  include/core/json/json_loader.h

  src/comm/serializer.cpp
  src/comm/deserializer.cpp
  src/comm/context.cpp
  src/utils/timer.cpp
  src/json/json.cpp
  src/json/json_loader.cpp
  )

add_library(dmp_core ${DMP_CORE_SOURCE_FILES})
target_link_libraries(dmp_core
  Threads::Threads
  stdc++fs
  ${cppzmq_LIBRARY}
  )

add_executable(test_pub src/test_pub.cpp)
target_link_libraries(test_pub dmp_core)

add_executable(test_sub src/test_sub.cpp)
target_link_libraries(test_sub dmp_core)

add_executable(test_serialize src/test_serialize.cpp)
target_link_libraries(test_serialize dmp_core)
