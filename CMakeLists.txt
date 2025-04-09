cmake_minimum_required(VERSION 3.12)
project(KITTI_Vision_Mapping VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(PCL REQUIRED)
find_package(libLAS REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(OpenMP)

# add custom FindXXX modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules")

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/include
)

# IO library: image and pose loading
add_library(io
  src/io/ImageLoader.cpp
  src/io/PoseLoader.cpp
)
target_include_directories(io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)
target_link_libraries(io PUBLIC
  ${OpenCV_LIBS}
)

# Utility functions: math, filesystem, progress bar
add_library(utils
  src/utils/math.cpp
  src/utils/fs.cpp
  src/utils/progress.cpp
)
target_include_directories(utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)

# Core mapping logic
add_library(core
  src/core/Mapper.cpp
)
target_include_directories(core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)
target_link_libraries(core PUBLIC
  io
  utils
  ${PCL_LIBRARIES}
  ${libLAS_LIBRARIES}
)

# Executable target
add_executable(kitti_mapper
  main.cpp
)
target_link_libraries(kitti_mapper PRIVATE
  core
)

# Link OpenMP if available
if (OpenMP_CXX_FOUND)
  target_link_libraries(kitti_mapper PRIVATE OpenMP::OpenMP_CXX)
endif()

install(TARGETS kitti_mapper
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

