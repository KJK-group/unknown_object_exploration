cmake_minimum_required(VERSION 3.14)
project(
  kdtree
  VERSION 0.1
  DESCRIPTION ""
  LANGUAGES CXX)

# include_directories(include) add_library(kdtree STATIC
# ${CMAKE_CURRENT_SOURCE_DIR}/src/kdtree.cpp) add_library(kdtree STATIC
# src/kdtree.cpp)

# target_include_directories(kdtree PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# target_compile_options(kdtree PUBLIC -std=c++17 -O3 -Wall)

set(KDTREE_LIB kdtree-header-only)

add_library(${KDTREE_LIB} INTERFACE)
target_sources(${KDTREE_LIB} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_include_directories(${KDTREE_LIB}
                           INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include/)

# set(KDTREE3_LIB kdtree-header-only)

# add_library(${KDTREE3_LIB} INTERFACE) target_sources(${KDTREE3_LIB} INTERFACE
# ${CMAKE_CURRENT_SOURCE_DIR}/include/)
# target_include_directories(${KDTREE3_LIB} INTERFACE
# ${CMAKE_CURRENT_SOURCE_DIR}/include/)
