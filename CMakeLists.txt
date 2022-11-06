cmake_minimum_required(VERSION 3.5)

project(phase_manager LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include_directories(
    include/
    src/
    ${EIGEN3_INCLUDE_DIR}
)

set(LIBRARY_TARGET_NAME phase_manager) # set library name

add_library(${LIBRARY_TARGET_NAME} SHARED
  src/phase_manager.cpp
)

target_include_directories(${LIBRARY_TARGET_NAME}
    INTERFACE
    ${EIGEN3_INCLUDE_DIR})

add_executable(letstry
    src/letstry.cpp)

target_link_libraries(letstry ${LIBRARY_TARGET_NAME})
