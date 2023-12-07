cmake_minimum_required(VERSION 3.5)

project(phase_manager LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

if(DEFINED ENV{CONDA_PREFIX})

  set(CMAKE_INSTALL_PREFIX $ENV{CONDA_PREFIX}/ CACHE PATH "bindings install prefix" FORCE)

endif()

find_package(Eigen3 3.3 REQUIRED)

#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include_directories(
    include/
    src/
    ${EIGEN3_INCLUDE_DIR}
)

set(LIBRARY_TARGET_NAME phase_manager) # set library name

add_library(${LIBRARY_TARGET_NAME} SHARED
  src/phase_manager.cpp
  src/phase.cpp
)

target_include_directories(${LIBRARY_TARGET_NAME}
    INTERFACE
    ${EIGEN3_INCLUDE_DIR})

include(GNUInstallDirs)
install(TARGETS  ${LIBRARY_TARGET_NAME}
        EXPORT   ${LIBRARY_TARGET_NAME}
        LIBRARY  DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE  DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        RUNTIME  DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)

#include(InstallBasicPackageFiles)
#install_basic_package_files(${LIBRARY_TARGET_NAME}
#                            VERSION ${${PROJECT_NAME}_VERSION}
#                            COMPATIBILITY AnyNewerVersion
#                            EXPORT ${LIBRARY_TARGET_NAME}
#                            VARS_PREFIX ${LIBRARY_TARGET_NAME}
#                            NO_CHECK_REQUIRED_COMPONENTS_MACRO)

add_subdirectory(bindings/python/)
add_subdirectory(tests/)
