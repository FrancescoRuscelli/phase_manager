cmake_minimum_required(VERSION 3.5)
include(ExternalProject)

# compite GTest
find_package(GTest)
find_package(Threads)
if(NOT GTEST_FOUND)
    set(gTestSource "/usr/src/gtest")
    ExternalProject_Add(GTest-ext SOURCE_DIR ${gTestSource}
                                  PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                                  INSTALL_COMMAND "")
    set(GTEST_LIB_DIRS "${CMAKE_CURRENT_BINARY_DIR}/external/src/GTest-ext-build/")
    set(GTEST_INCLUDE_DIRS ${gTestSource})
    set(GTEST_BOTH_LIBRARIES gtest gtest_main)
endif()


add_executable(testPhaseManager letstry.cpp)
target_link_libraries(testPhaseManager ${TestLibs} ${LIBRARY_TARGET_NAME})
#add_test(NAME testPhaseManager COMMAND testPhaseManager)

