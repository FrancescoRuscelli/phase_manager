cmake_minimum_required(VERSION 3.5)

project(phase_manager LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 3.3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             message_generation
             )

include_directories(
    include/
    src/
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

add_message_files(
    FILES
    Timeline.msg
    TimelineArray.msg
    )


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

set(LIBRARY_TARGET_NAME phase_manager) # set library name

set(phase_manager_SRC
    src/phase_manager.cpp
    src/timeline.cpp
    src/phase.cpp

    )

set(phase_manager_ros_SRC
    src/ros_server_class.cpp
    )

add_library(${LIBRARY_TARGET_NAME} SHARED
  ${phase_manager_SRC}
  ${phase_manager_ros_SRC}
)

target_link_libraries(${LIBRARY_TARGET_NAME}
    PUBLIC
    ${catkin_LIBRARIES}
    )

target_include_directories(${LIBRARY_TARGET_NAME}
    INTERFACE
    ${EIGEN3_INCLUDE_DIR})

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
