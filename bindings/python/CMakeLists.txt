cmake_minimum_required(VERSION 3.5)

if(${CMAKE_VERSION} VERSION_LESS "3.12.0")
    message("Please consider to switch to latest version to enable
    more reliable Python3 binaries detection")
else()
    find_package(Python ${PYBIND11_PYTHON_VERSION} COMPONENTS Development Interpreter REQUIRED)
endif()

find_package(pybind11 QUIET)

message(STATUS "check for pybind11")
if(${pybind11_FOUND})

    message(STATUS "compiling python bindings")

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    pybind11_add_module(pyphase pyPhase.cpp)
    target_link_libraries(pyphase PUBLIC phase_manager)

#    install(TARGETS pyphase
#            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

#    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
#    pybind11_add_module(pyphase_manager pyPhaseManager.cpp)
#    target_link_libraries(pyci PRIVATE phase_manager)

#    install(TARGETS pyphase_manager
#            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

#    install(FILES pyci_all.py
#            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

#    install(PROGRAMS interactive_client.py
#            DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

else()
    message(STATUS "pybind not found")
endif()
