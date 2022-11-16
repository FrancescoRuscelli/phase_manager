cmake_minimum_required(VERSION 3.5)
find_package(pybind11 QUIET)
add_compile_options(-std=c++14)


message(STATUS "check for pybind11")
if(${pybind11_FOUND})

    message(STATUS "compiling python bindings")

    execute_process(
      COMMAND "${PYTHON_EXECUTABLE}" -c "if True:
        from distutils import sysconfig as sc
        print(sc.get_python_lib(prefix='', plat_specific=True))"
      OUTPUT_VARIABLE PYTHON_SITE
      OUTPUT_STRIP_TRAILING_WHITESPACE)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    pybind11_add_module(pyphase pyPhase.cpp)
    target_link_libraries(pyphase PUBLIC phase_manager)

    install(TARGETS pyphase
            DESTINATION ${PYTHON_SITE}/${PROJECT_NAME})

#    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    pybind11_add_module(pymanager pyPhaseManager.cpp)
    target_link_libraries(pymanager PUBLIC phase_manager)

    install(TARGETS pymanager
            DESTINATION ${PYTHON_SITE}/${PROJECT_NAME})

    install(FILES __init__.py DESTINATION ${PYTHON_SITE}/${PROJECT_NAME})
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
