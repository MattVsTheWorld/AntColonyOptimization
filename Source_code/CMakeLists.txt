cmake_minimum_required(VERSION 3.10)
project(main)

if(WIN32)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_MODULE_PATH               "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
    set(CMAKE_CXX_FLAGS                 "${CMAKE_CXX_FLAGS}")
    set(CMAKE_CXX_FLAGS_RELEASE         "${CMAKE_CXX_FLAGS_RELEASE} -Ox")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "${CMAKE_CURRENT_SOURCE_DIR}/build")
endif()
if(UNIX)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_MODULE_PATH               "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
    set(CMAKE_CXX_FLAGS                 "${CMAKE_CXX_FLAGS} -O3")
    set(CMAKE_CXX_FLAGS_RELEASE         "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG -O3 -flto")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "${CMAKE_CURRENT_SOURCE_DIR}/build")
    set(LINKER_OPTIONS                  -flto -Wl,--no-as-needed)
endif()

if(CMAKE_SYSTEM_NAME STREQUAL Darwin)
    set(CMAKE_CXX_FLAGS_DEBUG       "${CMAKE_CXX_FLAGS_DEBUG} -save-temps=obj")
endif()

find_package(Cplex REQUIRED)
include_directories(SYSTEM ${CPLEX_INCLUDE_DIRS})

add_executable(main
        src/main.cpp
        src/Solvers/cpxmacro.h
        src/Utilities/utils.cpp
        src/Utilities/utils.h
        "src/Instance Generators/boardGenerator.h"
        "src/Instance Generators/boardGenerator.cpp"
        src/Solvers/TSPSolver.h
        src/Solvers/TSPSolver.cpp
        src/Solvers/solverManager.h
        src/Solvers/solverManager.cpp
        "src/Instance Generators/figureGenerator.cpp"
        "src/Instance Generators/figureGenerator.h"
        src/Utilities/typesAndDefs.h "src/Solvers/Ant Colony System/ACSHeuristic.cpp"
        "src/Solvers/Ant Colony System/ACSHeuristic.h" "src/Solvers/Ant Colony System/Ant.cpp"
        "src/Solvers/Ant Colony System/Ant.h" src/Utilities/unitTest.cpp src/Utilities/unitTest.h)

target_link_libraries(  main PRIVATE ${CPLEX_LIBRARIES})
if(UNIX)
    target_link_libraries(  main PRIVATE dl)
endif()