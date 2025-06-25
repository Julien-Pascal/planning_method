cmake_minimum_required(VERSION 3.16)
project(planning_method VERSION 1.0.0 LANGUAGES CXX)

# Configuration C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Options de compilation
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra -pedantic")
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
endif()

# Répertoires d'inclusion
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/external
)

# =====================================================================
# COLLECTE DES SOURCES
# =====================================================================

set(CORE_SOURCES
    src/Point.cpp
    src/Environnement.cpp
    src/PeriodicEnvironnement.cpp
    src/algorithms/graph/GraphSearchBase.cpp
    src/algorithms/graph/Dijkstra.cpp
    src/algorithms/graph/AStar.cpp
    src/algorithms/graph/FMM.cpp
    src/utils/PathExtractor.cpp
)

# =====================================================================
# CRÉATION DE LA BIBLIOTHÈQUE
# =====================================================================

add_library(planning_method_lib STATIC ${CORE_SOURCES})

target_include_directories(planning_method_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/external>
    $<INSTALL_INTERFACE:include>
)

# =====================================================================
# OPTIONS DE BUILD
# =====================================================================

option(BUILD_TESTS "Build test programs" ON)
option(BUILD_ALL_TEST "Build unified test executable" ON)

# =====================================================================
# TESTS - avec dépendance sur la bibliothèque
# =====================================================================

if(BUILD_TESTS)
    # Vérifier que le répertoire tests existe
    if(EXISTS "${CMAKE_SOURCE_DIR}/tests")
        add_subdirectory(tests)
    else()
        message(WARNING "Tests directory not found. Please create: ${CMAKE_SOURCE_DIR}/tests")
        message(STATUS "Tests will be skipped.")
        set(BUILD_TESTS OFF)
    endif()
endif()

# =====================================================================
# INSTALLATION
# =====================================================================

install(TARGETS planning_method_lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

# =====================================================================
# INFORMATIONS DE CONFIGURATION
# =====================================================================

message(STATUS "=== Planning Method Configuration ===")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Build Tests: ${BUILD_TESTS}")
message(STATUS "Build All Test: ${BUILD_ALL_TEST}")
message(STATUS "Tests Directory: ${CMAKE_SOURCE_DIR}/tests")
message(STATUS "=======================================")