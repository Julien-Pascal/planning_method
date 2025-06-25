cmake_minimum_required(VERSION 3.10)
project(PathFinding VERSION 1.0.0 LANGUAGES CXX)

# Configuration C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Options de compilation
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")
    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
endif()

# Répertoires d'inclusion
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/external
)

# =====================================================================
# COLLECTE DES SOURCES DE BASE (selon votre structure exacte)
# =====================================================================

# Sources de base (toujours nécessaires)
set(CORE_SOURCES
    src/Point.cpp
    src/Environnement.cpp
    src/PeriodicEnvironnement.cpp
    src/algorithms/graph/GraphSearchBase.cpp
    src/algorithms/graph/Dijkstra.cpp
    src/algorithms/graph/AStar.cpp
    src/algorithms/graph/FMM.cpp
)

# Note: Ajoutez ici les futurs fichiers selon votre structure VSCode :
# src/utils/PathExtractor.cpp (quand implémenté)
# src/algorithms/SearchAlgorithm.cpp (interface de base future)
# src/algorithms/graph/AStar.cpp (quand implémenté)
# src/algorithms/graph/FMM.cpp (quand implémenté)
# src/algorithms/graph/neighbors/GridNeighbors.cpp (quand implémenté)
# src/algorithms/graph/neighbors/EuclideanNeighbors.cpp (quand implémenté)
# src/algorithms/random/RandomSearchBase.cpp (quand implémenté)

# =====================================================================
# CRÉATION DE LA BIBLIOTHÈQUE
# =====================================================================

add_library(pathfinding_lib STATIC ${CORE_SOURCES})

# Configuration de la bibliothèque
target_include_directories(pathfinding_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/external>
    $<INSTALL_INTERFACE:include>
)

# =====================================================================
# EXÉCUTABLE PRINCIPAL
# =====================================================================

add_executable(graph_search src/main.cpp)
target_link_libraries(graph_search pathfinding_lib)

# =====================================================================
# INSTALLATION
# =====================================================================

install(TARGETS pathfinding_lib graph_search
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

# =====================================================================
# TESTS (optionnel)
# =====================================================================

option(BUILD_TESTS "Build test programs" OFF)
if(BUILD_TESTS)
    enable_testing()
    # add_subdirectory(tests)  # Décommentez quand vous créerez le dossier tests
endif()

# =====================================================================
# DOCUMENTATION (optionnel)
# =====================================================================

find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
    option(BUILD_DOCS "Build documentation" OFF)
    if(BUILD_DOCS)
        configure_file(${CMAKE_SOURCE_DIR}/docs/Doxyfile.in 
                      ${CMAKE_BINARY_DIR}/Doxyfile @ONLY)
        add_custom_target(docs
            ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/Doxyfile
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
        )
    endif()
endif()

# =====================================================================
# INFORMATIONS DE CONFIGURATION
# =====================================================================

message(STATUS "=== PathFinding Configuration ===")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Install Prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "")
message(STATUS "=== Structure actuelle ===")
message(STATUS "Core sources: ${CORE_SOURCES}")
message(STATUS "Main executable: graph_search")
message(STATUS "Library: pathfinding_lib")
message(STATUS "")
message(STATUS "=== Pour étendre le projet ===")
message(STATUS "1. Ajoutez vos nouveaux .cpp dans CORE_SOURCES")
message(STATUS "2. Créez les .hpp correspondants dans include/")
message(STATUS "3. Reconfigurer avec cmake")
message(STATUS "===============================")