# Tests CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

# Fonction helper pour créer les tests
function(add_planning_test test_name)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${test_name}.cpp")
        add_executable(${test_name} ${test_name}.cpp)
        target_link_libraries(${test_name} planning_method_lib)
        target_include_directories(${test_name} PRIVATE 
            ${CMAKE_SOURCE_DIR}/include
            ${CMAKE_SOURCE_DIR}/external
        )
        
        # Définir le répertoire de sortie pour les images
        target_compile_definitions(${test_name} PRIVATE 
            OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_results/"
        )
        
        # S'assurer que la bibliothèque est construite avant les tests
        add_dependencies(${test_name} planning_method_lib)
        
        message(STATUS "Added test: ${test_name}")
    else()
        message(WARNING "Test source not found: ${CMAKE_CURRENT_SOURCE_DIR}/${test_name}.cpp")
    endif()
endfunction()

# Créer le répertoire de sortie
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/test_results)

# Tests individuels
add_planning_test(test_dijkstra)
add_planning_test(test_astar)
add_planning_test(test_fmm)
add_planning_test(test_environnement)
add_planning_test(test_periodic)
add_planning_test(test_comparison)

# Test unifié (équivalent à l'ancien main)
if(BUILD_ALL_TEST)
    add_planning_test(test_all)
endif()

# Targets personnalisés pour exécuter les tests
add_custom_target(run_all_tests
    COMMAND ${CMAKE_COMMAND} -E echo "Running all Planning Method tests..."
    COMMAND test_dijkstra
    COMMAND test_astar  
    COMMAND test_fmm
    COMMAND test_environnement
    COMMAND test_periodic
    COMMAND test_comparison
    DEPENDS test_dijkstra test_astar test_fmm test_environnement test_periodic test_comparison
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

add_custom_target(run_dijkstra_test
    COMMAND test_dijkstra
    DEPENDS test_dijkstra
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

add_custom_target(run_astar_test
    COMMAND test_astar
    DEPENDS test_astar
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

add_custom_target(run_fmm_test
    COMMAND test_fmm
    DEPENDS test_fmm
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)