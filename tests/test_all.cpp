#include "Point.hpp"
#include "Environnement.hpp"
#include "PeriodicEnvironnement.hpp"
#include "algorithms/graph/Dijkstra.hpp"
#include "algorithms/graph/AStar.hpp"
#include "algorithms/graph/FMM.hpp"
#include "algorithms/strategies/ManhattanHeuristic.hpp"
#include "algorithms/strategies/EuclideanHeuristic.hpp"
#include <iostream>
#include <exception>

#ifndef OUTPUT_DIR
#define OUTPUT_DIR ""
#endif

int main() {
    try {
        std::cout << "=== Tests Complets - Tous les Algorithmes ===" << std::endl;
        std::cout << "Ce test reproduit le comportement de l'ancien main.cpp" << std::endl;
        
        // Test 1: Environnement aléatoire avec Dijkstra
        std::cout << "\n=== Test 1: Environnement aléatoire avec Dijkstra ===" << std::endl;
        Environnement env_small = Environnement::createRandomEnvironment({20, 15}, 0.20, 42);
        env_small.toPNG(std::string(OUTPUT_DIR) + "all_environnement_petit.png");
        
        Dijkstra searcher(&env_small);
        searcher.add_start({1.0f, 1.0f});
        searcher.execute();
        searcher.display_U_values_grid();
        searcher.save_U_values_image(std::string(OUTPUT_DIR) + "all_distances_petit.png");
        
        // Test 2: A* avec différentes heuristiques
        std::cout << "\n=== Test 2: A* avec heuristiques ===" << std::endl;
        
        ManhattanHeuristic manhattan(1.0f);
        EuclideanHeuristic euclidean(1.0f);
        
        Environnement env_astar = Environnement::createRandomEnvironment({30, 25}, 0.20, 111);
        
        AStar astar_manhattan(&env_astar, 1.0f, &manhattan);
        astar_manhattan.add_start({1.0f, 1.0f});
        astar_manhattan.add_end({28.0f, 23.0f});
        astar_manhattan.execute();
        astar_manhattan.save_U_values_image(std::string(OUTPUT_DIR) + "all_astar_manhattan.png");
        
        AStar astar_euclidean(&env_astar, 1.0f, &euclidean);
        astar_euclidean.add_start({1.0f, 1.0f});
        astar_euclidean.add_end({28.0f, 23.0f});
        astar_euclidean.execute();
        astar_euclidean.save_U_values_image(std::string(OUTPUT_DIR) + "all_astar_euclidean.png");
        
        // Test 3: FMM
        std::cout << "\n=== Test 3: FMM ===" << std::endl;
        
        Environnement env_fmm = Environnement::createRandomEnvironment({25, 20}, 0.15, 222);
        
        FMM fmm(&env_fmm, 1.0f);
        fmm.add_start({2.0f, 2.0f});
        fmm.execute();
        fmm.save_U_values_image(std::string(OUTPUT_DIR) + "all_fmm_basic.png");
        
        // Test 4: Points flottants
        std::cout << "\n=== Test 4: Points flottants ===" << std::endl;
        
        Environnement env_floating = Environnement::createRandomEnvironment({20, 15}, 0.20, 333);
        
        AStar astar_floating(&env_floating, 1.0f, &manhattan);
        astar_floating.add_start({2.2f, 1.8f});
        astar_floating.add_end({17.6f, 12.4f});
        astar_floating.execute();
        astar_floating.save_U_values_image(std::string(OUTPUT_DIR) + "all_floating_points.png");
        
        // Test 5: Environnements périodiques
        std::cout << "\n=== Test 5: Environnements périodiques ===" << std::endl;
        
        auto env_periodic = PeriodicEnvironnement::createPeriodicRandomEnvironment(
            {20, 15}, {true, true}, 0.15, 444);
        
        Dijkstra dijkstra_periodic(env_periodic.get());
        dijkstra_periodic.add_start({1.0f, 1.0f});
        dijkstra_periodic.execute();
        dijkstra_periodic.save_U_values_image(std::string(OUTPUT_DIR) + "all_periodic.png");
        
        // Test 6: Comparaison finale
        std::cout << "\n=== Test 6: Comparaison finale ===" << std::endl;
        
        Environnement env_final = Environnement::createRandomEnvironment({25, 20}, 0.20, 555);
        
        // Dijkstra
        Dijkstra dijkstra_final(&env_final);
        dijkstra_final.add_start({1.0f, 1.0f});
        dijkstra_final.execute();
        dijkstra_final.save_U_values_image(std::string(OUTPUT_DIR) + "all_final_dijkstra.png");
        
        // FMM
        FMM fmm_final(&env_final);
        fmm_final.add_start({1.0f, 1.0f});
        fmm_final.execute();
        fmm_final.save_U_values_image(std::string(OUTPUT_DIR) + "all_final_fmm.png");
        
        std::cout << "\n=== Tous les tests terminés avec succès! ===" << std::endl;
        std::cout << "Fichiers générés dans: " << OUTPUT_DIR << std::endl;
        std::cout << "\nStructure maintenant organisée en tests séparés:" << std::endl;
        std::cout << "- test_dijkstra: Tests spécifiques à Dijkstra" << std::endl;
        std::cout << "- test_astar: Tests spécifiques à A*" << std::endl;
        std::cout << "- test_fmm: Tests spécifiques à FMM" << std::endl;
        std::cout << "- test_environnement: Tests des environnements" << std::endl;
        std::cout << "- test_periodic: Tests périodiques" << std::endl;
        std::cout << "- test_comparison: Comparaisons d'algorithmes" << std::endl;
        std::cout << "- test_all: Test complet (ce fichier)" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Erreur dans test complet: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}