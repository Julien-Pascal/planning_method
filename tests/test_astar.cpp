#include "algorithms/graph/AStar.hpp"
#include "algorithms/strategies/ManhattanHeuristic.hpp"
#include "algorithms/strategies/EuclideanHeuristic.hpp" // Bien inclure
#include "Point.hpp"
#include "Environnement.hpp"
#include <iostream>
#include <exception>

#ifndef OUTPUT_DIR
#define OUTPUT_DIR ""
#endif

int main() {
    try {
        std::cout << "=== Test A* (minimal) ===" << std::endl;
        
        // Créer les heuristiques - CORRECTION ICI
        ManhattanHeuristic manhattan(1.0f);
        EuclideanHeuristic euclidean(1.0f);  // CORRIGÉ: EuclideanHeuristic au lieu de EuclidianHeuristic
        
        // Test simple
        Environnement env1 = Environnement::createRandomEnvironment({15, 12}, 0.20, 42);
        env1.toPNG(std::string(OUTPUT_DIR) + "astar_test_env.png");
        
        // Test A* Manhattan
        AStar astar1(&env1, 1.0f, &manhattan);
        astar1.add_start({1.0f, 1.0f});
        astar1.add_end({13.0f, 10.0f});
        astar1.execute();
        astar1.save_U_values_image(std::string(OUTPUT_DIR) + "astar_manhattan_result.png");
        
        // Test A* Euclidean - CORRECTION ICI AUSSI
        AStar astar2(&env1, 1.0f, &euclidean);  // CORRIGÉ: &euclidean au lieu d'une variable inexistante
        astar2.add_start({1.0f, 1.0f});
        astar2.add_end({13.0f, 10.0f});
        astar2.execute();
        astar2.save_U_values_image(std::string(OUTPUT_DIR) + "astar_euclidean_result.png");
        
        std::cout << "Test A* terminé avec succès!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Erreur dans test A*: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}