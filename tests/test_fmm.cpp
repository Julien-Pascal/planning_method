#include "Point.hpp"
#include "Environnement.hpp" 
#include "algorithms/graph/FMM.hpp"
#include <iostream>
#include <exception>

#ifndef OUTPUT_DIR
#define OUTPUT_DIR ""
#endif

int main() {
    try {
        std::cout << "=== Tests Algorithme FMM ===" << std::endl;
        
        // Test 1: FMM basique
        std::cout << "\n--- Test 1: FMM basique ---" << std::endl;
        Environnement env1 = Environnement::createRandomEnvironment({25, 20}, 0.10, 111);
        env1.toPNG(std::string(OUTPUT_DIR) + "fmm_basic_env.png");
        
        FMM fmm1(&env1, 1.0f);
        fmm1.add_start({2.0f, 2.0f});
        fmm1.execute();
        fmm1.display_U_values_grid();
        fmm1.save_U_values_image(std::string(OUTPUT_DIR) + "fmm_basic_result.png");
        
        // Test 2: FMM avec coût élevé
        std::cout << "\n--- Test 2: FMM coût élevé ---" << std::endl;
        Environnement env2 = Environnement::createRandomEnvironment({25, 20}, 0.20, 222);
        
        FMM fmm2(&env2, 3.0f);
        fmm2.add_start({2.0f, 2.0f});
        fmm2.execute();
        fmm2.save_U_values_image(std::string(OUTPUT_DIR) + "fmm_high_cost.png");
        
        std::cout << "Coût utilisé: " << fmm2.get_cost_per_unit_distance() << std::endl;
        
        // Test 3: FMM dans labyrinthe
        std::cout << "\n--- Test 3: FMM dans labyrinthe ---" << std::endl;
        Environnement env3 = Environnement::createMazeEnvironment({30, 25}, 333);
        env3.toPNG(std::string(OUTPUT_DIR) + "fmm_maze_env.png");
        
        FMM fmm3(&env3, 1.0f);
        fmm3.add_start({0.0f, 0.0f});
        fmm3.execute();
        fmm3.save_U_values_image(std::string(OUTPUT_DIR) + "fmm_maze_result.png");
        
        // Test 4: FMM multiples sources
        std::cout << "\n--- Test 4: FMM sources multiples ---" << std::endl;
        Environnement env4 = Environnement::createRandomEnvironment({30, 25}, 0.15, 444);
        
        FMM fmm4(&env4, 1.5f);
        fmm4.add_start({3.0f, 3.0f});
        fmm4.add_start({26.0f, 21.0f});
        fmm4.add_start({15.0f, 12.0f});
        fmm4.execute();
        fmm4.save_U_values_image(std::string(OUTPUT_DIR) + "fmm_multi_sources.png");
        
        // Test 5: FMM avec coût faible (propagation rapide)
        std::cout << "\n--- Test 5: FMM propagation rapide ---" << std::endl;
        Environnement env5 = Environnement::createRandomEnvironment({20, 15}, 0.20, 555);
        
        FMM fmm5(&env5, 0.5f);
        fmm5.add_start({10.0f, 7.0f});
        fmm5.execute();
        fmm5.save_U_values_image(std::string(OUTPUT_DIR) + "fmm_fast_propagation.png");
        
        std::cout << "\n=== Analyse coûts FMM ===" << std::endl;
        std::cout << "Coût 0.5 → Propagation 2x plus rapide" << std::endl;
        std::cout << "Coût 1.0 → Propagation normale" << std::endl;
        std::cout << "Coût 3.0 → Propagation 3x plus lente" << std::endl;
        
        std::cout << "\n=== Tests FMM terminés avec succès! ===" << std::endl;
        std::cout << "Fichiers générés dans " << OUTPUT_DIR << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Erreur dans test FMM: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}