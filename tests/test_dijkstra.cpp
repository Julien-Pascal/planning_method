#include "Point.hpp"
#include "Environnement.hpp"
#include "algorithms/graph/Dijkstra.hpp"
#include <iostream>
#include <exception>

#ifndef OUTPUT_DIR
#define OUTPUT_DIR ""
#endif

int main() {
    try {
        std::cout << "=== Tests Algorithme Dijkstra ===" << std::endl;
        
        // Test 1: Environnement simple
        std::cout << "\n--- Test 1: Environnement aléatoire simple ---" << std::endl;
        Environnement env1 = Environnement::createRandomEnvironment({20, 15}, 0.20, 42);
        env1.toPNG(std::string(OUTPUT_DIR) + "dijkstra_env1.png");
        
        Dijkstra dijkstra1(&env1);
        dijkstra1.add_start({1.0f, 1.0f});
        dijkstra1.execute();
        dijkstra1.display_U_values_grid();
        dijkstra1.save_U_values_image(std::string(OUTPUT_DIR) + "dijkstra_result1.png");
        
        // Test 2: Environnement plus grand
        std::cout << "\n--- Test 2: Environnement plus grand ---" << std::endl;
        Environnement env2 = Environnement::createRandomEnvironment({50, 40}, 0.15, 123);
        env2.toPNG(std::string(OUTPUT_DIR) + "dijkstra_env2.png");
        
        Dijkstra dijkstra2(&env2);
        dijkstra2.add_start({25.0f, 20.0f});
        dijkstra2.execute();
        dijkstra2.save_U_values_image(std::string(OUTPUT_DIR) + "dijkstra_result2.png");
        
        // Test 3: Labyrinthe
        std::cout << "\n--- Test 3: Labyrinthe ---" << std::endl;
        Environnement env3 = Environnement::createMazeEnvironment({30, 25}, 456);
        env3.toPNG(std::string(OUTPUT_DIR) + "dijkstra_maze.png");
        
        Dijkstra dijkstra3(&env3);
        dijkstra3.add_start({0.0f, 0.0f});
        dijkstra3.execute();
        dijkstra3.save_U_values_image(std::string(OUTPUT_DIR) + "dijkstra_maze_result.png");
        
        // Test 4: Multiple sources
        std::cout << "\n--- Test 4: Sources multiples ---" << std::endl;
        Environnement env4 = Environnement::createRandomEnvironment({25, 20}, 0.25, 789);
        
        Dijkstra dijkstra4(&env4);
        dijkstra4.add_start({2.0f, 2.0f});
        dijkstra4.add_start({22.0f, 17.0f});
        dijkstra4.add_start({12.0f, 10.0f});
        dijkstra4.execute();
        dijkstra4.save_U_values_image(std::string(OUTPUT_DIR) + "dijkstra_multi_sources.png");
        
        // Test 5: Coûts d'arête variables
        std::cout << "\n--- Test 5: Coûts d'arête variables ---" << std::endl;
        Environnement env5 = Environnement::createRandomEnvironment({20, 15}, 0.20, 999);
        
        Dijkstra dijkstra5(&env5, 2.5f);
        dijkstra5.add_start({0.0f, 1.0f});
        dijkstra5.execute();
        dijkstra5.save_U_values_image(std::string(OUTPUT_DIR) + "dijkstra_cost_25.png");
        
        std::cout << "\n=== Tests Dijkstra terminés avec succès! ===" << std::endl;
        std::cout << "Fichiers générés dans " << OUTPUT_DIR << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Erreur dans test Dijkstra: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}