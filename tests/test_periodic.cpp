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
        std::cout << "=== Tests Environnements Périodiques ===" << std::endl;
        
        // Test 1: Comparaison classique vs périodique
        std::cout << "\n--- Test 1: Comparaison classique vs périodique ---" << std::endl;
        
        // Environnement classique
        Environnement env_classic = Environnement::createRandomEnvironment({20, 15}, 0.15, 42);
        env_classic.toPNG(std::string(OUTPUT_DIR) + "periodic_classic.png");
        
        // Environnement périodique X
        auto env_periodic_x = PeriodicEnvironnement::createPeriodicRandomEnvironment(
            {20, 15}, {true, false}, 0.15, 42);
        env_periodic_x->toPNG(std::string(OUTPUT_DIR) + "periodic_x_only.png");
        
        // Environnement périodique X+Y
        auto env_periodic_xy = PeriodicEnvironnement::createPeriodicRandomEnvironment(
            {20, 15}, {true, true}, 0.15, 42);
        env_periodic_xy->toPNG(std::string(OUTPUT_DIR) + "periodic_xy.png");
        
        // Test 2: Dijkstra avec périodicité
        std::cout << "\n--- Test 2: Dijkstra périodique ---" << std::endl;
        
        Dijkstra dijkstra_classic(&env_classic);
        dijkstra_classic.add_start({1.0f, 1.0f});
        dijkstra_classic.execute();
        dijkstra_classic.save_U_values_image(std::string(OUTPUT_DIR) + "dijkstra_classic.png");
        
        Dijkstra dijkstra_periodic_x(env_periodic_x.get());
        dijkstra_periodic_x.add_start({1.0f, 1.0f});
        dijkstra_periodic_x.execute();
        dijkstra_periodic_x.save_U_values_image(std::string(OUTPUT_DIR) + "dijkstra_periodic_x.png");
        
        Dijkstra dijkstra_periodic_xy(env_periodic_xy.get());
        dijkstra_periodic_xy.add_start({1.0f, 1.0f});
        dijkstra_periodic_xy.execute();
        dijkstra_periodic_xy.save_U_values_image(std::string(OUTPUT_DIR) + "dijkstra_periodic_xy.png");
        
        // Test 3: A* avec heuristiques périodiques
        std::cout << "\n--- Test 3: A* périodique ---" << std::endl;
        
        auto env_astar_periodic = PeriodicEnvironnement::createPeriodicRandomEnvironment(
            {30, 25}, {true, false}, 0.20, 123);
        
        ManhattanHeuristic manhattan(1.0f);
        EuclideanHeuristic euclidean(1.0f);
        
        AStar astar_manhattan(env_astar_periodic.get(), 1.0f, &manhattan);
        astar_manhattan.add_start({2.0f, 2.0f});
        astar_manhattan.add_end({27.0f, 22.0f});
        astar_manhattan.execute();
        astar_manhattan.save_U_values_image(std::string(OUTPUT_DIR) + "astar_manhattan_periodic.png");
        
        AStar astar_euclidean(env_astar_periodic.get(), 1.0f, &euclidean);
        astar_euclidean.add_start({2.0f, 2.0f});
        astar_euclidean.add_end({27.0f, 22.0f});
        astar_euclidean.execute();
        astar_euclidean.save_U_values_image(std::string(OUTPUT_DIR) + "astar_euclidean_periodic.png");
        
        // Test 4: Calculs de distance périodique
        std::cout << "\n--- Test 4: Calculs de distance périodique ---" << std::endl;
        
        auto env_distance = std::make_unique<PeriodicEnvironnement>(
            std::vector<int>{10, 8}, std::vector<bool>{true, true});
        
        std::vector<float> point_a = {1.0f, 1.0f};
        std::vector<float> point_b = {9.0f, 7.0f};
        
        float manhattan_dist = env_distance->calculate_distance(point_a, point_b, 1);
        float euclidean_dist = env_distance->calculate_distance(point_a, point_b, 2);
        
        std::cout << "Distance (1,1) -> (9,7) dans grille 10x8 périodique:" << std::endl;
        std::cout << "  Manhattan: " << manhattan_dist << " (classique: " << (8+6) << ")" << std::endl;
        std::cout << "  Euclidienne: " << euclidean_dist << " (classique: " << std::sqrt(8*8 + 6*6) << ")" << std::endl;
        
        // Test 5: FMM périodique
        std::cout << "\n--- Test 5: FMM périodique ---" << std::endl;
        
        auto env_fmm_periodic = PeriodicEnvironnement::createPeriodicRandomEnvironment(
            {25, 20}, {true, true}, 0.10, 456);
        
        FMM fmm_periodic(env_fmm_periodic.get(), 1.0f);
        fmm_periodic.add_start({12.0f, 10.0f});
        fmm_periodic.execute();
        fmm_periodic.save_U_values_image(std::string(OUTPUT_DIR) + "fmm_periodic.png");
        
        // Test 6: Labyrinthe périodique
        std::cout << "\n--- Test 6: Labyrinthe périodique ---" << std::endl;
        
        auto maze_periodic = PeriodicEnvironnement::createPeriodicMazeEnvironment(
            {20, 15}, {true, false}, 789);
        maze_periodic->toPNG(std::string(OUTPUT_DIR) + "maze_periodic.png");
        
        Dijkstra dijkstra_maze(maze_periodic.get());
        dijkstra_maze.add_start({0.0f, 0.0f});
        dijkstra_maze.execute();
        dijkstra_maze.save_U_values_image(std::string(OUTPUT_DIR) + "maze_periodic_distances.png");
        
        std::cout << "\n=== Tests périodiques terminés! ===" << std::endl;
        std::cout << "Comparaison visuelle: classique vs périodique recommandée!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Erreur dans test périodique: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}