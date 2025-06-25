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

int main()
{
    try {
        std::cout << "=== Test 1: Environnement aléatoire avec pathfinding (Dijkstra) ===" << std::endl;
        
        // Créer un petit environnement pour voir les détails
        Environnement env_small = Environnement::createRandomEnvironment({20, 15}, 0.20, 42);
        env_small.toPNG("environnement_petit.png");
        
        // Exécuter la recherche de chemin avec Dijkstra
        Dijkstra searcher(&env_small);
        searcher.add_start({1.0f, 1.0f});  // Point de départ en (1,1)
        searcher.execute();
        
        // Afficher les U_values en grille
        searcher.display_U_values_grid();
        
        // Sauvegarder l'image des distances
        searcher.save_U_values_image("distances_petit.png");
        
        std::cout << "\n=== Test 2: Environnement plus grand ===" << std::endl;
        
        // Créer un environnement plus grand
        Environnement env_big = Environnement::createRandomEnvironment({50, 40}, 0.15, 123);
        env_big.toPNG("environnement_grand.png");
        
        // Recherche avec point de départ au centre
        Dijkstra searcher_big(&env_big);
        searcher_big.add_start({25.0f, 20.0f});
        searcher_big.execute();
        searcher_big.save_U_values_image("distances_grand.png");
        
        std::cout << "\n=== Test 3: Labyrinthe avec pathfinding ===" << std::endl;
        
        // Créer un labyrinthe
        Environnement env_maze = Environnement::createMazeEnvironment({30, 25}, 456);
        env_maze.toPNG("labyrinthe_petit.png");
        
        // Recherche dans le labyrinthe (départ coin en bas à gauche)
        Dijkstra searcher_maze(&env_maze);
        searcher_maze.add_start({0.0f, 0.0f});
        searcher_maze.execute();
        searcher_maze.save_U_values_image("distances_labyrinthe.png");
        
        std::cout << "\n=== Test 4: Multiples points de départ ===" << std::endl;
        
        // Environnement avec plusieurs sources
        Environnement env_multi_classic = Environnement::createRandomEnvironment({25, 20}, 0.25, 789);
        
        Dijkstra searcher_multi(&env_multi_classic);
        searcher_multi.add_start({2.0f, 2.0f});    // Coin gauche
        searcher_multi.add_start({22.0f, 17.0f});  // Coin droit
        searcher_multi.add_start({12.0f, 10.0f});  // Centre
        searcher_multi.execute();
        
        std::cout << "Grille avec 3 points de départ:" << std::endl;
        searcher_multi.display_U_values_grid();
        searcher_multi.save_U_values_image("distances_multi_sources.png");
        
        std::cout << "\n=== Test 5: Chargement depuis PNG (si disponible) ===" << std::endl;
        
        try {
            Environnement env_from_file = Environnement::fromPNG("carte.png", 128);
            env_from_file.toPNG("carte_rechargee.png");
            
            Dijkstra searcher_file(&env_from_file);
            auto dims = env_from_file.get_dims();
            // Point de départ près du coin
            searcher_file.add_start({static_cast<float>(dims[0]/10), static_cast<float>(dims[1]/10)});
            searcher_file.execute();
            searcher_file.save_U_values_image("distances_carte.png");
            
        } catch (const std::exception& e) {
            std::cout << "Fichier 'carte.png' non trouvé: " << e.what() << std::endl;
        }
        
        std::cout << "\n=== Test 6: Test avec différents coûts d'arêtes ===" << std::endl;
        
        // Test avec coût d'arête différent
        Environnement env_cost = Environnement::createRandomEnvironment({15, 15}, 0.20, 999);
        
        Dijkstra searcher_cost(&env_cost, 2.5f); // Coût d'arête = 2.5
        searcher_cost.add_start({0.0f, 1.0f});
        searcher_cost.execute();
        searcher_cost.save_U_values_image("distances_cout_25.png");
        
        std::cout << "Test avec coût d'arête = " << searcher_cost.get_edge_cost() << std::endl;
        
       // --- Nouveaux tests A* ---
        std::cout << "\n\n=== Nouveaux tests: Algorithme A* ===" << std::endl;

        // Créez des instances de vos heuristiques.
        // Elles doivent exister tant qu'elles sont utilisées par l'AStar solver.
        ManhattanHeuristic manhattan_strat(1.0f); // 1.0f pour une heuristique admissible
        EuclideanHeuristic euclidean_strat(1.0f); // 1.0f pour une heuristique admissible

        // Reutilisez les environnements ou créez-en de nouveaux pour A*
        // N'oubliez pas que A* a besoin d'un point d'arrivée pour sa fonction heuristique!

        // --- Test A* avec Manhattan Heuristic ---
        std::cout << "\n--- Test A* avec Heuristique Manhattan ---" << std::endl;
        Environnement env_astar_manhattan = Environnement::createRandomEnvironment({30, 25}, 0.20, 111);
        env_astar_manhattan.toPNG("environnement_astar_manhattan.png");

        AStar astar_manhattan(&env_astar_manhattan, 1.0f, &manhattan_strat); // Coût par défaut, heuristique Manhattan
        astar_manhattan.add_start({1.0f, 1.0f});
        astar_manhattan.add_end({28.0f, 23.0f}); // Point d'arrivée nécessaire pour A*
        astar_manhattan.execute();

        std::cout << "A* (Manhattan) - Nom heuristique: " << astar_manhattan.get_heuristic_strategy()->get_name() << std::endl;
        std::cout << "A* (Manhattan) - Description: " << astar_manhattan.get_heuristic_strategy()->get_description() << std::endl;
        astar_manhattan.save_U_values_image("distances_astar_manhattan.png");

        // --- Test A* avec Euclidean Heuristic ---
        std::cout << "\n--- Test A* avec Heuristique Euclidienne ---" << std::endl;
        Environnement env_astar_euclidean = Environnement::createRandomEnvironment({50, 50}, 0.0, 555);//Environnement::createMazeEnvironment({40, 30}, 222);
        env_astar_euclidean.toPNG("environnement_astar_euclidean.png");

        AStar astar_euclidean(&env_astar_euclidean, 1.0f, &euclidean_strat); // Heuristique Euclidienne
        astar_euclidean.add_start({0.0f, 0.0f});
        astar_euclidean.add_end({49.0f, 10.0f}); // Point d'arrivée nécessaire
        astar_euclidean.execute();

        std::cout << "A* (Euclidienne) - Nom heuristique: " << astar_euclidean.get_heuristic_strategy()->get_name() << std::endl;
        std::cout << "A* (Euclidienne) - Description: " << astar_euclidean.get_heuristic_strategy()->get_description() << std::endl;
        astar_euclidean.save_U_values_image("distances_astar_euclidean.png");

        // --- Test A* avec multiple ends ---
        std::cout << "\n--- Test A* avec Multiples Points d'Arrivée (Manhattan) ---" << std::endl;
        Environnement env_astar_multi_end = Environnement::createRandomEnvironment({35, 30}, 0.18, 333);
        env_astar_multi_end.toPNG("environnement_astar_multi_end.png");

        AStar astar_multi_end(&env_astar_multi_end, 1.0f, &manhattan_strat);
        astar_multi_end.add_start({1.0f, 1.0f});
        astar_multi_end.add_end({33.0f, 28.0f}); // Premier point d'arrivée
        astar_multi_end.add_end({10.0f, 25.0f}); // Deuxième point d'arrivée
        astar_multi_end.add_end({30.0f, 5.0f});  // Troisième point d'arrivée
        astar_multi_end.execute();

        astar_multi_end.save_U_values_image("distances_astar_multi_end.png");

        // --- Test FMM basique ---
        std::cout << "\n--- Test FMM basique (coût uniforme) ---" << std::endl;
        Environnement env_fmm_basic = Environnement::createRandomEnvironment({25, 20}, 0.0, 555);
        env_fmm_basic.toPNG("environnement_fmm_basic.png");
        
        FMM fmm_basic(&env_fmm_basic, 1.0f); // Coût par défaut
        fmm_basic.add_start({2.0f, 2.0f});
        fmm_basic.execute();
        fmm_basic.display_U_values_grid();
        
        std::cout << "FMM basique - Coût par unité: " << fmm_basic.get_cost_per_unit_distance() << std::endl;
        fmm_basic.save_U_values_image("distances_fmm_basic.png");
        
        // --- Test FMM avec différents coûts ---
        std::cout << "\n--- Test FMM avec coût élevé ---" << std::endl;
        Environnement env_fmm_cost = Environnement::createRandomEnvironment({25, 20}, 0.20, 555);
        
        FMM fmm_high_cost(&env_fmm_cost, 3.0f); // Coût élevé
        fmm_high_cost.add_start({2.0f, 2.0f});
        fmm_high_cost.execute();
        
        std::cout << "FMM coût élevé - Coût par unité: " << fmm_high_cost.get_cost_per_unit_distance() << std::endl;
        fmm_high_cost.save_U_values_image("distances_fmm_high_cost.png");
        
        // --- Test FMM dans un labyrinthe ---
        std::cout << "\n--- Test FMM dans un labyrinthe ---" << std::endl;
        Environnement env_fmm_maze = Environnement::createMazeEnvironment({30, 25}, 666);
        env_fmm_maze.toPNG("environnement_fmm_maze.png");
        
        FMM fmm_maze(&env_fmm_maze, 1.0f);
        fmm_maze.add_start({0.0f, 0.0f});
        fmm_maze.execute();
        fmm_maze.save_U_values_image("distances_fmm_maze.png");
        
        // --- Test FMM avec multiples sources ---
        std::cout << "\n--- Test FMM avec multiples sources ---" << std::endl;
        Environnement env_fmm_multi = Environnement::createRandomEnvironment({30, 25}, 0.15, 777);
        env_fmm_multi.toPNG("environnement_fmm_multi.png");
        
        FMM fmm_multi(&env_fmm_multi, 1.5f);
        fmm_multi.add_start({3.0f, 3.0f});     // Source 1
        fmm_multi.add_start({26.0f, 21.0f});   // Source 2
        fmm_multi.add_start({15.0f, 12.0f});   // Source 3 (centre)
        fmm_multi.execute();
        
        std::cout << "FMM multi-sources - Nombre de sources: 3" << std::endl;
        std::cout << "FMM multi-sources - Coût par unité: " << fmm_multi.get_cost_per_unit_distance() << std::endl;
        fmm_multi.save_U_values_image("distances_fmm_multi.png");
        
        // --- Test FMM avec point d'arrivée (arrêt anticipé) ---
        std::cout << "\n--- Test FMM avec point d'arrivée ---" << std::endl;
        Environnement env_fmm_target = Environnement::createRandomEnvironment({35, 30}, 0.20, 888);
        env_fmm_target.toPNG("environnement_fmm_target.png");
        
        FMM fmm_target(&env_fmm_target, 1.0f);
        fmm_target.add_start({1.0f, 1.0f});
        fmm_target.add_end({33.0f, 28.0f}); // Point d'arrivée pour arrêt anticipé
        fmm_target.execute();
        
        std::cout << "FMM avec cible - Propagation jusqu'au point d'arrivée" << std::endl;
        fmm_target.save_U_values_image("distances_fmm_target.png");
        
        // --- Comparaison FMM vs Dijkstra ---
        std::cout << "\n--- Comparaison FMM vs Dijkstra (même environnement) ---" << std::endl;
        Environnement env_comparison = Environnement::createRandomEnvironment({20, 15}, 0.25, 999);
        env_comparison.toPNG("environnement_comparison.png");
        
        // Dijkstra
        Dijkstra dijkstra_comp(&env_comparison, 1.0f);
        dijkstra_comp.add_start({1.0f, 1.0f});
        dijkstra_comp.execute();
        dijkstra_comp.save_U_values_image("distances_comparison_dijkstra.png");
        
        // FMM
        FMM fmm_comp(&env_comparison, 1.0f);
        fmm_comp.add_start({1.0f, 1.0f});
        fmm_comp.execute();
        fmm_comp.save_U_values_image("distances_comparison_fmm.png");
        
        std::cout << "Comparaison sauvegardée: distances_comparison_dijkstra.png vs distances_comparison_fmm.png" << std::endl;
        
        // --- Test FMM avec coût fractionnaire ---
        std::cout << "\n--- Test FMM avec coût fractionnaire (propagation rapide) ---" << std::endl;
        Environnement env_fmm_fast = Environnement::createRandomEnvironment({20, 15}, 0.20, 101);
        
        FMM fmm_fast(&env_fmm_fast, 0.5f); // Coût faible = propagation rapide
        fmm_fast.add_start({10.0f, 7.0f}); // Centre
        fmm_fast.execute();
        
        std::cout << "FMM propagation rapide - Coût par unité: " << fmm_fast.get_cost_per_unit_distance() << std::endl;
        fmm_fast.save_U_values_image("distances_fmm_fast.png");
        
        std::cout << "\n=== Analyse des coûts FMM ===" << std::endl;
        std::cout << "Coût = 0.5 → Propagation 2x plus rapide" << std::endl;
        std::cout << "Coût = 1.0 → Propagation normale" << std::endl;
        std::cout << "Coût = 3.0 → Propagation 3x plus lente" << std::endl;
        std::cout << "FMM résout l'équation Eikonal: |∇T| = F⁻¹ (où F⁻¹ = coût)" << std::endl;

        std::cout << "\n=== Analyse des résultats (Dijkstra) ===" << std::endl;

        // Analyser l'environnement petit (Dijkstra)
        auto dims_small = env_small.get_dims();
        std::cout << "Environnement petit - Dimensions: " << dims_small[0] << "x" << dims_small[1] << std::endl;

        // Compter les points accessibles
        int reachable = 0, unreachable = 0, obstacles = 0;
        auto point_map_small = env_small.get_map(); // Assuming get_map() exists and is accessible

        for (const auto& pair : point_map_small) {
            const Point& point = pair.second;
            if (point.get_obs()) {
                obstacles++;
            } else if (point.get_value() == INFINITY) { // Assuming INFINITY is defined
                unreachable++;
            } else {
                reachable++;
            }
        }

        std::cout << "Obstacles: " << obstacles << std::endl;
        std::cout << "Points accessibles (Dijkstra): " << reachable << std::endl;
        std::cout << "Points inaccessibles (Dijkstra): " << unreachable << std::endl;

        std::cout << "\n\n=== Nouveaux tests: Algorithme FMM (Fast Marching Method) ===" << std::endl;
        

        std::cout << "\n=== Test des points de départ et d'arrivée flottants ===" << std::endl;
    
        // Créer un environnement
        Environnement env = Environnement::createRandomEnvironment({20, 15}, 0.20, 42);
        env.toPNG("environnement_floating_test.png");
        
        // Test avec Dijkstra
        std::cout << "\n--- Dijkstra avec points flottants ---" << std::endl;
        Dijkstra dijkstra(&env);
        
        // Point de départ flottant au lieu de (5,5)
        dijkstra.add_start({5.3f, 5.7f});
        
        dijkstra.execute();
        dijkstra.save_U_values_image("distances_dijkstra_floating.png");
        
        // Test avec A* 
        std::cout << "\n--- A* avec points de départ et d'arrivée flottants ---" << std::endl;
        ManhattanHeuristic manhattan_heuristic(1.0f);
        AStar astar(&env, 1.0f, &manhattan_heuristic);
        
        // Point de départ et d'arrivée flottants
        astar.add_start({2.2f, 1.8f});      // Au lieu de (2,2)
        astar.add_end({17.6f, 12.4f});      // Au lieu de (18,12)
        
        astar.execute();
        astar.save_U_values_image("distances_astar_floating.png");
        
        // Comparaison avec points entiers classiques
        std::cout << "\n--- Comparaison avec points entiers ---" << std::endl;
        AStar astar_grid(&env, 1.0f, &manhattan_heuristic);
        astar_grid.add_start({2.0f, 2.0f});          // Point entier proche
        astar_grid.add_end({18.0f, 12.0f});          // Point entier proche
        
        astar_grid.execute();
        astar_grid.save_U_values_image("distances_astar_grid.png");
        
        // Test avec FMM
        std::cout << "\n--- FMM avec points flottants ---" << std::endl;
        FMM fmm(&env);
        
        fmm.add_start({10.5f, 7.5f});       // Centre flottant
        
        fmm.execute();
        fmm.save_U_values_image("distances_fmm_floating.png");

        // ====================================================================
        // TESTS PÉRIODIQUES COMMENCENT ICI
        // ====================================================================
        std::cout << "\n\n=== Tests des Environnements Périodiques ===" << std::endl;
        
        // ====================================================================
        // Test 1: Comparaison Environnement classique vs périodique
        // ====================================================================
        std::cout << "\n--- Test 1: Comparaison classique vs périodique ---" << std::endl;
        
        // Environnement classique
        std::cout << "Création environnement classique (20x15):" << std::endl;
        Environnement env_classic = Environnement::createRandomEnvironment({20, 15}, 0.15, 42);
        env_classic.toPNG("test_classic.png");
        
        // Environnement périodique équivalent
        std::cout << "Création environnement périodique (20x15) - X périodique:" << std::endl;
        auto env_periodic_x = PeriodicEnvironnement::createPeriodicRandomEnvironment(
            {20, 15}, {true, false}, 0.15, 42);
        env_periodic_x->toPNG("test_periodic_x.png");
        
        // Environnement périodique complet
        std::cout << "Création environnement périodique (20x15) - X et Y périodiques:" << std::endl;
        auto env_periodic_xy = PeriodicEnvironnement::createPeriodicRandomEnvironment(
            {20, 15}, {true, true}, 0.15, 42);
        env_periodic_xy->toPNG("test_periodic_xy.png");
        
        // ====================================================================
        // Test 2: Dijkstra avec différentes périodicités
        // ====================================================================
        std::cout << "\n--- Test 2: Dijkstra avec périodicité ---" << std::endl;
        
        // Test sur environnement classique
        Dijkstra dijkstra_classic(&env_classic);
        dijkstra_classic.add_start({1.0f, 1.0f});
        dijkstra_classic.execute();
        dijkstra_classic.save_U_values_image("dijkstra_classic.png");
        
        // Test sur environnement périodique X
        Dijkstra dijkstra_periodic_x(env_periodic_x.get());
        dijkstra_periodic_x.add_start({1.0f, 1.0f});
        dijkstra_periodic_x.execute();
        dijkstra_periodic_x.save_U_values_image("dijkstra_periodic_x.png");
        
        // Test sur environnement périodique X+Y
        Dijkstra dijkstra_periodic_xy(env_periodic_xy.get());
        dijkstra_periodic_xy.add_start({1.0f, 1.0f});
        dijkstra_periodic_xy.execute();
        dijkstra_periodic_xy.save_U_values_image("dijkstra_periodic_xy.png");
        
        // ====================================================================
        // Test 3: A* avec heuristiques périodiques
        // ====================================================================
        std::cout << "\n--- Test 3: A* avec heuristiques périodiques ---" << std::endl;
        
        // Créer un environnement de test plus grand
        auto env_astar_periodic = PeriodicEnvironnement::createPeriodicRandomEnvironment(
            {30, 25}, {true, false}, 0.20, 123);
        env_astar_periodic->toPNG("astar_periodic_env.png");
        
        // Test A* avec Manhattan (périodique)
        ManhattanHeuristic manhattan_periodic(1.0f);  // Nouvelle instance
        AStar astar_manhattan_periodic(env_astar_periodic.get(), 1.0f, &manhattan_periodic);
        astar_manhattan_periodic.add_start({2.0f, 2.0f});
        astar_manhattan_periodic.add_end({27.0f, 22.0f});
        astar_manhattan_periodic.execute();
        astar_manhattan_periodic.save_U_values_image("astar_manhattan_periodic.png");
        
        std::cout << "A* Manhattan: " << manhattan_periodic.get_name() << std::endl;
        std::cout << "Description: " << manhattan_periodic.get_description() << std::endl;
        
        // Test A* avec Euclidienne (périodique)
        EuclideanHeuristic euclidean_periodic(1.0f);
        AStar astar_euclidean_periodic(env_astar_periodic.get(), 1.0f, &euclidean_periodic);
        astar_euclidean_periodic.add_start({2.0f, 2.0f});
        astar_euclidean_periodic.add_end({27.0f, 22.0f});
        astar_euclidean_periodic.execute();
        astar_euclidean_periodic.save_U_values_image("astar_euclidean_periodic.png");
        
        std::cout << "A* Euclidienne: " << euclidean_periodic.get_name() << std::endl;
        std::cout << "Description: " << euclidean_periodic.get_description() << std::endl;
        
        // ====================================================================
        // Test 4: Test des calculs de distance périodique
        // ====================================================================
        std::cout << "\n--- Test 4: Calculs de distance périodique ---" << std::endl;
        
        // Créer un petit environnement pour tester les distances
        auto env_distance = std::make_unique<PeriodicEnvironnement>(
            std::vector<int>{10, 8}, std::vector<bool>{true, true});
        
        // Tester les distances
        std::vector<float> point_a = {1.0f, 1.0f};
        std::vector<float> point_b = {9.0f, 7.0f};
        
        float manhattan_dist = env_distance->calculate_distance(point_a, point_b, 1);
        float euclidean_dist = env_distance->calculate_distance(point_a, point_b, 2);
        
        std::cout << "Distance entre (1,1) et (9,7) dans grille 10x8 périodique:" << std::endl;
        std::cout << "  Manhattan: " << manhattan_dist << " (classique serait " << (8+6) << ")" << std::endl;
        std::cout << "  Euclidienne: " << euclidean_dist << " (classique serait " << std::sqrt(8*8 + 6*6) << ")" << std::endl;
        
        // Expliquer les résultats
        std::cout << "En périodique:" << std::endl;
        std::cout << "  X: min(|9-1|, 10-|9-1|) = min(8, 2) = 2" << std::endl;
        std::cout << "  Y: min(|7-1|, 8-|7-1|) = min(6, 2) = 2" << std::endl;
        std::cout << "  Manhattan périodique: 2 + 2 = 4" << std::endl;
        std::cout << "  Euclidienne périodique: sqrt(2² + 2²) = " << std::sqrt(8) << std::endl;
        
        // ====================================================================
        // Test 5: FMM avec périodicité
        // ====================================================================
        std::cout << "\n--- Test 5: FMM avec périodicité ---" << std::endl;
        
        auto env_fmm_periodic = PeriodicEnvironnement::createPeriodicRandomEnvironment(
            {25, 20}, {true, true}, 0.10, 456);
        env_fmm_periodic->toPNG("fmm_periodic_env.png");
        
        FMM fmm_periodic(env_fmm_periodic.get(), 1.0f);
        fmm_periodic.add_start({12.0f, 10.0f});  // Centre
        fmm_periodic.execute();
        fmm_periodic.save_U_values_image("fmm_periodic.png");
        
        std::cout << "FMM périodique - Propagation depuis le centre" << std::endl;
        
        // ====================================================================
        // Test 6: Labyrinthe périodique
        // ====================================================================
        std::cout << "\n--- Test 6: Labyrinthe périodique ---" << std::endl;
        
        auto maze_periodic = PeriodicEnvironnement::createPeriodicMazeEnvironment(
            {20, 15}, {true, false}, 789);
        maze_periodic->toPNG("maze_periodic.png");
        
        Dijkstra dijkstra_maze_periodic(maze_periodic.get());
        dijkstra_maze_periodic.add_start({0.0f, 0.0f});
        dijkstra_maze_periodic.execute();
        dijkstra_maze_periodic.save_U_values_image("maze_periodic_distances.png");
        
        // ====================================================================
        // Test 7: Multiples sources avec périodicité
        // ====================================================================
        std::cout << "\n--- Test 7: Sources multiples périodiques ---" << std::endl;
        
        auto env_multi_periodic = PeriodicEnvironnement::createPeriodicRandomEnvironment(
            {30, 20}, {true, true}, 0.20, 999);
        
        Dijkstra dijkstra_multi_periodic(env_multi_periodic.get());
        dijkstra_multi_periodic.add_start({5.0f, 5.0f});     // Coin
        dijkstra_multi_periodic.add_start({25.0f, 15.0f});   // Coin opposé
        dijkstra_multi_periodic.add_start({15.0f, 10.0f});   // Centre
        dijkstra_multi_periodic.execute();
        dijkstra_multi_periodic.save_U_values_image("multi_sources_periodic.png");

        std::cout << "\n=== Fichiers générés ===" << std::endl;
        std::cout << "Environnements classiques:" << std::endl;
        std::cout << "- environnement_petit.png (20x15, obstacles)" << std::endl;
        std::cout << "- environnement_grand.png (50x40, obstacles)" << std::endl;
        std::cout << "- labyrinthe_petit.png (30x25, labyrinthe)" << std::endl;
        std::cout << "- environnement_astar_manhattan.png (30x25, obstacles)" << std::endl;
        std::cout << "- environnement_astar_euclidean.png (50x50, obstacles)" << std::endl;
        
        std::cout << "\nEnvironnements périodiques:" << std::endl;
        std::cout << "- test_classic.png (environnement classique)" << std::endl;
        std::cout << "- test_periodic_x.png (X périodique)" << std::endl;
        std::cout << "- test_periodic_xy.png (X et Y périodiques)" << std::endl;
        std::cout << "- astar_periodic_env.png (environnement A* périodique)" << std::endl;
        std::cout << "- fmm_periodic_env.png (environnement FMM périodique)" << std::endl;
        std::cout << "- maze_periodic.png (labyrinthe périodique)" << std::endl;
        
        std::cout << "\nCartes de distances:" << std::endl;
        std::cout << "- distances_petit.png (Dijkstra depuis (1,1))" << std::endl;
        std::cout << "- distances_grand.png (Dijkstra depuis le centre)" << std::endl;
        std::cout << "- distances_astar_manhattan.png (A* Manhattan)" << std::endl;
        std::cout << "- distances_astar_euclidean.png (A* Euclidienne)" << std::endl;
        std::cout << "- distances_fmm_basic.png (FMM coût = 1.0)" << std::endl;
        
        std::cout << "\nCartes de distances périodiques:" << std::endl;
        std::cout << "- dijkstra_classic.png (Dijkstra classique)" << std::endl;
        std::cout << "- dijkstra_periodic_x.png (Dijkstra X périodique)" << std::endl;
        std::cout << "- dijkstra_periodic_xy.png (Dijkstra X+Y périodiques)" << std::endl;
        std::cout << "- astar_manhattan_periodic.png (A* Manhattan périodique)" << std::endl;
        std::cout << "- astar_euclidean_periodic.png (A* Euclidienne périodique)" << std::endl;
        std::cout << "- fmm_periodic.png (FMM périodique)" << std::endl;
        std::cout << "- multi_sources_periodic.png (sources multiples périodiques)" << std::endl;
        
        std::cout << "\n=== Observations attendues ===" << std::endl;
        std::cout << "• Environnements périodiques: propagation qui 'wrappe' aux bords" << std::endl;
        std::cout << "• Distances plus courtes grâce aux raccourcis périodiques" << std::endl;
        std::cout << "• Heuristiques A*: automatiquement adaptées à la périodicité" << std::endl;
        std::cout << "• FMM: propagation continue avec conditions aux limites périodiques" << std::endl;
        std::cout << "• Comparaison visuelle: classique vs périodique très instructive!" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Erreur: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}