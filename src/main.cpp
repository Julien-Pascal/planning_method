#include "Point.hpp"
#include "Environnement.hpp"
#include "algorithms/graph/Dijkstra.hpp"
#include "algorithms/graph/AStar.hpp"
#include "algorithms/graph/FMM.hpp"
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
        Environnement env_multi = Environnement::createRandomEnvironment({25, 20}, 0.25, 789);
        
        Dijkstra searcher_multi(&env_multi);
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
        Environnement env_astar_euclidean = Environnement::createMazeEnvironment({40, 30}, 222);
        env_astar_euclidean.toPNG("environnement_astar_euclidean.png");

        AStar astar_euclidean(&env_astar_euclidean, 1.0f, &euclidean_strat); // Heuristique Euclidienne
        astar_euclidean.add_start({0.0f, 0.0f});
        astar_euclidean.add_end({22.0f, 16.0f}); // Point d'arrivée nécessaire
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
        

        std::cout << "\n=== Fichiers générés ===" << std::endl;
        std::cout << "Environnements:" << std::endl;
        std::cout << "- environnement_petit.png (20x15, obstacles)" << std::endl;
        std::cout << "- environnement_grand.png (50x40, obstacles)" << std::endl;
        std::cout << "- labyrinthe_petit.png (30x25, labyrinthe)" << std::endl;
        std::cout << "- environnement_astar_manhattan.png (30x25, obstacles)" << std::endl;
        std::cout << "- environnement_astar_euclidean.png (40x30, labyrinthe)" << std::endl;
        std::cout << "- environnement_astar_multi_end.png (35x30, obstacles)" << std::endl;
        std::cout << "- environnement_fmm_basic.png (25x20, obstacles)" << std::endl;
        std::cout << "- environnement_fmm_maze.png (30x25, labyrinthe)" << std::endl;
        std::cout << "- environnement_fmm_multi.png (30x25, obstacles)" << std::endl;
        std::cout << "- environnement_fmm_target.png (35x30, obstacles)" << std::endl;
        std::cout << "- environnement_comparison.png (20x15, obstacles)" << std::endl;
        
        std::cout << "\nCartes de distances (U_values) - Algorithme Dijkstra:" << std::endl;
        std::cout << "- distances_petit.png (distances depuis (1,1))" << std::endl;
        std::cout << "- distances_grand.png (distances depuis le centre)" << std::endl;
        std::cout << "- distances_labyrinthe.png (distances dans le labyrinthe)" << std::endl;
        std::cout << "- distances_multi_sources.png (3 points de départ)" << std::endl;
        std::cout << "- distances_cout_25.png (coût d'arête = 2.5)" << std::endl;
        std::cout << "- distances_comparison_dijkstra.png (comparaison avec FMM)" << std::endl;
        
        std::cout << "\nCartes de distances (U_values) - Algorithme A*:" << std::endl;
        std::cout << "- distances_astar_manhattan.png (A* avec Manhattan)" << std::endl;
        std::cout << "- distances_astar_euclidean.png (A* avec Euclidienne)" << std::endl;
        std::cout << "- distances_astar_multi_end.png (A* avec multiples destinations)" << std::endl;
        
        std::cout << "\nCartes de distances (U_values) - Algorithme FMM:" << std::endl;
        std::cout << "- distances_fmm_basic.png (FMM coût = 1.0)" << std::endl;
        std::cout << "- distances_fmm_high_cost.png (FMM coût = 3.0)" << std::endl;
        std::cout << "- distances_fmm_maze.png (FMM dans labyrinthe)" << std::endl;
        std::cout << "- distances_fmm_multi.png (FMM 3 sources, coût = 1.5)" << std::endl;
        std::cout << "- distances_fmm_target.png (FMM avec arrêt anticipé)" << std::endl;
        std::cout << "- distances_comparison_fmm.png (comparaison avec Dijkstra)" << std::endl;
        std::cout << "- distances_fmm_fast.png (FMM coût = 0.5)" << std::endl;
        
        std::cout << "\nCode couleur des distances: Bleu (proche) -> Vert -> Jaune -> Rouge (loin)" << std::endl;
        std::cout << "Noir = obstacle, Gris foncé = inaccessible" << std::endl;
        
        std::cout << "\n=== Différences entre les algorithmes ===" << std::endl;
        std::cout << "• Dijkstra: Distances discrètes sur graphe (sauts d'arête en arête)" << std::endl;
        std::cout << "• A*: Dijkstra avec heuristique pour atteindre une cible rapidement" << std::endl;
        std::cout << "• FMM: Résolution de l'équation Eikonal, propagation continue" << std::endl;
        std::cout << "  → FMM donne des distances plus 'lisses' et physiquement réalistes" << std::endl;
        std::cout << "  → FMM permet de modéliser des vitesses de propagation variables" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Erreur: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}