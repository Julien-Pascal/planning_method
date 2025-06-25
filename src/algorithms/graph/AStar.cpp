#include "algorithms/graph/AStar.hpp"
#include <cmath> 
#include <limits> 
#include <stdexcept> 

AStar::AStar(Environnement* environment, float cost, HeuristicStrategy* heuristic_strat)
    : GraphSearchBase(environment), edge_cost(cost), heuristic_strategy(heuristic_strat)
{}

// Méthodes pour gérer le coût d'arête
void AStar::set_edge_cost(float cost) {
    edge_cost = cost;
}

float AStar::get_edge_cost() const {
    return edge_cost;
}

// Définit la stratégie d'heuristique à utiliser
void AStar::set_heuristic_strategy(HeuristicStrategy* heuristic_strat) {
    if (!heuristic_strat) {
        throw std::invalid_argument("AStar::set_heuristic_strategy: heuristic_strat cannot be nullptr.");
    }
    heuristic_strategy = heuristic_strat;
}

// Obtient la stratégie d'heuristique
const HeuristicStrategy* AStar::get_heuristic_strategy() const {
    return heuristic_strategy;
}

// Calcule la nouvelle valeur U (f(n)) pour A*
float AStar::calculate_new_value(const Point* current, const Point* neighbor) {
    if (!heuristic_strategy) {
        // En l'absence de stratégie d'heuristique, A* se comporte comme Dijkstra
        // ou vous pouvez lancer une exception si une stratégie est obligatoire.
        // Pour cet exemple, nous allons le faire se comporter comme Dijkstra (h(n) = 0).
        return current->get_value() + edge_cost;
        // Ou: throw std::runtime_error("AStar::calculate_new_value: Heuristic strategy not set.");
    }

    // g(n) = coût du chemin du départ à 'current' + coût de 'current' à 'neighbor'
    float g_n = current->get_value() + edge_cost;

    // h(n) = heuristique du 'neighbor' vers le point d'arrivée le plus proche
    std::vector<float> closest_end_coords = find_closest_end_coords(neighbor);
    float h_n = 0.0f;
    if (!closest_end_coords.empty()) {
        if(heuristic_strategy != nullptr)
        h_n = heuristic_strategy->calculate_heuristic(neighbor, closest_end_coords);
        else h_n = 0;
    }

    // f(n) = g(n) + h(n)
    return g_n + h_n;
}

// Détermine si l'algorithme A* doit continuer
bool AStar::should_continue() const {
    if (front.empty()) {
        return false;
    }

    // A* s'arrête si le point avec la plus faible valeur U (qui est f(n)) est un point d'arrivée.
    const Point* next_point = front.top();

    // Récupérer les coordonnées du point actuellement en tête de la file de priorité
    const std::vector<float>& next_point_coords = next_point->get_coords();

    for (const auto& end_coords : ends) {

        bool match = true;
        for (size_t i = 0; i < end_coords.size(); ++i) {
            // Comparer chaque coordonnée.
            // On utilise static_cast<int> car les coordonnées de grille sont généralement des entiers.
            if (static_cast<int>(end_coords[i]) != static_cast<int>(next_point_coords[i])) {
                match = false;
                break;
            }
        }

        if (match) {
            return false; // Point d'arrivée atteint et est le prochain à être traité
        }
    }
    return true; // Continuer la recherche
}

// Trouve les coordonnées du point d'arrivée le plus proche d'un point donné
std::vector<float> AStar::find_closest_end_coords(const Point* p) const {
    if (ends.empty()) {
        return {}; // Retourne un vecteur vide si aucun point d'arrivée n'est défini
    }

    std::vector<float> closest_coords;
    float min_dist_sq = std::numeric_limits<float>::max();

    const std::vector<float>& p_coords = p->get_coords();

    for (const auto& end_coord_vec : ends) {

        float current_dist_sq = 0.0f;
        for (size_t i = 0; i < end_coord_vec.size(); ++i) {
            float diff = p_coords[i] - end_coord_vec[i];
            current_dist_sq += diff * diff; // Calcul de la distance euclidienne au carré
        }

        if (current_dist_sq < min_dist_sq) {
            min_dist_sq = current_dist_sq;
            closest_coords = end_coord_vec;
        }
    }
    return closest_coords;
}