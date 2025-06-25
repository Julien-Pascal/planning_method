#include "algorithms/graph/FMM.hpp"
#include "Utils/Comparison.hpp"
#include <numeric>


FMM::FMM(Environnement* environment, float distance_cost) 
    : GraphSearchBase(environment), cost_per_unit_distance(distance_cost) {}

void FMM::set_cost_per_unit_distance(float distance_cost) {
    cost_per_unit_distance = distance_cost;
}

float FMM::get_cost_per_unit_distance() const {
    return cost_per_unit_distance;
}

float FMM::calculate_new_value(const Point* current, const Point* neighbor) {

    //(void)neighbor; // Éviter warning unused parameter pour l'instant
    (void)current;
    auto neighs = env->get_neigh(*neighbor);
    auto coords = neighbor->get_coords();
    std::vector<Point*> contributing_neighs;
    for (size_t i = 0; i < neighs.size(); i+=2)
    {
        contributing_neighs.push_back(std::min(neighs[i],neighs[i+1],
            [](Point* a, Point* b) { return a->get_value() < b->get_value(); }));
    }
    return solve_eikonal(contributing_neighs);
}



// // Détermine si l'algorithme FMM doit continuer
// bool FMM::should_continue() const {
//     if (front.empty()) {
//         return false;
//     }

//     // A* s'arrête si le point avec la plus faible valeur U (qui est f(n)) est un point d'arrivée.
//     const Point* next_point = front.top();

//     // Récupérer les coordonnées du point actuellement en tête de la file de priorité
//     const std::vector<float>& next_point_coords = next_point->get_coords();

//     for (const auto& end_coords : ends) {

//         bool match = true;
//         for (size_t i = 0; i < end_coords.size(); ++i) {
//             // Comparer chaque coordonnée.
//             // On utilise static_cast<int> car les coordonnées de grille sont généralement des entiers.
//             if (static_cast<int>(end_coords[i]) != static_cast<int>(next_point_coords[i])) {
//                 match = false;
//                 break;
//             }
//         }

//         if (match) {
//             return false; // Point d'arrivée atteint et est le prochain à être traité
//         }
//     }
//     return true; // Continuer la recherche
// }


float FMM::solve_eikonal(const std::vector<Point*>& contributing_neighs) {

    float a = static_cast<float>(contributing_neighs.size());

    // Calculate the sum of U_value
    float sum = std::accumulate(contributing_neighs.begin(), contributing_neighs.end(), 0.0f,
        [](float total, Point* point) {
            return total + point->get_value();
        });

    float b = -2 * sum;

    // Calculate the sum of squares of U_value
    float sum_of_squares = std::accumulate(contributing_neighs.begin(), contributing_neighs.end(), 0.0f,
        [](float total, Point* point) {
            float value = point->get_value();
            return total + value * value;
        });

    float c = sum_of_squares - cost_per_unit_distance * cost_per_unit_distance;


    float delta = b * b - 4 * a * c;

    if (delta >= 0 && !std::isinf(delta)) {
        float result = (-b + std::sqrt(delta)) / (2 * a);
        return result;
    } else {
        float min_value = (*std::min_element(contributing_neighs.begin(), contributing_neighs.end(),
            [](Point* a, Point* b) { return a->get_value() < b->get_value(); }))->get_value();
        float result = min_value + cost_per_unit_distance;
        return result;
    }
}