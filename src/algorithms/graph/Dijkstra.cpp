#include "algorithms/graph/Dijkstra.hpp"

Dijkstra::Dijkstra(Environnement* environment, float cost) 
    : GraphSearchBase(environment), edge_cost(cost) {}

void Dijkstra::set_edge_cost(float cost) {
    edge_cost = cost;
}

float Dijkstra::get_edge_cost() const {
    return edge_cost;
}

float Dijkstra::calculate_new_value(const Point* current, const Point* neighbor) {
    // Pour Dijkstra basique: nouvelle valeur = valeur actuelle + coût d'arête constant
    // Dans le futur, on pourrait calculer la distance euclidienne réelle entre current et neighbor
    // ou utiliser des coûts d'arêtes variables selon le terrain
    
    (void)neighbor; // Éviter warning unused parameter pour l'instant
    
    return current->get_value() + edge_cost;
}