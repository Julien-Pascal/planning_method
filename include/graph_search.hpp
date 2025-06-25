#ifndef GRAPH_SEARCH_HPP
#define GRAPH_SEARCH_HPP

#include "Point.hpp"
#include "Environnement.hpp"
#include "utils/Comparison.hpp"
#include <queue>
#include <vector>
#include <string>

class graph_search
{
private:
    Environnement* env; 
    std::priority_queue<Point*, std::vector<Point*>, Compare> front;
    std::vector<std::vector<float>> starts;
    std::vector<std::vector<float>> ends;

public:
    graph_search(Environnement* environment);
    
    // Gestion des points de départ et d'arrivée
    void add_start(const std::vector<float>& coords);
    void add_end(const std::vector<float>& coords);
    void clear_starts();
    void clear_ends();
    
    // Réinitialisation et exécution
    void reset_environment();
    void execute();
    
    // Affichage et sauvegarde des résultats
    void display_U_values_grid() const;
    void save_U_values_image(const std::string& filename) const;
};

#endif // GRAPH_SEARCH_HPP