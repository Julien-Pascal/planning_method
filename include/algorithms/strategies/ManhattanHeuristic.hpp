#ifndef MANHATTANHEURISTIC_HPP
#define MANHATTANHEURISTIC_HPP

#include "HeuristicStrategy.hpp"
#include "../../Environnement.hpp"

/**
 * @brief Heuristique Manhattan (distance L1)
 * 
 * Calcule |x1-x2| + |y1-y2| en tenant compte de la périodicité
 * 
 * USAGE: Grilles où seuls les mouvements orthogonaux sont autorisés
 * PROPRIÉTÉS: Admissible, rapide à calculer, supporte la périodicité
 */
class ManhattanHeuristic : public HeuristicStrategy {
private:
    float weight_factor; ///< Facteur de pondération (1.0 = admissible)
    
public:
    explicit ManhattanHeuristic(float weight = 1.0f, const Environnement* env = nullptr) 
        : HeuristicStrategy(env), weight_factor(weight) 
    {
        if (weight < 1.0f) {
            throw std::invalid_argument("Weight must be >= 1.0 for admissibility");
        }
    }
    
    float calculate_heuristic(const Point* from, const std::vector<float>& goal) const override {
        auto coords = from->get_coords();
        
        if (environment) {
            // Utiliser la méthode de l'environnement (gère automatiquement la périodicité)
            return weight_factor * environment->calculate_distance(coords, goal, 1);
        } else {
            // Fallback vers calcul classique
            float sum = 0;
            for (size_t i = 0; i < coords.size(); ++i) {
                sum += std::abs(coords[i] - goal[i]);
            }
            return weight_factor * sum;
        }
    }
    
    std::string get_name() const override { 
        return "Manhattan(w=" + std::to_string(weight_factor) + 
               (environment ? ",periodic" : "") + ")"; 
    }
    
    bool is_admissible() const override { 
        return weight_factor <= 1.0f; 
    }
    
    std::string get_description() const override {
        return "Distance Manhattan |dx| + |dy|. Idéale pour mouvements orthogonaux (4-connectivité). " +
               std::string(environment ? "Supporte la périodicité." : "");
    }
    
    void set_weight(float weight) { weight_factor = weight; }
    float get_weight() const { return weight_factor; }
};

#endif // MANHATTANHEURISTIC_HPP