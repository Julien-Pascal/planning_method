// 3. Modification de EuclideanHeuristic.hpp - Utiliser l'environnement
#ifndef EUCLIDEANHEURISTIC_HPP
#define EUCLIDEANHEURISTIC_HPP

#include "HeuristicStrategy.hpp"
#include "../../Environnement.hpp"
#include <cmath>

/**
 * @brief Heuristique Euclidienne (distance L2)
 * 
 * Calcule sqrt((x1-x2)² + (y1-y2)² + (z1-z2)² + ...) en tenant compte de la périodicité
 * 
 * USAGE: Environnements où mouvement dans toutes directions possible
 * PROPRIÉTÉS: Admissible, plus précise que Manhattan pour mouvements libres, supporte la périodicité
 */
class EuclideanHeuristic : public HeuristicStrategy {
private:
    float weight_factor;
    
public:
    explicit EuclideanHeuristic(float weight = 1.0f, const Environnement* env = nullptr) 
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
            return weight_factor * environment->calculate_distance(coords, goal, 2);
        } else {
            // Fallback vers calcul classique
            float sum = 0;
            for (size_t i = 0; i < coords.size(); ++i) {
                float diff = coords[i] - goal[i];
                sum += diff * diff;
            }
            return weight_factor * std::sqrt(sum);
        }
    }
    
    std::string get_name() const override { 
        return "Euclidean(w=" + std::to_string(weight_factor) + 
               (environment ? ",periodic" : "") + ")"; 
    }
    
    bool is_admissible() const override { 
        return weight_factor <= 1.0f; 
    }
    
    std::string get_description() const override {
        return "Distance euclidienne sqrt(dx² + dy²). Idéale pour mouvements libres en toutes directions. " +
               std::string(environment ? "Supporte la périodicité." : "");
    }
    
    void set_weight(float weight) { weight_factor = weight; }
    float get_weight() const { return weight_factor; }
};

#endif // EUCLIDEANHEURISTIC_HPP