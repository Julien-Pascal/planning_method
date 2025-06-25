#ifndef OCTILEHEURISTIC_HPP
#define OCTILEHEURISTIC_HPP

#include "HeuristicStrategy.hpp"
#include <algorithm>
#include <cmath>

/**
 * @brief Heuristique Octile (optimale pour grilles 8-connectées)
 * 
 * Combine mouvements diagonaux (coût √2) et orthogonaux (coût 1)
 * 
 * USAGE: Grilles 8-connectées avec coûts réalistes
 * PROPRIÉTÉS: Admissible, plus précise que Manhattan et Diagonal
 */
class OctileHeuristic : public HeuristicStrategy {
private:
    float diagonal_cost;   ///< Coût diagonal (typiquement √2 ≈ 1.414)
    float orthogonal_cost; ///< Coût orthogonal (typiquement 1.0)
    
public:
    OctileHeuristic(float diag_cost = std::sqrt(2.0f), float ortho_cost = 1.0f) 
        : diagonal_cost(diag_cost), orthogonal_cost(ortho_cost) {}
    
    float calculate_heuristic(const Point* from, const std::vector<float>& goal) const override {
        auto coords = from->get_coords();
        
        float sum_abs_diff = 0.0f;
        float max_abs_diff = 0.0f;

        // Calculate the sum of absolute differences and the maximum absolute difference
        for (size_t i = 0; i < coords.size(); ++i) {
            float diff = std::abs(coords[i] - goal[i]);
            sum_abs_diff += diff;
            if (diff > max_abs_diff) {
                max_abs_diff = diff;
            }
        }
        return diagonal_cost * max_abs_diff + orthogonal_cost * (sum_abs_diff - max_abs_diff);
    }
    
    std::string get_name() const override { 
        return "Octile(d=" + std::to_string(diagonal_cost) + ",o=" + std::to_string(orthogonal_cost) + ")"; 
    }
    
    bool is_admissible() const override { 
        return diagonal_cost >= std::sqrt(2.0f) && orthogonal_cost >= 1.0f; 
    }
    
    std::string get_description() const override {
        return "Heuristique octile optimale pour grilles 8-connectées. Combine diagonales (√2) et orthogonales (1).";
    }
};

#endif // OCTILEHEURISTIC_HPP