#ifndef DIAGONALHEURISTIC_HPP
#define DIAGONALHEURISTIC_HPP

#include "HeuristicStrategy.hpp"
#include <algorithm>

/**
 * @brief Heuristique Diagonale (Chebyshev)
 * 
 * Calcule max(|dx|, |dy|) pour diagonales + orthogonales
 * 
 * USAGE: Grilles 8-connectées où diagonales coûtent 1 (comme orthogonales)
 * PROPRIÉTÉS: Admissible pour grilles 8-connectées uniformes
 */
class DiagonalHeuristic : public HeuristicStrategy {
private:
    float diagonal_cost;   ///< Coût déplacement diagonal
    float orthogonal_cost; ///< Coût déplacement orthogonal
    
public:
    DiagonalHeuristic(float diag_cost = 1.0f, float ortho_cost = 1.0f) 
        : diagonal_cost(diag_cost), orthogonal_cost(ortho_cost) {}
    
    float calculate_heuristic(const Point* from, const std::vector<float>& goal) const override {
        auto coords = from->get_coords();
        
        float res = 0;
        for (size_t i = 0 ; i<coords.size() ; i++)
        {
            res = std::max(res,std::abs(coords[i] - goal[i]));
        }
        
        // Chebyshev: maximum des deux distances
        return res * diagonal_cost;
    }
    
    std::string get_name() const override { 
        return "Diagonal(d=" + std::to_string(diagonal_cost) + ",o=" + std::to_string(orthogonal_cost) + ")"; 
    }
    
    bool is_admissible() const override { 
        return diagonal_cost >= 1.0f && orthogonal_cost >= 1.0f; 
    }
    
    std::string get_description() const override {
        return "Distance Chebyshev max(|dx|, |dy|). Idéale pour grilles 8-connectées avec coût diagonal uniforme.";
    }
};

#endif // DIAGONALHEURISTIC_HPP