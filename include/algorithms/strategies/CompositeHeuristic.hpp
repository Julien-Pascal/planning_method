#ifndef COMPOSITEHEURISTIC_HPP
#define COMPOSITEHEURISTIC_HPP

#include "HeuristicStrategy.hpp"
#include <vector>
#include <memory>
#include <algorithm>

/**
 * @brief Heuristique composite combinant plusieurs heuristiques
 * 
 * Permet de prendre le max, min, ou moyenne de plusieurs heuristiques
 */
class CompositeHeuristic : public HeuristicStrategy {
public:
    enum CombineMode {
        MAX,     ///< Prendre le maximum (prudent)
        MIN,     ///< Prendre le minimum (optimiste)
        AVERAGE, ///< Prendre la moyenne
        WEIGHTED ///< Moyenne pondérée
    };
    
private:
    std::vector<std::shared_ptr<HeuristicStrategy>> heuristics;
    std::vector<float> weights;
    CombineMode mode;
    
public:
    CompositeHeuristic(CombineMode combine_mode = MAX) : mode(combine_mode) {}
    
    void add_heuristic(std::shared_ptr<HeuristicStrategy> heuristic, float weight = 1.0f) {
        heuristics.push_back(heuristic);
        weights.push_back(weight);
    }
    
    float calculate_heuristic(const Point* from, const std::vector<float>& goal) const override {
        if (heuristics.empty()) return 0.0f;
        
        std::vector<float> values;
        for (const auto& h : heuristics) {
            values.push_back(h->calculate_heuristic(from, goal));
        }
        
        switch (mode) {
            case MAX:
                return *std::max_element(values.begin(), values.end());
                
            case MIN:
                return *std::min_element(values.begin(), values.end());
                
            case AVERAGE: {
                float sum = 0.0f;
                for (float v : values) sum += v;
                return sum / values.size();
            }
            
            case WEIGHTED: {
                float weighted_sum = 0.0f;
                float weight_sum = 0.0f;
                for (size_t i = 0; i < values.size(); ++i) {
                    weighted_sum += values[i] * weights[i];
                    weight_sum += weights[i];
                }
                return weight_sum > 0 ? weighted_sum / weight_sum : 0.0f;
            }
        }
        return 0.0f;
    }
    
    std::string get_name() const override {
        std::string mode_str;
        switch (mode) {
            case MAX: mode_str = "MAX"; break;
            case MIN: mode_str = "MIN"; break;
            case AVERAGE: mode_str = "AVG"; break;
            case WEIGHTED: mode_str = "WEIGHTED"; break;
        }
        return "Composite(" + mode_str + "," + std::to_string(heuristics.size()) + ")";
    }
    
    bool is_admissible() const override {
        // Composite est admissible si toutes ses heuristiques le sont ET mode != MAX
        if (mode == MAX) return false; // MAX peut surestimer
        
        for (const auto& h : heuristics) {
            if (!h->is_admissible()) return false;
        }
        return true;
    }
    
    std::string get_description() const override {
        return "Heuristique composite combinant " + std::to_string(heuristics.size()) + " heuristiques.";
    }
};

#endif // COMPOSITEHEURISTIC_HPP