#ifndef HEURISTICSTRATEGY_HPP
#define HEURISTICSTRATEGY_HPP

#include "../../Point.hpp"
#include <vector>
#include <string>
#include <stdexcept>

// Forward declaration pour éviter la dépendance circulaire
class Environnement;

/**
 * @brief Interface pour les stratégies d'heuristique A*
 * 
 * Chaque heuristique doit estimer la distance restante vers le goal.
 * L'heuristique doit être admissible (jamais surestimer) pour garantir l'optimalité.
 */
class HeuristicStrategy {
protected:
    const Environnement* environment; ///< Pointeur vers l'environnement (pour accéder aux méthodes de distance)

public:
    explicit HeuristicStrategy(const Environnement* env = nullptr) : environment(env) {}
    virtual ~HeuristicStrategy() = default;
    
    /**
     * @brief Définit l'environnement à utiliser pour les calculs de distance
     */
    void set_environment(const Environnement* env) { environment = env; }
    
    /**
     * @brief Calcule l'heuristique h(n) = distance estimée vers le goal
     * @param from Point de départ
     * @param goal Coordonnées du goal
     * @return Distance estimée (doit être admissible)
     */
    virtual float calculate_heuristic(const Point* from, const std::vector<float>& goal) const = 0;
    
    /**
     * @brief Nom de l'heuristique (pour debug/logs)
     */
    virtual std::string get_name() const = 0;
    
    /**
     * @brief Indique si l'heuristique est admissible
     * @return true si admissible (garantit optimalité)
     */
    virtual bool is_admissible() const = 0;
    
    /**
     * @brief Description de l'heuristique et ses cas d'usage
     */
    virtual std::string get_description() const = 0;
};

#endif // HEURISTICSTRATEGY_HPP