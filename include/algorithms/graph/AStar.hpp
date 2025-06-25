#ifndef ASTAR_HPP
#define ASTAR_HPP

#include "GraphSearchBase.hpp"
#include "../strategies/HeuristicStrategy.hpp"
#include "../strategies/CompositeHeuristic.hpp"
#include "../strategies/DiagonalHeuristic.hpp"
#include "../strategies/EuclideanHeuristic.hpp"
#include "../strategies/ManhattanHeuristic.hpp"
#include "../strategies/OctileHeuristic.hpp"
#include <memory>

/**
 * @brief Implémentation de l'algorithme A*
 *
 * L'algorithme A* est un algorithme de recherche de chemin qui étend Dijkstra
 * en utilisant une fonction heuristique pour guider sa recherche et trouver
 * le chemin le plus court de manière plus efficace.
 *
 * Caractéristiques:
 * - Utilise une fonction d'évaluation f(n) = g(n) + h(n)
 * où g(n) est le coût du chemin du point de départ à n,
 * et h(n) est le coût heuristique estimé du point n au point d'arrivée.
 * - Garantit le chemin optimal si l'heuristique est admissible (ne surestime jamais le coût réel).
 * - Plus efficace que Dijkstra pour des graphes de grande taille avec une bonne heuristique.
 */
class AStar : public GraphSearchBase
{
private:
    float edge_cost; ///< Coût d'une arête (par défaut 1.0 pour grille uniforme)
    HeuristicStrategy* heuristic_strategy; ///< Pointeur vers la stratégie d'heuristique actuelle

public:
    /**
     * @brief Constructeur
     * @param environment Pointeur vers l'environnement
     * @param cost Coût d'une arête (défaut: 1.0)
     * @param heuristic_strat Pointeur vers l'objet stratégie d'heuristique (doit être géré par l'appelant)
     */
    explicit AStar(Environnement* environment, float cost = 1.0f, HeuristicStrategy* heuristic_strat = nullptr);

    /**
     * @brief Destructeur
     * Note: La stratégie d'heuristique n'est PAS supprimée par ce destructeur.
     * Elle doit être gérée par l'appelant pour permettre le partage de stratégies.
     */
    virtual ~AStar() = default;

    /**
     * @brief Modifie le coût d'une arête
     * @param cost Nouveau coût d'arête
     */
    void set_edge_cost(float cost);

    /**
     * @brief Obtient le coût actuel d'une arête
     * @return Coût d'arête
     */
    float get_edge_cost() const;

    /**
     * @brief Définit la stratégie d'heuristique à utiliser pour le calcul A*
     * @param heuristic_strat Pointeur vers la nouvelle stratégie d'heuristique
     */
    void set_heuristic_strategy(HeuristicStrategy* heuristic_strat);

    /**
     * @brief Obtient un pointeur vers la stratégie d'heuristique actuellement utilisée.
     * @return Pointeur vers la stratégie d'heuristique. Peut être nullptr si aucune n'est définie.
     */
    const HeuristicStrategy* get_heuristic_strategy() const;

protected:
    /**
     * @brief Calcule la nouvelle valeur U (f(n)) pour un voisin selon A*
     * Pour A*: f(n) = g(n) + h(n)
     * g(n) = valeur_actuelle (du parent) + coût_arête
     * h(n) = heuristique(voisin, point_d_arrivee_le_plus_proche)
     * @param current Point actuel (parent)
     * @param neighbor Point voisin
     * @return Nouvelle valeur U (f(n))
     */
    virtual float calculate_new_value(const Point* current, const Point* neighbor) override;

    /**
     * @brief Détermine si l'algorithme A* doit continuer
     * S'arrête lorsque le point d'arrivée est atteint ou la frontière est vide.
     * @return true si l'algorithme doit continuer
     */
    virtual bool should_continue() const override;

private:
    /**
     * @brief Trouve le point d'arrivée le plus proche d'un point donné
     * Utilisé pour les heuristiques. Retourne les coordonnées du goal, pas le Point*.
     * @param p Le point à partir duquel chercher l'arrivée la plus proche
     * @return Les coordonnées du point d'arrivée le plus proche sous forme de std::vector<float>.
     * Retourne un vecteur vide si aucun point d'arrivée n'est défini.
     */
    std::vector<float> find_closest_end_coords(const Point* p) const;
};

#endif // ASTAR_HPP