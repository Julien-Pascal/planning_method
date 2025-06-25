#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include "GraphSearchBase.hpp"

/**
 * @brief Implémentation de l'algorithme de Dijkstra
 * 
 * L'algorithme de Dijkstra trouve le chemin le plus court depuis un ou plusieurs
 * points sources vers tous les autres points accessibles du graphe.
 * 
 * Caractéristiques:
 * - Pas d'heuristique (contrairement à A*)
 * - Garantit le chemin optimal si tous les poids d'arêtes sont positifs
 * - Complexité: O((V + E) log V) où V = vertices, E = edges
 */
class Dijkstra : public GraphSearchBase
{
private:
    float edge_cost; ///< Coût d'une arête (par défaut 1.0 pour grille uniforme)

public:
    /**
     * @brief Constructeur avec coût d'arête par défaut
     * @param environment Pointeur vers l'environnement
     * @param cost Coût d'une arête (défaut: 1.0)
     */
    explicit Dijkstra(Environnement* environment, float cost = 1.0f);
    
    /**
     * @brief Destructeur
     */
    virtual ~Dijkstra() = default;
    
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

protected:
    /**
     * @brief Calcule la nouvelle valeur U pour un voisin selon Dijkstra
     * Pour Dijkstra: nouvelle_valeur = valeur_actuelle + coût_arête
     * @param current Point actuel
     * @param neighbor Point voisin (non utilisé dans Dijkstra basique)
     * @return Nouvelle valeur U
     */
    virtual float calculate_new_value(const Point* current, const Point* neighbor) override;
};

#endif // DIJKSTRA_HPP