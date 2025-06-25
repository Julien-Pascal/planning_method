#ifndef FMM_HPP
#define FMM_HPP

#include "GraphSearchBase.hpp"
#include <limits>   // For std::numeric_limits
#include <algorithm> // For std::sort
#include <cmath>    // For std::sqrt

/**
 * @brief Implémentation de l'algorithme Fast Marching Method (FMM)
 *
 * Le FMM calcule la carte des temps de parcours (ou distances)
 * depuis un ou plusieurs points sources en résolvant numériquement l'équation Eikonal
 * $|\nabla T| = F^{-1}$. Il est idéal pour générer des champs de distance.
 *
 * Il partage des similarités avec Dijkstra mais utilise une mise à jour locale
 * basée sur l'équation Eikonal, ce qui le rend plus précis pour les fronts de propagation.
 */
class FMM : public GraphSearchBase
{
private:
    float cost_per_unit_distance; ///< Coût par unité de distance (l'inverse de la vitesse locale, F^-1)

public:
    /**
     * @brief Constructeur
     * @param environment Pointeur vers l'environnement
     * @param cost Coût par unité de distance (défaut: 1.0, pour une propagation de vitesse 1)
     */
    explicit FMM(Environnement* environment, float cost = 1.0f);

    /**
     * @brief Destructeur
     */
    virtual ~FMM() = default;

    /**
     * @brief Modifie le coût par unité de distance pour la propagation
     * @param cost Nouveau coût. Doit être positif.
     */
    void set_cost_per_unit_distance(float cost);

    /**
     * @brief Obtient le coût actuel par unité de distance
     * @return Coût
     */
    float get_cost_per_unit_distance() const;

protected:
    /**
     * @brief Calcule la nouvelle valeur U (temps/distance) pour un voisin selon FMM.
     * Cette méthode applique la solution de l'équation Eikonal sur grille.
     * Elle examine les valeurs U des voisins déjà FROZEN (finalisés) du point voisin
     * pour déterminer sa nouvelle valeur en résolvant une équation quadratique
     * ou en appliquant une mise à jour linéaire.
     *
     * @param current Point actuel (le point dont la valeur U vient d'être finalisée)
     * @param neighbor Point voisin dont la valeur U est à calculer/mettre à jour
     * @return Nouvelle valeur U calculée pour le voisin
     */
    virtual float calculate_new_value(const Point* current, const Point* neighbor) override;


    float solve_eikonal(const std::vector<Point*>&);
};

#endif // FMM_HPP