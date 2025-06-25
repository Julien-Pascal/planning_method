#ifndef PERIODICENVIRONNEMENT_HPP
#define PERIODICENVIRONNEMENT_HPP

#include "Environnement.hpp"
#include <memory>

/**
 * @brief Environnement avec dimensions périodiques (conditions aux limites périodiques)
 * 
 * Cette classe étend Environnement pour supporter des dimensions périodiques.
 * Dans une dimension périodique, les bords opposés sont connectés (topologie torique).
 * 
 * Exemple d'usage :
 * - Dimension X périodique : le point (0, y) est voisin de (width-1, y)
 * - Dimension Y périodique : le point (x, 0) est voisin de (x, height-1)
 */
class PeriodicEnvironnement : public Environnement
{
private:
    std::vector<bool> periodic_dims;  ///< true si la dimension correspondante est périodique

public:
    /**
     * @brief Constructeur par défaut
     */
    PeriodicEnvironnement();
    
    /**
     * @brief Constructeur avec dimensions et périodicité
     * @param dimensions Tailles de chaque dimension
     * @param periodic Vecteur de booléens indiquant la périodicité de chaque dimension
     */
    PeriodicEnvironnement(const std::vector<int>& dimensions, const std::vector<bool>& periodic);
    
    /**
     * @brief Constructeur de copie depuis un Environnement classique
     * @param env Environnement source
     * @param periodic Vecteur de périodicité
     */
    PeriodicEnvironnement(const Environnement& env, const std::vector<bool>& periodic);

    /**
     * @brief Définit quelles dimensions sont périodiques
     * @param periodic Vecteur de booléens (même taille que le nombre de dimensions)
     */
    void set_periodic_dims(const std::vector<bool>& periodic);
    
    /**
     * @brief Vérifie si une dimension spécifique est périodique
     * @param dim_index Index de la dimension (0-indexé)
     * @return true si la dimension est périodique
     */
    bool is_periodic(int dim_index) const;
    
    /**
     * @brief Obtient le vecteur de périodicité
     * @return Vecteur de booléens de périodicité
     */
    std::vector<bool> get_periodic_dims() const;

    // Override des méthodes virtuelles de la classe de base
    
    /**
     * @brief Vérifie si des coordonnées sont dans les limites (périodiques ou non)
     * Pour les dimensions périodiques, toujours vrai. Pour les non-périodiques, teste les bornes.
     */
    bool is_in_bounds(const std::vector<float>& coords) const override;
    
    /**
     * @brief Obtient les voisins d'un point en tenant compte de la périodicité
     * Les voisins peuvent "wraparound" aux bords dans les dimensions périodiques.
     */
    std::vector<Point*> get_neigh(const Point& pt) override;
    
    /**
     * @brief Obtient les coins d'hypercube avec pondération pour interpolation périodique
     * Gère les hypercubes qui se chevauchent aux bords périodiques.
     */
    std::vector<std::pair<Point*, float>> get_hypercube_corners_with_weights(const std::vector<float>& coords) const override;
    
    /**
     * @brief Calcule la distance entre deux points en tenant compte de la périodicité
     * @param a Premier point
     * @param b Deuxième point  
     * @param norm_type Type de norme (1=Manhattan, 2=Euclidienne)
     * @return Distance minimale (peut passer par les bords périodiques)
     */
    float calculate_distance(const std::vector<float>& a, const std::vector<float>& b, int norm_type = 2) const override;

    // Factory methods statiques pour créer des environnements périodiques
    
    /**
     * @brief Crée un environnement aléatoire avec périodicité
     */
    static std::unique_ptr<PeriodicEnvironnement> createPeriodicRandomEnvironment(
        const std::vector<int>& dimensions,
        const std::vector<bool>& periodic,
        double obstacle_probability = 0.3,
        unsigned int seed = 0);
    
    /**
     * @brief Crée un labyrinthe avec périodicité
     */
    static std::unique_ptr<PeriodicEnvironnement> createPeriodicMazeEnvironment(
        const std::vector<int>& dimensions,
        const std::vector<bool>& periodic,
        unsigned int seed = 0);

private:
    /**
     * @brief Normalise une coordonnée selon la périodicité
     * @param coord Coordonnée à normaliser
     * @param dim_index Index de la dimension
     * @return Coordonnée normalisée dans [0, dim_size)
     */
    float normalize_coordinate(float coord, int dim_index) const;
    
    /**
     * @brief Normalise un vecteur de coordonnées
     * @param coords Coordonnées à normaliser
     * @return Coordonnées normalisées
     */
    std::vector<float> normalize_coords(const std::vector<float>& coords) const;
    
    /**
     * @brief Calcule la distance minimale entre deux coordonnées dans une dimension
     * @param a Première coordonnée
     * @param b Deuxième coordonnée
     * @param dim_size Taille de la dimension
     * @param is_periodic true si la dimension est périodique
     * @return Distance minimale
     */
    float calculate_dimension_distance(float a, float b, int dim_size, bool is_periodic) const;
    
    /**
     * @brief Valide que le vecteur de périodicité correspond aux dimensions
     */
    void validate_periodic_dims() const;
};

#endif // PERIODICENVIRONNEMENT_HPP